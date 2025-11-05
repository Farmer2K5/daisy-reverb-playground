/**
 * @file example_lush_plate_v3.cpp
 * @brief Example wiring for LushPlate on Daisy Seed (48 kHz).
 *
 * Requirements:
 *   - delaybuffer_ext.h
 *   - firefly_delay_perf.h
 *   - lush_plate_v3.h (this header)
 */

#include "daisy_pod.h"
#include "daisysp.h"
#include "lush_plate_reverb.h"

using namespace daisy;
using namespace daisysp;
using namespace daisyfarm;

static DaisyPod hw;
static CpuLoadMeter cpuLoadMeter;

constexpr size_t NLINES = 4;
constexpr size_t NFF = 16;

// Buffer sizes (tweakable)
constexpr size_t ER_MAX = 8192;     // per channel (~170 ms @ 48 k)
constexpr size_t DIFF_MAX = 16384;  // per diffuser
constexpr size_t PRE_MAX = 16384;   // up to ~341 ms pre-delay headroom
constexpr size_t TANK_MAX = 65536;  // per tank line (~1.36 s)
constexpr size_t CLOUD_MAX = 65536; // firefly buffer

// External buffers (SDRAM recommended)
DSY_SDRAM_BSS float erL[ER_MAX], erR[ER_MAX];
DSY_SDRAM_BSS float ap0[DIFF_MAX], ap1[DIFF_MAX];
DSY_SDRAM_BSS float prebuf[PRE_MAX];
DSY_SDRAM_BSS float tank[NLINES][TANK_MAX];
DSY_SDRAM_BSS float cloud[CLOUD_MAX];

// Instance
LushPlate<float, ER_MAX, DIFF_MAX, PRE_MAX, TANK_MAX, CLOUD_MAX, NLINES, NFF> plate;

void Plate_Init(float sr_)
{
    float *tbufs[NLINES];
    for (size_t i = 0; i < NLINES; ++i)
        tbufs[i] = tank[i];

    plate.Init(erL, erR, ap0, ap1, prebuf, tbufs, cloud, sr_);

    // Tasteful defaults (already set by Init, shown here for reference)
    plate.SetWet(0.75f);
    plate.SetDecay(0.85f);
    plate.SetBrightness(0.70f);   // exponential; also sets tilt amount
    plate.SetBrightSwitch(false); // tilt hard-bypass default
    plate.SetRoomSize(0.90f);
    plate.SetERWidth(0.70f);
    plate.SetDiffusion(0.8f);

    // Pre-delay + modulation (dynamic)
    plate.SetPreDelay(30.0f);          // base ms
    plate.SetPreDelayMod(0.10f, 2.0f); // rate Hz, depth ms

    // Cloud blend
    plate.SetCloudSend(0.30f);
    plate.SetCloudOutMix(0.30f);

    // Freeze: off by default; 50 ms ramp already configured
    plate.SetFreeze(false);

    plate.SetOutputGain(1.0f);
}

inline void Plate_Process(float in, float &outL, float &outR)
{
    plate.Process(in, outL, outR);
}

bool engaged = true;   // toggle with button or jumper
float knob_wet = 0.7f; // optional parameter control
void AudioCallback(AudioHandle::InputBuffer in,
                   AudioHandle::OutputBuffer out,
                   size_t size)
{
    cpuLoadMeter.OnBlockStart();

    hw.ProcessAllControls();
    if (hw.button1.RisingEdge())
        engaged = !engaged;
    hw.led1.Set(engaged, engaged, engaged);
    hw.UpdateLeds();

    for (size_t i = 0; i < size; ++i)
    {
        if (engaged)
        {
            float dry = 0.5f * (in[0][i] + in[1][i]);
            float L, R;
            Plate_Process(dry, L, R);
            out[0][i] = L * knob_wet + dry * (1.f - knob_wet);
            out[1][i] = R * knob_wet + dry * (1.f - knob_wet);
        }
        else
        {
            for (size_t i = 0; i < size; ++i)
            {
                out[0][i] = in[0][i]; // left in -> left out
                out[1][i] = in[1][i]; // right in -> right out
            }
        }
    }

    cpuLoadMeter.OnBlockEnd();
}

int main(void)
{
    // initialize seed hardware and daisysp modules
    const bool boost = true;
    hw.Init(boost);

    hw.SetAudioBlockSize(48);

    const float sample_rate = hw.AudioSampleRate();
    const size_t block_size = hw.AudioBlockSize();

    Plate_Init(sample_rate);

    /** Initialize USB serial logging for debug output. */
    hw.seed.StartLog(true);

    /** Initialize the CPU load meter for performance monitoring. */
    cpuLoadMeter.Init(sample_rate, block_size);

    // start callback
    hw.StartAudio(AudioCallback);

    /** Start the ADC peripheral */
    hw.StartAdc();

    hw.seed.PrintLine("Daisy Seed running @ 48kHz");

    auto checkpoint = daisy::System::GetNow();
    /** Main loop (non-realtime). Handles periodic logging and housekeeping. */
    while (1)
    {
        auto now = daisy::System::GetNow();
        if (now - checkpoint >= 1000)
        {
            checkpoint = now;
            auto min = cpuLoadMeter.GetMinCpuLoad();
            auto max = cpuLoadMeter.GetMaxCpuLoad();
            auto avg = cpuLoadMeter.GetAvgCpuLoad();
            hw.seed.PrintLine(
                "CPU Min:" FLT_FMT(3) "   Max:" FLT_FMT(3) "   Avg:" FLT_FMT(3),
                FLT_VAR(3, min * 100),
                FLT_VAR(3, max * 100),
                FLT_VAR(3, avg * 100));

            cpuLoadMeter.Reset();
        }
    }
}

// Example audio callback pattern:
//
// void AudioCallback(float** in, float** out, size_t size)
// {
//     for(size_t i = 0; i < size; i++)
//     {
//         float L, R;
//         LushPlateV3_Process(in[0][i], L, R);
//         out[0][i] = L;
//         out[1][i] = R;
//     }
// }
//
// Example freeze toggle responding to a button:
// void OnButton(bool pressed) { plate.SetFreeze(pressed); }
//
// Example live control mapping:
// plate.SetPreDelay(knob_predelay_ms);
// plate.SetPreDelayModRate(knob_predelay_rate_hz);
// plate.SetPreDelayModDepth(knob_predelay_depth_ms);
// plate.SetDiffusion(knob_diffusion_0_1);
// plate.SetWet(knob_wet_0_1);
// plate.SetDecay(knob_decay_0_1);
// plate.SetBrightness(knob_brightness_0_1);
// plate.SetRoomSize(knob_roomsize_0_1);
// plate.SetERWidth(knob_width_0_1);
// plate.SetCloudSend(knob_cloud_send_0_1);
// plate.SetCloudOutMix(knob_cloud_outmix_0_1);
// plate.SetBrightSwitch(knob_bright_on);
// plate.SetOutputGain(knob_out_gain_0_to_2);
