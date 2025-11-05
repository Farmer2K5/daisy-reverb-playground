/**
 * @file example_firefly_plate.cpp
 * @brief Example: Huge Ambient 'Firefly Plate' Hybrid Reverb.
 *
 * Chain: Early Reflections → Diffusion → Firefly Cloud → Damped FDN Tank
 */

#include "daisy_pod.h"
#include "daisysp.h"
#include "firefly_plate_reverb.h"

using namespace daisy;
using namespace daisysp;
using namespace daisyfarm;

static DaisyPod hw;
static CpuLoadMeter cpuLoadMeter;

// Buffer sizes
constexpr size_t ER_MAX = 16384;    // ~341 ms
constexpr size_t DIFF_MAX = 24576;  // per diffuser
constexpr size_t TANK_MAX = 131072; // per tank line (~2.73s)
constexpr size_t NF = 64;           // number of fireflies
constexpr size_t NLINES = 8;        // FDN size

// Buffers
DSY_SDRAM_BSS float er_buf[ER_MAX];
DSY_SDRAM_BSS float diff0[DIFF_MAX];
DSY_SDRAM_BSS float diff1[DIFF_MAX];
DSY_SDRAM_BSS float diff2[DIFF_MAX];
DSY_SDRAM_BSS float diff3[DIFF_MAX];
DSY_SDRAM_BSS float cloud_buf[TANK_MAX];
DSY_SDRAM_BSS float tank_buf[NLINES][TANK_MAX];

// Effect instance
FireflyPlateReverb<float, ER_MAX, DIFF_MAX, TANK_MAX, NF, NLINES> plate;

void FireflyPlateInit(float sr_)
{
    float *tbufs[NLINES];
    for (size_t i = 0; i < NLINES; i++)
        tbufs[i] = tank_buf[i];

    plate.Init(er_buf, diff0, diff1, diff2, diff3, cloud_buf, tbufs, sr_);

    // High-level tuning for huge ambient space
    plate.SetWet(0.85f);
    plate.SetDecay(0.9f);          // long decay
    plate.SetBrightness(0.7f);     // fairly bright
    plate.SetRoomSize(0.85f);      // large
    plate.SetFireflyDensity(0.6f); // more blinking fireflies
    plate.SetTankSend(0.9f);
    plate.SetCloudSend(0.95f);
}

inline void FireflyPlateProcess(float in, float &outL, float &outR)
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
            FireflyPlateProcess(dry, L, R);
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

    FireflyPlateInit(sample_rate);

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

// Example audio callback form:
// void AudioCallback(float** in, float** out, size_t size)
// {
//     for(size_t i = 0; i < size; i++)
//     {
//         float L, R;
//         FireflyPlateProcess(in[0][i], L, R);
//         out[0][i] = L;
//         out[1][i] = R;
//     }
// }
