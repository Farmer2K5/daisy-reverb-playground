/**
 * @file example_full_reverb.cpp
 * @brief Daisy Seed-style example: Early Reflections -> Diffusion -> Damped FDN Tank.
 */

#include "daisy_pod.h"
#include "daisysp.h"
#include "early_reflections.h"
#include "allpass_diffusion_network.h"
#include "fdn_line_damped_tank.h"

using namespace daisy;
using namespace daisysp;
using namespace daisyfarm;

static DaisyPod hw;
static CpuLoadMeter cpuLoadMeter;

// Early reflections: one shared buffer, 8 taps max
constexpr size_t ER_MAX = 8192;
DSY_SDRAM_BSS float er_buf[ER_MAX];
EarlyReflections<float, ER_MAX, 8> ER;

// Diffusion: 4 allpass stages
constexpr size_t DIFF_MAX = 16384;
DSY_SDRAM_BSS float diff_buf0[DIFF_MAX];
DSY_SDRAM_BSS float diff_buf1[DIFF_MAX];
DSY_SDRAM_BSS float diff_buf2[DIFF_MAX];
DSY_SDRAM_BSS float diff_buf3[DIFF_MAX];
AllpassDiffusionNetwork<float, DIFF_MAX, 4> DIFF;

// Tank: 8 lines, long buffers
constexpr size_t N = 8;
constexpr size_t TANK_MAX = 96000; // ~2s at 48k
DSY_SDRAM_BSS float tank_buf[N][TANK_MAX];
FDNLineDampedTank<float, TANK_MAX, N> TANK;

void ReverbInit(float SR)
{
    // ER init
    ER.Init(er_buf, SR);
    ER.SetTap(0, 8.0f, 0.7f, -0.6f);
    ER.SetTap(1, 12.5f, 0.6f, 0.2f);
    ER.SetTap(2, 17.0f, 0.55f, -0.1f);
    ER.SetTap(3, 23.0f, 0.5f, 0.6f);
    ER.SetTap(4, 30.0f, 0.45f, -0.3f);
    ER.SetTap(5, 37.0f, 0.4f, 0.1f);

    // Diffusion init
    float *dbufs[4] = {diff_buf0, diff_buf1, diff_buf2, diff_buf3};
    DIFF.Init(dbufs);
    DIFF.SetStage(0, 1200.0f, 0.75f); // ~25 ms
    DIFF.SetStage(1, 1900.0f, 0.70f);
    DIFF.SetStage(2, 2600.0f, 0.65f);
    DIFF.SetStage(3, 3200.0f, 0.60f);

    // Tank init
    float *tbufs[N];
    for (size_t i = 0; i < N; i++)
        tbufs[i] = tank_buf[i];
    TANK.Init(tbufs, SR);
    TANK.SetFeedback(0.78f);
    TANK.SetModulation(0.25f, 2.0f);
    TANK.SetDampingFreq(6000.0f); // gentle HF rolloff
}

void ReverbProcess(float in, float &outL, float &outR)
{
    // Early reflections
    float erL, erR;
    ER.Process(in, erL, erR);
    float er_sum = 0.5f * (erL + erR);

    // Diffusion
    float diff = DIFF.Process(er_sum);

    // Tank
    TANK.Process(diff, outL, outR);
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
            ReverbProcess(dry, L, R);
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

    const float sample_rate = hw.AudioSampleRate();
    const size_t block_size = hw.AudioBlockSize();

    ReverbInit(sample_rate);

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
