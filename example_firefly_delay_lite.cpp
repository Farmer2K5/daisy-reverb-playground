
/**
 * @file example_firefly_lite.cpp
 * @brief Example integration for FireflyDelayLite on Daisy-style platform.
 */
#include "daisy_pod.h"
#include "daisysp.h"
#include "firefly_delay_lite.h"

using namespace daisy;
using namespace daisysp;
using namespace daisyfarm;

static DaisyPod hw;
static CpuLoadMeter cpuLoadMeter;

constexpr size_t MAX = 131072;    // ~2.73s at 48k
constexpr size_t NFIREFLIES = 20; // 20 was good balance for CPU

DSY_SDRAM_BSS float firefly_buf[MAX];
FireflyDelayLite<float, MAX, NFIREFLIES> fireflies;

void FireflyLiteInit(float SR)
{
    fireflies.Init(firefly_buf, SR);
    fireflies.SetWet(0.65f);
    fireflies.SetFeedback(0.38f);
    fireflies.SetDelayRangeMs(40.0f, 1200.0f);
    fireflies.SetDensity(0.45f);
    fireflies.SetBlinkBaseRate(0.22f);
    fireflies.SetControlDiv(8); // control at 48kHz / 8 = 6 kHz
    fireflies.SetDetuneCents(3.0f);
    fireflies.EnableDetune(true); // set false for even less CPU
}

inline void FireflyLiteProcess(float in, float &outL, float &outR)
{
    fireflies.Process(in, outL, outR);
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
            FireflyLiteProcess(dry, L, R);
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

    FireflyLiteInit(sample_rate);

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
