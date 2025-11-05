#include "daisy_pod.h"
#include "daisysp.h"
#include "nebula_hall_tank.h"

using namespace daisy;
using namespace daisysp;
using namespace daisyfarm;

static DaisyPod hw;
static CpuLoadMeter cpuLoadMeter;

// ==========================
// CONFIG
// ==========================
constexpr size_t kLines = 8;
constexpr size_t kMaxDelaySamples = 20000; // ~400ms per line @ 48k
static float DSY_SDRAM_BSS delay_buf[kLines][kMaxDelaySamples];
static float DSY_SDRAM_BSS shimmer_buf[kMaxDelaySamples];

NebulaHallTank<float, kMaxDelaySamples, kLines> verb;

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
            verb.Process(dry, L, R);
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

    hw.SetAudioBlockSize(32);

    const float sample_rate = hw.AudioSampleRate();
    const size_t block_size = hw.AudioBlockSize();

    // Pointer array for delay lines
    float *line_ptrs[kLines];
    for (size_t i = 0; i < kLines; i++)
        line_ptrs[i] = delay_buf[i];

    // Init reverb
    verb.Init(line_ptrs, sample_rate, shimmer_buf, kMaxDelaySamples);

    verb.SetMixHouseholder();

    const float delays[8] = {1153.f, 1627.f, 2137.f, 2791.f, 3413.f, 4001.f, 4799.f, 5527.f};
    for (size_t i = 0; i < 8; i++)
        verb.SetDelay(i, delays[i]);

    verb.SetDamping(1.03f, 0.92f, 0.78f);
    verb.SetDampingCrossover(280.f, 5100.f);

    verb.SetFeedback(0.87f);
    verb.SetBloom(0.30f);

    verb.SetShimmer(true, 0.16f);
    verb.SetShimmerParams(+12.f, 0.03f, 18.f, 14.f);
    verb.SetShimmerChorus(0.00f, 0.0000f);

    // NEW – tone tilt (no private access)
    verb.SetToneWarmth(1.06f);
    verb.SetToneCutoff(2500.f);

    verb.SetModulation(0.0f, 0.0f);
    verb.SetDune(false);
    verb.SetWander(false);
    verb.SetHoloDream(0, 0, 0);

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
