/**
 * @file firefly_plate.h
 * @brief Firefly Plate Hybrid Reverb: ER → Diffusion → Firefly Cloud → Damped FDN Tank.
 *
 * Designed for huge ambient spaces with long, lush tails.
 * Dependencies:
 *   - delaybuffer_ext.h
 *   - early_reflections.h
 *   - diffusion_stage.h
 *   - reverb_tank_damped.h
 *   - firefly_delay.h
 */
#pragma once
#include "allpass_diffusion_network.h"
#include "early_reflections.h"
#include "firefly_delay_perf_v3.h"
#include "fdn_line_damped_tank.h"
#include <cmath>
#include <cstddef>

namespace daisyfarm
{

    /**
     * @tparam T         Sample type (float)
     * @tparam ER_MAX    Early reflections buffer size (samples)
     * @tparam DIFF_MAX  Per-diffuser buffer size (samples)
     * @tparam TANK_MAX  Per-tank-line buffer size (samples)
     * @tparam NF        Number of fireflies in the cloud
     * @tparam NLINES    Number of FDN lines
     */
    template <typename T,
              size_t ER_MAX,
              size_t DIFF_MAX,
              size_t TANK_MAX,
              size_t NF = 64,
              size_t NLINES = 8>
    class FireflyPlateReverb
    {
    public:
        FireflyPlateReverb() : sr_(48000.0f), wet_(0.8f), tank_send_(0.85f), cloud_send_(0.9f) {}

        /** Initialize with external SDRAM buffers. */
        void Init(
            // ER
            T *er_buffer,
            // Diffusers (4)
            T *diff_buf0, T *diff_buf1, T *diff_buf2, T *diff_buf3,
            // Firefly cloud
            T *cloud_buffer,
            // Tank (NLINES)
            T *tank_buffers[NLINES],
            float sample_rate)
        {
            sr_ = sample_rate;

            // Early reflections (6 taps default)
            ER_.Init(er_buffer, sr_);
            ER_.SetTap(0, 9.0f, 0.70f, -0.7f);
            ER_.SetTap(1, 14.0f, 0.62f, 0.1f);
            ER_.SetTap(2, 19.0f, 0.55f, -0.2f);
            ER_.SetTap(3, 26.0f, 0.50f, 0.6f);
            ER_.SetTap(4, 34.0f, 0.45f, -0.4f);
            ER_.SetTap(5, 42.0f, 0.40f, 0.2f);

            // Diffusion network
            T *dbufs[4] = {diff_buf0, diff_buf1, diff_buf2, diff_buf3};
            DIFF_.Init(dbufs);
            // Long-ish diffusers for big space (g near 0.7–0.78)
            DIFF_.SetStage(0, 1200.0f, 0.76f); // ~25 ms
            DIFF_.SetStage(1, 1800.0f, 0.74f);
            DIFF_.SetStage(2, 2500.0f, 0.72f);
            DIFF_.SetStage(3, 3200.0f, 0.70f);

            // Firefly cloud
            CLOUD_.Init(cloud_buffer, sr_);
            CLOUD_.SetWet(1.0f);
            CLOUD_.SetFeedback(0.50f); // short feedback in cloud stage
            CLOUD_.SetDelayRangeMs(80.0f, 1500.0f);
            CLOUD_.SetDetuneCents(7.0f);
            CLOUD_.SetBlinkBaseRate(0.16f);
            // CLOUD_.SetEnvTimes(0.010f, 0.300f);

            // Tank
            TANK_.Init(tank_buffers, sr_);
            TANK_.SetFeedback(0.82f); // long decay
            TANK_.SetModulation(0.22f, 2.5f);
            TANK_.SetDampingFreq(5200.0f); // airy but not shrill
        }

        /** Global wet mix (0..1). */
        void SetWet(float w) { wet_ = Clamp(w, 0.0f, 1.0f); }

        /** Balance into the FDN tank. */
        void SetTankSend(float s) { tank_send_ = Clamp(s, 0.0f, 1.0f); }

        /** Balance into Firefly cloud. */
        void SetCloudSend(float s) { cloud_send_ = Clamp(s, 0.0f, 1.0f); }

        /** High-level controls */
        void SetDecay(float amt) // 0..1
        {
            // Map to tank feedback (0.72..0.9)
            float fb = 0.72f + 0.18f * Clamp(amt, 0.0f, 1.0f);
            TANK_.SetFeedback(fb);
        }

        void SetBrightness(float amt) // 0..1
        {
            // Map to damping freq (2k..12k)
            float fc = 2000.0f + 10000.0f * Clamp(amt, 0.0f, 1.0f);
            TANK_.SetDampingFreq(fc);
        }

        void SetRoomSize(float amt) // 0..1
        {
            // Scale diffuser delays
            const float scale = 0.8f + 0.8f * Clamp(amt, 0.0f, 1.0f);
            DIFF_.SetStage(0, 1200.0f * scale, 0.76f);
            DIFF_.SetStage(1, 1800.0f * scale, 0.74f);
            DIFF_.SetStage(2, 2500.0f * scale, 0.72f);
            DIFF_.SetStage(3, 3200.0f * scale, 0.70f);
        }

        void SetFireflyDensity(float amt) // 0..1
        {
            CLOUD_.SetDensity(Clamp(amt, 0.0f, 1.0f));
        }

        /** One-sample process: mono in → stereo out. */
        inline void Process(T in, T &outL, T &outR)
        {
            // 1) Early reflections (stereo)
            T erL, erR;
            ER_.Process(in, erL, erR);
            const T er_sum = static_cast<T>(0.5f) * (erL + erR);

            // 2) Diffusion (mono)
            const T diff = DIFF_.Process(er_sum);

            // 3) Firefly cloud (stereo)
            T cloudL, cloudR;
            CLOUD_.Process(diff, cloudL, cloudR);

            // 4) Send both diff and cloud into the tank
            const T tank_in = static_cast<T>(tank_send_) * diff +
                              static_cast<T>(cloud_send_) * static_cast<T>(0.5f) * (cloudL + cloudR);

            T tankL, tankR;
            TANK_.Process(tank_in, tankL, tankR);

            // 5) Final mix: tank + cloud (wet) against input (dry inside each stage already handled)
            const T wetL = static_cast<T>(0.7f) * tankL + static_cast<T>(0.3f) * cloudL;
            const T wetR = static_cast<T>(0.7f) * tankR + static_cast<T>(0.3f) * cloudR;

            outL = static_cast<T>(wet_) * wetL + static_cast<T>(1.0f - wet_) * in;
            outR = static_cast<T>(wet_) * wetR + static_cast<T>(1.0f - wet_) * in;
        }

    private:
        template <typename X>
        X Clamp(X x, X a, X b) const { return (x < a ? a : (x > b ? b : x)); }

        float sr_, wet_, tank_send_, cloud_send_;

        // Blocks
        EarlyReflections<T, ER_MAX, 8> ER_;
        AllpassDiffusionNetwork<T, DIFF_MAX, 4> DIFF_;
        FireflyDelayPerfV3<T, TANK_MAX, NF> CLOUD_; // reuse large buffer size macro for convenience
        FDNLineDampedTank<T, TANK_MAX, NLINES> TANK_;
    };

} // namespace daisyfarm
