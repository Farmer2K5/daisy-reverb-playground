
/**
 * @file firefly_delay_perf_v3.h
 * @brief "Firefly" delay cloud — perf-core with lush, calm, wide-motion tweaks for 32 flies.
 *
 *  - Foundation: perf architecture (sub-rate control, flat arrays, fractional delay cloud).
 *  - Target vibe: Style A (calm, soft, drifting pads) with WIDE-MOTION stereo.
 *  - Safe for N=32–64 with modest CPU even on Daisy Seed (48 kHz).
 *
 * Additions vs perf:
 *  - Micro drift on blink phases (breaks coherence for large N without CPU spikes).
 *  - Second, ultra-slow detune LFO (breathing tail motion) — polynomial sine.
 *  - Per-firefly envelope rate randomization (less "pumping", smoother onset/decay).
 *  - Slow stereo wander (very small pan motion at control rate) for width without seasickness.
 *  - Post-mix wet gain (as in perf) and FTZ/DAZ & denormal guards.
 *
 * Default voicing is tuned for: N=32, calm & wide. For 64 flies, consider increasing control_div to 12.
 *
 * Usage (example):
 *   DSY_SDRAM_BSS float buf[MAX_SAMPS];
 *   daisyfarm::FireflyDelayPerfV3<float, MAX_SAMPS, 32> fx;
 *   fx.Init(buf, 48000.0f);
 *   fx.SetControlDiv(8);            // ~6 kHz control @ 48 kHz
 *   fx.SetOutputGain(1.0f);         // post-mix wet gain
 *   fx.EnableDetune(true);
 *   // optional sweetening:
 *   fx.SetDriftDepth(0.015f);
 *   fx.SetSlowDetuneCents(5.0f);
 *   fx.SetStereoWander(0.10f);
 */
#pragma once
#include "delay_buffer_ext.h"
#include <cmath>
#include <cstddef>
#include <cstdint>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace daisyfarm
{

    // Optional helper: call once after seed.Init()
    static inline void EnableFTZ_DAZ_V3()
    {
#if defined(__arm__) || defined(__thumb__)
        uint32_t fpscr;
        __asm volatile("VMRS %0, FPSCR" : "=r"(fpscr));
        fpscr |= (1u << 24) | (1u << 25); // FZ | DN
        __asm volatile("VMSR FPSCR, %0" : : "r"(fpscr));
#endif
    }

    template <typename T, size_t max_size, size_t N_FIREFLIES>
    class FireflyDelayPerfV3
    {
    public:
        FireflyDelayPerfV3()
            : sr_(48000.0f),
              fb_(0.30f),
              wet_(0.65f),
              output_gain_(1.0f),
              min_ms_(30.0f),
              max_ms_(1100.0f),
              detune_cents_fast_(3.0f),
              detune_cents_slow_(5.0f),
              density_(0.40f),      // calmer cloud
              base_rate_hz_(0.12f), // slower blink base for "calm"
              control_div_(8),
              counter_(0),
              enable_detune_(true),
              drift_depth_(0.015f),  // tiny phase drift (0..~0.05)
              stereo_wander_(0.10f), // radians of pan wander (very small)
              seed_(0xC0FFEE11u)
        {
            for (size_t i = 0; i < N_FIREFLIES; ++i)
            {
                base_delay_[i] = 0.25f * sr_;
                depth_fast_[i] = 0.3f;
                depth_slow_[i] = 0.2f;
                pan0_[i] = 2.0f * Rand01_() - 1.0f;
                env_[i] = 0.0f;
                gain_[i] = 0.45f + 0.55f * Rand01_();
                duty_[i] = 0.10f + 0.20f * Rand01_();
                phase_[i] = Rand01_();
                phase_inc_base_[i] = 0.0f;
                phase_slow_[i] = Rand01_();
                slow_inc_[i] = 0.0f;
                env_rate_[i] = 0.10f + 0.08f * Rand01_(); // per-firefly env speed (calm)
                pan_angle0_[i] = 0.0f;                    // filled on Init()
                panL_[i] = 0.5f;
                panR_[i] = 0.5f;
                ctrl_[i] = {base_delay_[i], 0.25f, 0.25f};
            }
        }

        void Init(T *external_buffer, float sample_rate)
        {
            d_.Init(external_buffer);
            sr_ = sample_rate;
            counter_ = 0;

            for (size_t i = 0; i < N_FIREFLIES; ++i)
            {
                const float u = Rand01_();
                const float ms = min_ms_ + u * (max_ms_ - min_ms_);
                base_delay_[i] = ms * 0.001f * sr_;

                // Equal-power base pan (pre-wander)
                pan_angle0_[i] = 0.25f * float(M_PI) * (pan0_[i] + 1.0f);
                panL_[i] = std::cos(pan_angle0_[i]);
                panR_[i] = std::sin(pan_angle0_[i]);
            }

            ConfigureDetune_();
            ConfigureRates_();
        }

        void Reset()
        {
            d_.Reset();
            counter_ = 0;
            for (size_t i = 0; i < N_FIREFLIES; ++i)
            {
                env_[i] = 0.0f;
                // keep phases for continuity
            }
        }

        // --- Parameters ---
        void SetFeedback(float fb) { fb_ = Clamp_(fb, 0.0f, 0.98f); }
        void SetWet(float w) { wet_ = Clamp_(w, 0.0f, 1.0f); }
        void SetOutputGain(float g) { output_gain_ = Clamp_(g, 0.0f, 4.0f); }
        void SetDensity(float d) { density_ = Clamp_(d, 0.0f, 1.0f); }
        void SetDelayRangeMs(float mi, float ma)
        {
            min_ms_ = (mi < 1.0f ? 1.0f : mi);
            max_ms_ = (ma < min_ms_ ? min_ms_ : ma);
        }
        void SetDetuneCents(float cents_fast)
        {
            detune_cents_fast_ = (cents_fast < 0.0f ? 0.0f : cents_fast);
            ConfigureDetune_();
        }
        void SetSlowDetuneCents(float cents_slow)
        {
            detune_cents_slow_ = (cents_slow < 0.0f ? 0.0f : cents_slow);
            ConfigureDetune_();
        }
        void EnableDetune(bool e)
        {
            enable_detune_ = e;
            ConfigureDetune_();
        }
        void SetBlinkBaseRate(float hz)
        {
            base_rate_hz_ = (hz < 0.01f ? 0.01f : hz);
            ConfigureRates_();
        }
        void SetControlDiv(uint32_t div)
        {
            control_div_ = (div == 0 ? 1u : div);
            ConfigureRates_();
        }
        void SetDriftDepth(float d) { drift_depth_ = Clamp_(d, 0.0f, 0.08f); }
        void SetStereoWander(float rad) { stereo_wander_ = Clamp_(rad, 0.0f, 0.60f); }

        /** Per-sample process: mono in → stereo out. */
        inline void Process(T in, T &outL, T &outR)
        {
            // optional tiny dither to avoid denormals in long tails
            in += static_cast<T>(1e-20f);

            if ((counter_++ % control_div_) == 0)
                UpdateControl_();

            float L = 0.0f, R = 0.0f;
            for (size_t i = 0; i < N_FIREFLIES; ++i)
            {
                const T y = d_.Read(ctrl_[i].delay);
                L += ctrl_[i].gainL * y;
                R += ctrl_[i].gainR * y;
            }

            const T wetmono = static_cast<T>(0.5f) * (L + R);
            T fbSample = in + static_cast<T>(fb_) * wetmono;

            // Denormal clamp
            if (std::fabs((float)fbSample) < 1e-30f)
                fbSample = static_cast<T>(0.0f);

            d_.Write(fbSample);
            d_.Advance();

            const T wetL = static_cast<T>(wet_) * static_cast<T>(L) * static_cast<T>(output_gain_);
            const T wetR = static_cast<T>(wet_) * static_cast<T>(R) * static_cast<T>(output_gain_);
            outL = wetL + static_cast<T>(1.0f - wet_) * in;
            outR = wetR + static_cast<T>(1.0f - wet_) * in;
        }

    private:
        struct CtrlState
        {
            float delay, gainL, gainR;
        };

        inline void UpdateControl_()
        {
            const float ctrl_sr = sr_ / float(control_div_);

            // tiny time-varying drift noise generator
            auto noiseUni = [this]()
            {
                seed_ = 1664525u * seed_ + 1013904223u;
                return float((seed_ >> 8) & 0x00FFFFFFu) * (1.0f / float(0x00FFFFFFu)) - 0.5f; // [-0.5..0.5]
            };

            for (size_t i = 0; i < N_FIREFLIES; ++i)
            {
                // Micro drift on phase increment
                const float drift = 1.0f + drift_depth_ * noiseUni();

                float ph = phase_[i] + (phase_inc_base_[i] * drift);
                if (ph >= 1.0f)
                    ph -= 1.0f;
                if (ph < 0.0f)
                    ph += 1.0f;
                phase_[i] = ph;

                // Gate & envelope (per-firefly rate for smooth calm pads)
                const float thr = duty_[i] * density_;
                const float gate = (ph < thr) ? 1.0f : 0.0f;
                env_[i] += env_rate_[i] * (gate - env_[i]);

                // Delay modulation: fast detune + ultra-slow detune ("breathing")
                float delay_samps = base_delay_[i];
                if (enable_detune_)
                {
                    // fast (blink-synchronous-ish) — use poly sine
                    const float s_fast = fastSin_(ph * float(M_PI)); // [approx sin(π*ph)]
                    delay_samps += depth_fast_[i] * s_fast;

                    // slow independent LFO
                    float ps = phase_slow_[i] + slow_inc_[i] / ctrl_sr;
                    if (ps >= 1.0f)
                        ps -= 1.0f;
                    phase_slow_[i] = ps;
                    const float s_slow = fastSin_(ps * 2.0f * float(M_PI));
                    delay_samps += depth_slow_[i] * s_slow;
                }

                // Wide-motion stereo: very small pan wander on the slow LFO
                float pan_angle = pan_angle0_[i] + stereo_wander_ * fastSin_(phase_slow_[i] * 2.0f * float(M_PI) + 0.73f * i);
                const float panL = std::cos(pan_angle);
                const float panR = std::sin(pan_angle);

                const float g = env_[i] * gain_[i];
                ctrl_[i].delay = delay_samps;
                ctrl_[i].gainL = g * panL;
                ctrl_[i].gainR = g * panR;
            }
        }

        inline float fastSin_(float x) const
        {
            // Fast polynomial sine (adequate for LFOs)
            const float B = 4.0f / M_PI;
            const float C = -4.0f / (M_PI * M_PI);
            float y = B * x + C * x * std::fabs(x);
            const float P = 0.225f;
            y = P * (y * std::fabs(y) - y) + y;
            return y;
        }

        void ConfigureDetune_()
        {
            const float scale_fast = detune_cents_fast_ / 1200.0f;
            const float scale_slow = detune_cents_slow_ / 1200.0f;
            for (size_t i = 0; i < N_FIREFLIES; ++i)
            {
                float dfast = enable_detune_ ? (0.012f * scale_fast * base_delay_[i]) : 0.0f;
                float dslow = enable_detune_ ? (0.020f * scale_slow * base_delay_[i]) : 0.0f;
                depth_fast_[i] = Clamp_(dfast, 0.0f, 6.0f);
                depth_slow_[i] = Clamp_(dslow, 0.0f, 10.0f);
            }
        }

        void ConfigureRates_()
        {
            const float ctrl_sr = sr_ / float(control_div_);
            for (size_t i = 0; i < N_FIREFLIES; ++i)
            {
                // Blink base rate with per-voice spread
                float hz = base_rate_hz_ * (0.6f + 0.8f * (0.25f + 0.75f * Rand01_()));
                phase_inc_base_[i] = hz / ctrl_sr;

                // Ultra-slow LFO target around 0.02–0.06 Hz (very slow breathing)
                float slow_hz = 0.02f + 0.04f * Rand01_();
                slow_inc_[i] = slow_hz; // increment measured in cycles/second (applied /ctrl_sr in Update)
            }
        }

        template <typename X>
        static inline X Clamp_(X x, X a, X b) { return (x < a ? a : (x > b ? b : x)); }

        inline float Rand01_()
        {
            seed_ = 1664525u * seed_ + 1013904223u;
            return float((seed_ >> 8) & 0x00FFFFFFu) / float(0x00FFFFFFu);
        }

        // State / data
        DelayBufferExt<T, max_size> d_;
        // struct CtrlState
        // {
        //     float delay, gainL, gainR;
        // };
        CtrlState ctrl_[N_FIREFLIES];

        // Per-firefly params/state
        float base_delay_[N_FIREFLIES];
        float depth_fast_[N_FIREFLIES]; // samples
        float depth_slow_[N_FIREFLIES]; // samples
        float pan0_[N_FIREFLIES];       // -1..+1 base
        float pan_angle0_[N_FIREFLIES]; // equal-power base angle
        float panL_[N_FIREFLIES];
        float panR_[N_FIREFLIES];
        float gain_[N_FIREFLIES];
        float duty_[N_FIREFLIES];
        float env_[N_FIREFLIES];
        float env_rate_[N_FIREFLIES];
        float phase_[N_FIREFLIES];
        float phase_inc_base_[N_FIREFLIES];
        float phase_slow_[N_FIREFLIES];
        float slow_inc_[N_FIREFLIES]; // Hz, applied at control rate

        // Globals
        float sr_, fb_, wet_, output_gain_;
        float min_ms_, max_ms_;
        float detune_cents_fast_, detune_cents_slow_;
        float density_, base_rate_hz_;
        uint32_t control_div_, counter_;
        bool enable_detune_;
        float drift_depth_;
        float stereo_wander_;
        uint32_t seed_;
    };

} // namespace daisyfarm
