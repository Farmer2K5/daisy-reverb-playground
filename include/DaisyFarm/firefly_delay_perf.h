
/**
 * @file firefly_delay_perf.h
 * @brief Lean, performance-optimized "Firefly" delay cloud for Daisy Seed.
 *
 * Features
 *  - Sub-rate control updates via SetControlDiv() (default 8 → 6 kHz @ 48 kHz).
 *  - Optional detune wobble; fast polynomial.
 *  - Precomputed equal-power panning (no sin/cos in hot loop).
 *  - NEW: SetOutputGain(float) scales the *wet* signal post-mix (after wet% is applied).
 *  - Corrected Clamp() logic; FTZ/DAZ helper to avoid denorm stalls.
 *
 * Usage
 *    DSY_SDRAM_BSS float buf[MAX];
 *    daisyfarm::FireflyDelay<float, MAX, 16> fx;
 *    fx.Init(buf, 48000.0f);
 *    fx.SetControlDiv(8);  // 6 kHz control (default)
 *    fx.SetOutputGain(1.0f);
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
    static inline void EnableFTZ_DAZ()
    {
#if defined(__arm__) || defined(__thumb__)
        uint32_t fpscr;
        __asm volatile("VMRS %0, FPSCR" : "=r"(fpscr));
        fpscr |= (1u << 24) | (1u << 25); // FZ | DN
        __asm volatile("VMSR FPSCR, %0" : : "r"(fpscr));
#endif
    }

    template <typename T, size_t max_size, size_t N_FIREFLIES>
    class FireflyDelayPerf
    {
    public:
        FireflyDelayPerf()
            : sr_(48000.0f),
              fb_(0.35f),
              wet_(0.6f),
              output_gain_(1.0f),
              min_ms_(25.0f),
              max_ms_(950.0f),
              detune_cents_(3.0f),
              density_(0.45f),
              base_rate_hz_(0.20f),
              control_div_(8),
              counter_(0),
              enable_detune_(true),
              seed_(0xA5A5A5A5u)
        {
            for (size_t i = 0; i < N_FIREFLIES; ++i)
            {
                base_delay_[i] = 0.25f * sr_;
                depth_[i] = 0.5f; // in samples
                pan_[i] = 2.0f * Rand01() - 1.0f;
                env_[i] = 0.0f;
                gain_[i] = 0.5f + 0.5f * Rand01();
                duty_[i] = 0.12f + 0.22f * Rand01();
                phase_[i] = Rand01();
                phase_inc_[i] = 0.0f;
                panL_[i] = 0.5f; // filled on Init()
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
                const float u = Rand01();
                const float ms = min_ms_ + u * (max_ms_ - min_ms_);
                base_delay_[i] = ms * 0.001f * sr_;
                const float angle = 0.25f * float(M_PI) * (pan_[i] + 1.0f);
                panL_[i] = std::cos(angle);
                panR_[i] = std::sin(angle);
            }

            ConfigureDetune_();
            ConfigureRates_();
        }

        void Reset()
        {
            d_.Reset();
            counter_ = 0;
            for (size_t i = 0; i < N_FIREFLIES; ++i)
                env_[i] = 0.0f;
        }

        // --- Parameters ---
        void SetFeedback(float fb) { fb_ = Clamp_(fb, 0.0f, 0.98f); }
        void SetWet(float w) { wet_ = Clamp_(w, 0.0f, 1.0f); }
        void SetOutputGain(float g) { output_gain_ = Clamp_(g, 0.0f, 4.0f); }
        void SetDensity(float d) { density_ = Clamp_(d, 0.0f, 1.0f); }
        void SetDelayRangeMs(float min_ms, float max_ms)
        {
            min_ms_ = (min_ms < 1.0f ? 1.0f : min_ms);
            max_ms_ = (max_ms < min_ms_ ? min_ms_ : max_ms);
        }
        void SetDetuneCents(float c)
        {
            detune_cents_ = (c < 0.0f ? 0.0f : c);
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

        /** Per-sample process: mono in → stereo out. */
        inline void Process(T in, T &outL, T &outR)
        {
            // Optional tiny dither to avoid denormals in long tails
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

            // Wet/dry with post-mix wet gain
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
            for (size_t i = 0; i < N_FIREFLIES; ++i)
            {
                float ph = phase_[i] + phase_inc_[i];
                if (ph >= 1.0f)
                    ph -= 1.0f;
                phase_[i] = ph;

                const float thr = duty_[i] * density_;
                const float gate = (ph < thr) ? 1.0f : 0.0f;
                env_[i] += 0.2f * (gate - env_[i]);

                float delay_samps = base_delay_[i];
                if (enable_detune_)
                {
                    const float x = ph * float(M_PI);
                    const float s = fastSin_(x);
                    delay_samps += depth_[i] * s;
                }

                const float g = env_[i] * gain_[i];
                ctrl_[i].delay = delay_samps;
                ctrl_[i].gainL = g * panL_[i];
                ctrl_[i].gainR = g * panR_[i];
            }
        }

        inline float fastSin_(float x) const
        {
            // Fast polynomial sine (good for slow LFO ranges)
            const float B = 4.0f / M_PI;
            const float C = -4.0f / (M_PI * M_PI);
            float y = B * x + C * x * std::fabs(x);
            const float P = 0.225f;
            y = P * (y * std::fabs(y) - y) + y;
            return y;
        }

        void ConfigureDetune_()
        {
            const float cents_scale = detune_cents_ / 1200.0f;
            for (size_t i = 0; i < N_FIREFLIES; ++i)
            {
                float depth = enable_detune_ ? (0.015f * cents_scale * base_delay_[i]) : 0.0f;
                depth_[i] = Clamp_(depth, 0.0f, 6.0f); // cap in samples
            }
        }

        void ConfigureRates_()
        {
            const float ctrl_sr = sr_ / float(control_div_);
            for (size_t i = 0; i < N_FIREFLIES; ++i)
            {
                float hz = base_rate_hz_ * (0.6f + 0.8f * (0.25f + 0.75f * Rand01()));
                phase_inc_[i] = hz / ctrl_sr;
            }
        }

        template <typename X>
        static inline X Clamp_(X x, X a, X b) { return (x < a ? a : (x > b ? b : x)); }

        inline float Rand01()
        {
            seed_ = 1664525u * seed_ + 1013904223u;
            return float((seed_ >> 8) & 0x00FFFFFFu) / float(0x00FFFFFFu);
        }

        // State / data
        DelayBufferExt<T, max_size> d_;
        CtrlState ctrl_[N_FIREFLIES];

        // Per-firefly params/state
        float base_delay_[N_FIREFLIES];
        float depth_[N_FIREFLIES];
        float pan_[N_FIREFLIES];
        float panL_[N_FIREFLIES];
        float panR_[N_FIREFLIES];
        float gain_[N_FIREFLIES];
        float duty_[N_FIREFLIES];
        float env_[N_FIREFLIES];
        float phase_[N_FIREFLIES];
        float phase_inc_[N_FIREFLIES];

        // Globals
        float sr_, fb_, wet_, output_gain_;
        float min_ms_, max_ms_, detune_cents_, density_, base_rate_hz_;
        uint32_t control_div_, counter_;
        bool enable_detune_;
        uint32_t seed_;
    };

} // namespace daisyfarm
