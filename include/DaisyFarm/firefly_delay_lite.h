
/**
 * @file firefly_delay_lite.h
 * @brief CPU-friendly "Firefly" delay cloud with sub-rate control for Daisy Seed.
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

    template <typename T, size_t max_size, size_t N_FIREFLIES>
    class FireflyDelayLite
    {
    public:
        FireflyDelayLite()
            : sr_(48000.0f),
              fb_(0.4f),
              mix_(0.6f),
              min_ms_(25.0f),
              max_ms_(900.0f),
              detune_cents_(3.0f),
              density_(0.45f),
              base_rate_hz_(0.2f),
              control_div_(8),
              counter_(0),
              enable_detune_(true),
              seed_(0x13579BDFu)
        {
            for (size_t i = 0; i < N_FIREFLIES; ++i)
            {
                base_delay_[i] = 0.25f * sr_;
                depth_[i] = 0.5f;
                pan_[i] = 2.0f * Rand01() - 1.0f;
                gain_[i] = 0.5f + 0.5f * Rand01();
                duty_[i] = 0.12f + 0.22f * Rand01();
                phase_[i] = Rand01();
                phase_inc_[i] = base_rate_hz_ / sr_ * float(control_div_);
                env_[i] = 0.0f;
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
            }
            ConfigureDetune();
            ConfigureRates();
        }

        void Reset()
        {
            d_.Reset();
            counter_ = 0;
            for (size_t i = 0; i < N_FIREFLIES; ++i)
                env_[i] = 0.0f;
        }

        void SetFeedback(float fb) { fb_ = Clamp(fb, 0.0f, 0.98f); }
        void SetWet(float w) { mix_ = Clamp(w, 0.0f, 1.0f); }
        void SetDensity(float d) { density_ = Clamp(d, 0.0f, 1.0f); }
        void SetDelayRangeMs(float min_ms, float max_ms)
        {
            min_ms_ = (min_ms < 1.0f ? 1.0f : min_ms);
            max_ms_ = (max_ms < min_ms_ ? min_ms_ : max_ms);
        }
        void SetDetuneCents(float c)
        {
            detune_cents_ = (c < 0.0f ? 0.0f : c);
            ConfigureDetune();
        }
        void EnableDetune(bool e) { enable_detune_ = e; }
        void SetBlinkBaseRate(float hz)
        {
            base_rate_hz_ = (hz < 0.01f ? 0.01f : hz);
            ConfigureRates();
        }
        void SetControlDiv(uint32_t div)
        {
            control_div_ = (div == 0 ? 1u : div);
            ConfigureRates();
        }

        inline void Process(T in, T &outL, T &outR)
        {
            if ((counter_++ % control_div_) == 0)
                UpdateControl();

            float L = 0.0f, R = 0.0f;
            for (size_t i = 0; i < N_FIREFLIES; ++i)
            {
                const T y = d_.Read(ctrl_[i].delay);
                L += ctrl_[i].gainL * y;
                R += ctrl_[i].gainR * y;
            }

            const T wet_mono = static_cast<T>(0.5f) * (L + R);
            d_.Write(in + static_cast<T>(fb_) * wet_mono);
            d_.Advance();

            outL = static_cast<T>(mix_) * static_cast<T>(L) + static_cast<T>(1.0f - mix_) * in;
            outR = static_cast<T>(mix_) * static_cast<T>(R) + static_cast<T>(1.0f - mix_) * in;
        }

    private:
        struct CtrlState
        {
            float delay, gainL, gainR;
        };

        void UpdateControl()
        {
            for (size_t i = 0; i < N_FIREFLIES; ++i)
            {
                phase_[i] += phase_inc_[i];
                if (phase_[i] >= 1.0f)
                    phase_[i] -= 1.0f;

                const float on = (phase_[i] < (duty_[i] * density_)) ? 1.0f : 0.0f;

                env_[i] += 0.2f * (on - env_[i]);

                float delay_samps = base_delay_[i];
                if (enable_detune_)
                {
                    const float ph = phase_[i] * (2.0f * float(M_PI));
                    const float s = FastSin(ph * 0.5f);
                    delay_samps += depth_[i] * s;
                }

                const float pan = pan_[i];
                const float gl = 0.5f * (1.0f - pan);
                const float gr = 0.5f * (1.0f + pan);

                const float g = env_[i] * gain_[i];
                ctrl_[i].delay = delay_samps;
                ctrl_[i].gainL = g * gl;
                ctrl_[i].gainR = g * gr;
            }
        }

        inline float FastSin(float x) const
        {
            const float B = 4.0f / M_PI;
            const float C = -4.0f / (M_PI * M_PI);
            float y = B * x + C * x * std::fabs(x);
            const float P = 0.225f;
            y = P * (y * std::fabs(y) - y) + y;
            return y;
        }

        void ConfigureDetune()
        {
            const float cents_scale = detune_cents_ / 1200.0f;
            for (size_t i = 0; i < N_FIREFLIES; ++i)
            {
                const float depth = (enable_detune_ ? (0.015f * cents_scale * base_delay_[i]) : 0.0f);
                depth_[i] = Clamp(depth, 0.0f, 6.0f);
            }
        }

        void ConfigureRates()
        {
            const float ctrl_sr = sr_ / float(control_div_);
            for (size_t i = 0; i < N_FIREFLIES; ++i)
            {
                float hz = base_rate_hz_ * (0.6f + 0.8f * (0.3f + 0.7f * Rand01()));
                phase_inc_[i] = hz / ctrl_sr;
            }
        }

        template <typename X>
        static inline X Clamp(X x, X a, X b) { return (x < a ? a : (x > b ? b : x)); }

        inline float Rand01()
        {
            seed_ = 1664525u * seed_ + 1013904223u;
            return float((seed_ >> 8) & 0x00FFFFFFu) / float(0x00FFFFFFu);
        }

        DelayBufferExt<T, max_size> d_;
        CtrlState ctrl_[N_FIREFLIES];

        float base_delay_[N_FIREFLIES];
        float depth_[N_FIREFLIES];
        float pan_[N_FIREFLIES];
        float gain_[N_FIREFLIES];
        float duty_[N_FIREFLIES];
        float phase_[N_FIREFLIES];
        float phase_inc_[N_FIREFLIES];
        float env_[N_FIREFLIES];

        float sr_, fb_, mix_;
        float min_ms_, max_ms_;
        float detune_cents_;
        float density_;
        float base_rate_hz_;
        uint32_t control_div_;
        uint32_t counter_;
        bool enable_detune_;
        uint32_t seed_;
    };

} // namespace daisyfarm
