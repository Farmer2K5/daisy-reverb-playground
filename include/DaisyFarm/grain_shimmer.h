#pragma once
#include <cmath>
#include <algorithm>
#include <cstdint>
#include "delay_buffer_ext.h"

// =============================================================
// GrainShimmer: Overlapping granular octave shimmer
//  - Drop-in modular block, similar API to ShimmerPitchShift
//  - Uses moving read heads over a delay buffer to resample grains
//  - Hann-windowed grains with randomized starts and pan
//  - Pitch set by semitones (ratio = 2^(semi/12))
//
// API:
//   template <size_t max_size, size_t NumGrains = 6>
//   class GrainShimmer {
//     void Init(float* external, float sr);
//     void SetParams(float semi, float grain_ms, float density_hz,
//                    float min_ms, float jitter_ms = 10.0f);
//     float Process(float x); // mono
//     void ProcessStereo(float xL, float xR, float& yL, float& yR); // optional
//   };
//
// Notes:
//  - Requires DelayBufferExt<float, max_size> supporting fractional reads.
//  - Keep max_size reasonably larger than (min_ms + grain_ms + jitter_ms) * sr/1000.
//  - For stereo, simple random pan per grain is used for light decorrelation.
// =============================================================

namespace daisyfarm
{
    template <size_t max_size, size_t NumGrains = 6>
    class GrainShimmer
    {
    public:
        void Init(float *external, float sr)
        {
            d_.Init(external);
            sr_ = sr;
            rng_ = 0x1234abcdu;
            // Defaults: clean octave up, short grains, moderate density
            SetParams(+12.0f, /*grain_ms*/ 35.0f, /*density_hz*/ 30.0f,
                      /*min_ms*/ 20.0f, /*jitter_ms*/ 8.0f);
            emitter_phase_ = 0.0f;
            ClearGrains();
        }

        // semi: semitones (+12 = octave up, negative allowed)
        // grain_ms: length of each grain in milliseconds (typically 20-60ms)
        // density_hz: grains per second (10-60 typical)
        // min_ms: base read delay (safety headroom)
        // jitter_ms: random extra delay added to new grains (0..jitter_ms)
        void SetParams(float semi, float grain_ms, float density_hz,
                       float min_ms, float jitter_ms = 10.0f)
        {
            semi_ = semi;
            ratio_ = std::pow(2.0f, semi_ / 12.0f);
            grain_ms_ = std::max(5.0f, grain_ms);
            density_hz_ = std::max(0.5f, density_hz);
            min_delay_ = std::max(1.0f, min_ms * 0.001f * sr_);
            jitter_smp_ = std::max(0.0f, jitter_ms * 0.001f * sr_);
            grain_smp_ = std::max(8.0f, grain_ms_ * 0.001f * sr_);

            // Delay increment per sample needed to read at 'ratio' speed
            // Reading faster than real-time (ratio>1) requires decreasing delay.
            dstep_ = (1.0f - ratio_);
        }

        // Mono process
        inline float Process(float x)
        {
            d_.Write(x);
            d_.Advance();
            EmitIfNeeded();
            float y = 0.0f;
            float wsum = 0.0f;
            for (size_t i = 0; i < NumGrains; ++i)
            {
                if (!g_[i].active)
                    continue;
                const float t = g_[i].age / g_[i].dur;
                if (t >= 1.0f)
                {
                    g_[i].active = false;
                    continue;
                }

                const float w = hann_(t);
                const float s = d_.Read(g_[i].delay);
                y += w * s;
                wsum += w;

                g_[i].delay += dstep_;
                g_[i].age += 1.0f;
            }
            if (wsum <= 1e-6f)
                return 0.0f;
            // Gentle normalization to keep level consistent with overlap
            return y / (wsum + 1e-6f);
        }

        // Stereo process with light random panning per grain.
        inline void ProcessStereo(float xL, float xR, float &yL, float &yR)
        {
            const float x = 0.5f * (xL + xR);
            d_.Write(x);
            d_.Advance();
            EmitIfNeeded();

            float L = 0.0f, R = 0.0f;
            float wsumL = 0.0f, wsumR = 0.0f;

            for (size_t i = 0; i < NumGrains; ++i)
            {
                if (!g_[i].active)
                    continue;
                const float t = g_[i].age / g_[i].dur;
                if (t >= 1.0f)
                {
                    g_[i].active = false;
                    continue;
                }

                const float w = hann_(t);
                const float s = d_.Read(g_[i].delay);

                // equal-power pan
                const float cL = g_[i].panL;
                const float cR = g_[i].panR;

                L += w * s * cL;
                R += w * s * cR;
                wsumL += w * cL;
                wsumR += w * cR;

                g_[i].delay += dstep_;
                g_[i].age += 1.0f;
            }

            yL = (wsumL > 1e-6f) ? (L / (wsumL + 1e-6f)) : 0.0f;
            yR = (wsumR > 1e-6f) ? (R / (wsumR + 1e-6f)) : 0.0f;
        }

        // Optional utility: clear all active grains
        void ClearGrains()
        {
            for (size_t i = 0; i < NumGrains; ++i)
            {
                g_[i].active = false;
                g_[i].age = 0.0f;
                g_[i].dur = grain_smp_;
                g_[i].delay = min_delay_;
                g_[i].panL = 0.7071f;
                g_[i].panR = 0.7071f;
            }
        }

    private:
        struct Grain
        {
            bool active;
            float age;   // samples since start
            float dur;   // total duration in samples
            float delay; // current delay in samples (fractional allowed)
            float panL;  // equal-power coefficients
            float panR;
        };

        inline float hann_(float t) const
        {
            // 0..1 -> Hann window
            return 0.5f - 0.5f * std::cos(2.0f * float(M_PI) * t);
        }

        inline float frand_()
        {
            // xorshift32
            uint32_t x = rng_;
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            rng_ = x;
            return (float)(x) * (1.0f / 4294967296.0f); // 0..1
        }

        inline void EmitIfNeeded()
        {
            emitter_phase_ += density_hz_ / sr_;
            while (emitter_phase_ >= 1.0f)
            {
                emitter_phase_ -= 1.0f;
                SpawnGrain_();
            }
        }

        inline void SpawnGrain_()
        {
            // find a free slot (or the oldest)
            size_t idx = NumGrains;
            float oldest_t = -1.0f;
            for (size_t i = 0; i < NumGrains; ++i)
            {
                if (!g_[i].active)
                {
                    idx = i;
                    break;
                }
                const float t = g_[i].age / std::max(1.0f, g_[i].dur);
                if (t > oldest_t)
                {
                    oldest_t = t;
                    idx = i;
                }
            }

            // (re)initialize grain
            Grain &gr = g_[idx];
            gr.active = true;
            gr.age = 0.0f;
            gr.dur = grain_smp_;

            // randomize additional delay within jitter, clamp inside buffer
            float extra = jitter_smp_ * frand_();
            float d0 = min_delay_ + extra;

            // make sure we stay within the circular buffer safe region
            const float safety = 4.0f;
            if (d0 + grain_smp_ >= max_size - safety)
                d0 = std::max(min_delay_, max_size - safety - grain_smp_ - 1.0f);

            gr.delay = d0;

            // random equal-power pan
            const float pan = 2.0f * frand_() - 1.0f; // -1..1
            // map to angle 0..pi/2 for equal-power
            const float a = 0.25f * float(M_PI) * (pan + 1.0f);
            gr.panL = std::cos(a);
            gr.panR = std::sin(a);
        }

        DelayBufferExt<float, max_size> d_;

        // Params
        float sr_ = 48000.0f;
        float semi_ = 12.0f;
        float ratio_ = 2.0f;
        float grain_ms_ = 35.0f;
        float grain_smp_ = 1600.0f;
        float density_hz_ = 30.0f;
        float min_delay_ = 20.0f;
        float jitter_smp_ = 0.0f;

        // Derived
        float dstep_ = -1.0f; // delay delta per sample

        // State
        Grain g_[NumGrains];
        float emitter_phase_ = 0.0f;
        uint32_t rng_ = 0x1234abcdu;
    };
}