#pragma once
#include "modulated_diffuser.h" // (the one we just built)
#include <cmath>
#include <cstddef>

namespace daisyfarm
{

    /**
     * @brief Series network of N modulated allpass diffusers.
     *
     * Features:
     *  - Same semantics as DiffusionNetwork<T,max,N>, but each stage has dynamic modulation.
     *  - "Motion" macro control adjusts both LFO depth and wander amount proportionally.
     *  - Stage parameters (delay & g) remain independently settable.
     *
     * Template:
     *   T        = float
     *   max_size = buffer length for each stage (same as AllpassDiffuser used before)
     *   N        = number of stages (2–8 typical; 4 is classic)
     */
    template <typename T, size_t max_size, size_t N>
    class ModulatedDiffusionNetwork
    {
    public:
        ModulatedDiffusionNetwork()
            : sample_rate_(48000.0f),
              motion_(0.0f)
        {
            for (size_t i = 0; i < N; ++i)
            {
                base_delay_samps_[i] = 150.0f + 50.0f * i;
                base_g_[i] = 0.7f;
            }
        }

        void Init(T *external_buffers[N], float sample_rate)
        {
            sample_rate_ = sample_rate;
            for (size_t i = 0; i < N; ++i)
            {
                apm_[i].Init(external_buffers[i], sample_rate_);
                apm_[i].SetBase(base_delay_samps_[i], base_g_[i]);
            }
            SetMotion(0.0f); // default = static
        }

        void Reset()
        {
            for (size_t i = 0; i < N; ++i)
                apm_[i].Reset();
        }

        /**
         * @param i (0..N-1)
         * @param delay_samps base delay length
         * @param g allpass feedback coefficient
         */
        void SetStage(size_t i, float delay_samps, float g)
        {
            if (i < N)
            {
                base_delay_samps_[i] = delay_samps;
                base_g_[i] = g;
                apm_[i].SetBase(delay_samps, g);
            }
        }

        /**
         * @brief Macro modulation control.
         *
         * motion = 0.0 → no modulation (static)
         * motion = 1.0 → fully lush animated diffusion
         *
         * Internally maps to:
         *   LFO depth      = motion * 2.0 samples (typical gentle movement)
         *   Wander amount  = motion * 0.6 samples (randomized drift)
         *   LFO rate       = gentle (~0.07 Hz)
         *   Wander TC      = ~0.3 Hz smoothing
         */
        void SetMotion(float motion)
        {
            motion_ = (motion < 0.f ? 0.f : (motion > 1.f ? 1.f : motion));

            // Global modulation curves (can be tuned later)
            float depth = motion_ * 2.0f; // up to ~2 samples
            float wander = motion_ * 0.6f;
            float lfo_hz = 0.07f;    // slow scenic movement
            float wander_tc = 0.30f; // slow-pivoting spatial drift

            for (size_t i = 0; i < N; ++i)
            {
                apm_[i].SetLFO(lfo_hz, depth);
                apm_[i].SetWander(wander, wander_tc);
            }
        }

        inline T Process(T x)
        {
            for (size_t i = 0; i < N; ++i)
                x = apm_[i].Process(x); // or ProcessHermite for smoother motion
            return x;
        }

    private:
        float sample_rate_;
        float motion_;

        float base_delay_samps_[N];
        float base_g_[N];

        ModulatedDiffuser<T, max_size> apm_[N];
    };

} // namespace daisyfarm
