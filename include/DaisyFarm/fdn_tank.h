/**
 * @file reverb_tank.h
 * @brief Compact FDN Reverb using DelayBufferExt for Daisy Seed.
 *
 * Inspired by Zita-rev and Valhalla-style designs.
 * Implements an NxN feedback delay network (FDN) with modulation and diffusion.
 */
#pragma once
#include "delay_buffer_ext.h"
#include <cmath>

namespace daisyfarm
{

    template <typename T, size_t max_size, size_t N = 8>
    class FDNTank
    {
    public:
        FDNTank() {}

        /** @brief Initialize all delay buffers with external SDRAM memory. */
        void Init(T *external_buffers[N], float sample_rate)
        {
            for (size_t i = 0; i < N; i++)
            {
                delay_[i].Init(external_buffers[i]);
                delay_time_[i] = (0.05f + 0.015f * i) * sample_rate; // 50–155 ms
                phase_[i] = 0.0f;
            }
            sample_rate_ = sample_rate;
            feedback_ = 0.75f;
            mod_depth_ = 1.0f; // samples
            mod_rate_ = 0.1f;  // Hz
        }

        /** @brief Set reverb feedback amount (0–1). */
        void SetFeedback(float fb) { feedback_ = fb; }

        /** @brief Set modulation rate (Hz) and depth (samples). */
        void SetModulation(float rate, float depth)
        {
            mod_rate_ = rate;
            mod_depth_ = depth;
        }

        /**
         * @brief Process one input sample.
         * @param in Input sample (mono)
         * @param outL Left channel output
         * @param outR Right channel output
         */
        void Process(T in, T &outL, T &outR)
        {
            T y[N];

            // 1. Read all delay taps with modulation
            for (size_t i = 0; i < N; i++)
            {
                float mod = sinf(phase_[i]) * mod_depth_;
                y[i] = delay_[i].Read(delay_time_[i] + mod);
                phase_[i] += 2.0f * M_PI * mod_rate_ / sample_rate_;
                if (phase_[i] > 2.0f * M_PI)
                    phase_[i] -= 2.0f * M_PI;
            }

            // 2. Apply Householder feedback mixing
            T sum = 0;
            for (size_t i = 0; i < N; i++)
                sum += y[i];
            sum *= (feedback_ / static_cast<T>(N));

            // 3. Write input + feedback mixture back into each line
            for (size_t i = 0; i < N; i++)
            {
                T input = in + (sum - y[i]); // decorrelated input
                delay_[i].Write(input);
            }

            // 4. Advance all write pointers
            for (size_t i = 0; i < N; i++)
                delay_[i].Advance();

            // 5. Mix outputs to stereo
            // outL = (y[0] + y[3] + y[5] + y[7]) * 0.25f;
            // outR = (y[1] + y[2] + y[4] + y[6]) * 0.25f;
            // To make this scale, replace the fixed-index sum with a decorrelated stereo panner
            outL = 0;
            outR = 0;
            for (size_t i = 0; i < N; i++)
            {
                float pan = float(i) / float(N - 1); // 0 = Left, 1 = Right
                outL += y[i] * (1.0f - pan);
                outR += y[i] * pan;
            }
            outL /= N;
            outR /= N;
        }

    private:
        DelayBufferExt<T, max_size> delay_[N];
        float delay_time_[N];
        float phase_[N];
        float sample_rate_;
        float feedback_;
        float mod_rate_;
        float mod_depth_;
    };

} // namespace daisyfarm
