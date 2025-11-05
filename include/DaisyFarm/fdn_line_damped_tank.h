/**
 * @file reverb_tank_damped.h
 * @brief NxN FDN reverb tank with per-line damping and modulation.
 */
#pragma once
#include "delay_buffer_ext.h"
#include <cmath>

namespace daisyfarm
{

    template <typename T, size_t max_size, size_t N = 8>
    class FDNLineDampedTank
    {
    public:
        FDNLineDampedTank() : sample_rate_(48000.0f), feedback_(0.75f), mod_rate_(0.1f), mod_depth_(1.0f)
        {
            for (size_t i = 0; i < N; i++)
            {
                delay_time_[i] = 2000.0f + 300.0f * i; // samples (≈ 42–88ms at 48k)
                phase_[i] = 0.0f;
                a_[i] = 0.6f; // damping coefficient (0..1), higher = darker
                ylp_[i] = 0.0f;
            }
        }

        void Init(T *external_buffers[N], float sample_rate)
        {
            for (size_t i = 0; i < N; i++)
                d_[i].Init(external_buffers[i]);
            sample_rate_ = sample_rate;
        }

        void Reset()
        {
            for (size_t i = 0; i < N; i++)
            {
                d_[i].Reset();
                phase_[i] = 0.0f;
                ylp_[i] = 0.0f;
            }
        }

        void SetFeedback(float fb) { feedback_ = fb; }
        void SetModulation(float rate_hz, float depth_samps)
        {
            mod_rate_ = rate_hz;
            mod_depth_ = depth_samps;
        }

        /** @brief Set a single-pole low-pass damping coefficient for all lines.
         *  y[n] = (1-a) * x[n] + a * y[n-1];  a in [0,1). Larger a = stronger damping (darker).
         */
        void SetDampingCoeff(float a)
        {
            for (size_t i = 0; i < N; i++)
                a_[i] = a;
        }

        /** @brief Set damping using cutoff frequency (Hz). */
        void SetDampingFreq(float fc_hz)
        {
            const float a = std::exp(-2.0f * M_PI * fc_hz / sample_rate_);
            SetDampingCoeff(a);
        }

        void SetDelaySamples(size_t i, float delay_samps)
        {
            if (i < N)
                delay_time_[i] = delay_samps;
        }

        /** @brief Process one mono sample to stereo out. */
        void Process(T in, T &outL, T &outR)
        {
            T y[N];
            // 1) Read all lines with modulation
            for (size_t i = 0; i < N; i++)
            {
                const float mod = std::sin(phase_[i]) * mod_depth_;
                y[i] = d_[i].Read(delay_time_[i] + mod);
                phase_[i] += 2.0f * M_PI * mod_rate_ / sample_rate_;
                if (phase_[i] > 2.0f * M_PI)
                    phase_[i] -= 2.0f * M_PI;
            }

            // 2) Householder-like mixing (sum and distribute)
            T sum = 0;
            for (size_t i = 0; i < N; i++)
                sum += y[i];
            const T base = (feedback_ / static_cast<T>(N)) * sum;

            // 3) Damped feedback per line and write
            for (size_t i = 0; i < N; i++)
            {
                // decorrelated per-line feedback signal
                const T fb_i = base - static_cast<T>(0.5f) * y[i];
                // one-pole LP on feedback
                const float a = a_[i];
                const T filtered = static_cast<T>((1.0f - a) * fb_i + a * ylp_[i]);
                ylp_[i] = filtered;
                d_[i].Write(in + filtered);
            }

            // 4) Advance all pointers
            for (size_t i = 0; i < N; i++)
                d_[i].Advance();

            // 5) Stereo mixout (interleaved lines)
            // outL = static_cast<T>(0.25f) * (y[0] + y[3] + y[5] + y[7]);
            // outR = static_cast<T>(0.25f) * (y[1] + y[2] + y[4] + y[6]);
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
        DelayBufferExt<T, max_size> d_[N];
        float delay_time_[N];
        float phase_[N];
        float ylp_[N]; // LPF state per line
        float a_[N];   // LPF coefficient per line
        float sample_rate_;
        float feedback_;
        float mod_rate_, mod_depth_;
    };

} // namespace daisyfarm
