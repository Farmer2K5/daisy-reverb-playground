#pragma once
#include "delay_buffer_ext.h"
#include "mix_matrix.h"
#include "shelf_pair.h"
#include "triband_damping.h"
#include <cmath>
#include <cstdint>

namespace daisyfarm
{

    template <typename T, size_t max_size, size_t N = 8>
    class FDNLineModTank
    {
    public:
        void Init(T *external_buffers[N], float sample_rate)
        {
            sr_ = sample_rate;
            for (size_t i = 0; i < N; i++)
            {
                d_[i].Init(external_buffers[i]);
                phase_[i] = 0;
                ylp_[i] = 0;
                a_[i] = 0.6f;
                delay_samps_[i] = 2000.f + 300.f * i;
            }

            mix_.Init(sr_);
            mix_.SetMode(MixMatrix<N>::RANDOM_ORTHO);
            mix_.SetROParams(0.05f, 0.12f);

            tb_.Init(sr_);
            tb_.SetCrossover(300.f, 6000.f);
            tb_.SetDecay(1.05f, 1.0f, 0.78f);

            sh_.Init(sr_);
            sh_.SetCutoff(2500.f);
            sh_.SetLowGain(1.05f);
            sh_.SetHighGain(0.85f);
        }

        void Process(T x, T &outL, T &outR)
        {
            T y[N];
            for (size_t i = 0; i < N; i++)
            {
                float mod = std::sin(phase_[i]) * mod_depth_;
                y[i] = d_[i].Read(delay_samps_[i] + mod);
                phase_[i] += 2.f * 3.14159265f * (mod_rate_ / sr_);
                if (phase_[i] > 2.f * 3.14159265f)
                    phase_[i] -= 2.f * 3.14159265f;
            }

            mix_.Apply(y);
            tb_.ProcessArray<N>(y);
            for (size_t i = 0; i < N; i++)
                y[i] = sh_.Process(y[i]);

            T sum = 0;
            for (size_t i = 0; i < N; i++)
                sum += y[i];
            T base = (feedback_ / T(N)) * sum;

            for (size_t i = 0; i < N; i++)
            {
                T fb = base - T(0.5f) * y[i];
                T filtered = (1.f - a_[i]) * fb + a_[i] * ylp_[i];
                ylp_[i] = filtered;
                d_[i].Write(x + filtered);
            }
            for (size_t i = 0; i < N; i++)
                d_[i].Advance();

            if constexpr (N == 4)
            {
                outL = 0.5f * (y[0] + y[2]);
                outR = 0.5f * (y[1] + y[3]);
            }
            else
            {
                outL = 0.25f * (y[0] + y[3] + y[5] + y[7]);
                outR = 0.25f * (y[1] + y[2] + y[4] + y[6]);
            }
        }

        void SetFeedback(float fb) { feedback_ = fb; }
        void SetModulation(float rate, float depth)
        {
            mod_rate_ = rate;
            mod_depth_ = depth;
        }
        void SetDelay(size_t i, float samp)
        {
            if (i < N)
                delay_samps_[i] = samp;
        }

        // --- Mix matrix control (public API) ---
        void SetMixMode(typename MixMatrix<N>::Mode m) { mix_.SetMode(m); }
        void SetMixRandomOrtho(float rate_hz, float depth)
        {
            mix_.SetMode(MixMatrix<N>::RANDOM_ORTHO);
            mix_.SetROParams(rate_hz, depth);
        }
        void SetMixHadamard() { mix_.SetMode(MixMatrix<N>::HADAMARD); }
        void SetMixHouseholder() { mix_.SetMode(MixMatrix<N>::HOUSEHOLDER); }

    private:
        float sr_ = 48000.f;
        DelayBufferExt<T, max_size> d_[N];
        float delay_samps_[N], phase_[N], ylp_[N], a_[N];
        float feedback_ = 0.78f, mod_rate_ = 0.05f, mod_depth_ = 2.f;

        MixMatrix<N> mix_;
        TriBandDamping tb_;
        ShelfPair sh_;
    };

} // end namespace