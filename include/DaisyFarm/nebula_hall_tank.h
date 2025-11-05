#pragma once
#include "delay_buffer_ext.h"
#include "mix_matrix.h"
#include "shelf_pair.h"
#include "shimmer_pitch_shift.h"
#include "triband_damping.h"
#include <cmath>
#include <cstdint>

namespace daisyfarm
{

    // =============================================================
    // FDN Tank with: Freeze + Dual Shimmer + Spread + PreDelay + Dynamics + Bloom
    // + DUNE (Nebula Drift) + WANDER (meta) + HOLO Dream Space (phase-lattice halo)
    // =============================================================
    template <typename T, size_t max_size, size_t N = 8>
    class NebulaHallTank
    {
    public:
        void Init(T *ext_buf[N], float sr, float *shimmer_buf, size_t shimmer_size)
        {
            (void)shimmer_size;
            sr_ = sr;
            for (size_t i = 0; i < N; i++)
            {
                d_[i].Init(ext_buf[i]);
                phase_[i] = 0;
                ylp_[i] = 0;
                a_[i] = 0.6f;
                delay_samps_[i] = 2000.f + 300.f * i;
                shimmer_spread_[i] = (i & 1) ? +1.f : -1.f;
                dune_phase_off_[i] = fmodf(float(i) * 0.6180339887f * 6.283185307f, 6.283185307f); // golden-ratio offsets
            }
            mix_.Init(sr_);
            mix_.SetMode(MixMatrix<N>::RANDOM_ORTHO);
            mix_.SetROParams(0.05f, 0.12f);
            tb_.Init(sr_);
            tb_.SetCrossover(300.f, 6000.f);
            tb_.SetDecay(1.05f, 1.0f, 0.78f);
            shf_.Init(sr_);
            shf_.SetCutoff(2500.f);
            shf_.SetLowGain(1.05f);
            shf_.SetHighGain(0.85f);

            sh1_.Init(shimmer_buf, sr_); // +12
            sh2_.Init(shimmer_buf, sr_); // +24 share ok

            // predelay init
            predelay_len_ = 1;
            predelay_wptr_ = 0;
            for (auto &v : predelay_buf_)
                v = 0.f;

            // HOLO initial states
            holo_m1_ = holo_m2_ = holo_s1_ = holo_s2_ = 0.f;
        }

        // Params
        void SetFeedback(float fb) { feedback_ = fb; }

        void SetModulation(float r, float d)
        {
            mod_rate_ = r;
            mod_depth_ = d;
        }

        void SetDelay(size_t i, float s)
        {
            if (i < N)
                delay_samps_[i] = s;
        }

        void SetFreeze(bool on) { freeze_ = on; }

        void SetDamping(float lowDecay, float midDecay, float highDecay)
        {
            tb_.SetDecay(lowDecay, midDecay, highDecay);
        }

        void SetDampingCrossover(float lowHz, float highHz)
        {
            tb_.SetCrossover(lowHz, highHz);
        }

        // --- Tone / Tilt Controls (ShelfPair) ---
        void SetToneCutoff(float fc) { shf_.SetCutoff(fc); }
        void SetToneLow(float g) { shf_.SetLowGain(g); }
        void SetToneHigh(float g) { shf_.SetHighGain(g); }

        // Convenience: single "tone warmth" macro control
        // amt > 1.0 → warmer | amt < 1.0 → brighter
        void SetToneWarmth(float amt)
        {
            shf_.SetLowGain(amt);
            shf_.SetHighGain(2.0f - amt); // symmetrical tilt curve
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

        void SetShimmer(bool on, float send = 0.12f)
        {
            shimmer_on_ = on;
            shimmer_send_ = send;
        }

        void SetShimmerParams(float semi = +12.f, float rate_hz = 0.06f, float sw_ms = 30.f, float mn_ms = 20.f)
        {
            sh1_.SetParams(semi, rate_hz, sw_ms, mn_ms);
            sh2_.SetParams(semi * 2.f, rate_hz * 0.5f, sw_ms * 1.3f, mn_ms);
        }

        void SetShimmerChorus(float rate_hz, float depth)
        {
            chorus_rate_ = rate_hz;
            chorus_depth_ = depth;
        }

        void SetPreDelay(float ms)
        {
            float s = (ms * 0.001f) * sr_;
            if (s < 1)
                s = 1;
            if (s > (float)(sizeof(predelay_buf_) / sizeof(predelay_buf_[0]) - 2))
                s = (float)(sizeof(predelay_buf_) / sizeof(predelay_buf_[0]) - 2);
            predelay_len_ = (size_t)s;
        }

        void SetDynamics(float amt, float atk_ms = 5.f, float rel_ms = 200.f)
        {
            dyn_amount_ = amt;
            env_attack_ = std::exp(-1.f / ((atk_ms * 0.001f) * sr_));
            env_release_ = std::exp(-1.f / ((rel_ms * 0.001f) * sr_));
        }

        void SetBloom(float amt)
        {
            if (amt < 0)
                amt = 0;
            if (amt > 1)
                amt = 1;
            bloom_time_ = amt;
        }

        // ---- DUNE (Nebula Drift) ----
        void SetDune(bool on, float rate_hz = 0.10f, float space_depth = 0.18f, float size_depth_samps = 0.75f)
        {
            dune_on_ = on;
            dune_rate_base_ = rate_hz;
            dune_rate_ = rate_hz;
            dune_space_depth_ = space_depth;
            dune_size_base_ = size_depth_samps;
            dune_size_depth_ = size_depth_samps;
            if (dune_on_)
                mix_.SetROParams(dune_rate_, dune_space_depth_);
        }

        // ---- WANDER (meta-modulation) ----
        void SetWander(bool on, float rateDepth = 0.12f, float sizeDepth = 0.18f)
        {
            wander_on_ = on;
            wander_rate_depth_ = rateDepth;
            wander_size_depth_ = sizeDepth;
        }

        // ---- HOLO Dream Space ----
        // amount: 0..1 (intensity of halo)
        // tilt:   0..1 (0 = darker, 1 = brighter halo emphasis)
        // xfeed:  0..1 (0 = none, 1 = strong crossfeed)
        void SetHoloDream(float amount = 0.35f, float tilt = 0.6f, float xfeed = 0.12f)
        {
            holo_amount_ = (amount < 0 ? 0 : amount > 1 ? 1
                                                        : amount);
            holo_tilt_ = (tilt < 0 ? 0 : tilt > 1 ? 1
                                                  : tilt);
            holo_xfeed_ = (xfeed < 0 ? 0 : xfeed > 1 ? 1
                                                     : xfeed);
        }

        // BLM (Bloom Rise Shape)  0 = no bloom, 1 = wide cinematic
        void SetBloomAmount(float amt)
        {
            // perceptual curve
            if (amt < 0.f)
                amt = 0.f;
            if (amt > 1.f)
                amt = 1.f;
            float mapped = 0.05f + amt * 0.45f; // range: 0.05 → 0.50
            SetBloom(mapped);
        }

        // SHM (Shimmer Lift Amount)  0 = none, 1 = pronounced angel pad
        void SetShimmerAmount(float amt)
        {
            if (amt < 0.f)
                amt = 0.f;
            if (amt > 1.f)
                amt = 1.f;
            shimmer_send_ = 0.02f + amt * 0.28f; // clear, never hazy range
        }

        // CLD (Cloud / Tail Thickness) adjusts feedback vs damping
        void SetCloudDensity(float amt)
        {
            if (amt < 0.f)
                amt = 0.f;
            if (amt > 1.f)
                amt = 1.f;
            float fb = 0.76f + amt * 0.14f;  // 0.76 → 0.90
            float mid = 1.00f - amt * 0.12f; // 1.00 → 0.88
            SetFeedback(fb);
            SetDamping(1.03f, mid, 0.78f);
        }

        // TLT (Tone Warm/Cold tilt)  0 = bright plate, 1 = warm hall
        void SetTone(float amt)
        {
            if (amt < 0.f)
                amt = 0.f;
            if (amt > 1.f)
                amt = 1.f;
            float low = 1.00f + amt * 0.12f;
            float high = 1.00f - amt * 0.14f;
            SetToneLow(low);
            SetToneHigh(high);
        }

        // Audio
        void Process(T x, T &outL, T &outR)
        {
            // PreDelay
            predelay_buf_[predelay_wptr_] = x;
            predelay_wptr_++;
            if (predelay_wptr_ >= predelay_len_)
                predelay_wptr_ = 0;
            float x_delayed = predelay_buf_[predelay_wptr_];

            // Dynamics envelope
            float ia = std::fabs(x_delayed);
            if (ia > env_)
                env_ = env_attack_ * env_ + (1.f - env_attack_) * ia;
            else
                env_ = env_release_ * env_ + (1.f - env_release_) * ia;

            // Advance DUNE + WANDER
            if (dune_on_)
            {
                // base dune osc
                dune_phase_ += 2.f * 3.14159265f * (dune_rate_ / sr_);
                if (dune_phase_ > 2.f * 3.14159265f)
                    dune_phase_ -= 2.f * 3.14159265f;

                // wander meta
                if (wander_on_)
                {
                    wander_phase_rate_ += wander_rate_speed_;
                    wander_phase_size_ += wander_size_speed_;
                    if (wander_phase_rate_ > 1.f)
                        wander_phase_rate_ -= 1.f;
                    if (wander_phase_size_ > 1.f)
                        wander_phase_size_ -= 1.f;

                    float rate_mod = 1.f + wander_rate_depth_ * std::sin(2.f * 3.14159265f * wander_phase_rate_);
                    float size_mod = 1.f + wander_size_depth_ * std::cos(2.f * 3.14159265f * wander_phase_size_);

                    dune_rate_ = dune_rate_base_ * rate_mod;
                    dune_size_depth_ = dune_size_base_ * size_mod;
                    mix_.SetROParams(dune_rate_, dune_space_depth_);
                }
            }

            // Read modulated delay lines (with optional DUNE size modulation)
            T y[N];
            for (size_t i = 0; i < N; i++)
            {
                float mod = std::sin(phase_[i]) * mod_depth_;
                if (dune_on_)
                {
                    float dshift = dune_size_depth_ * std::sin(dune_phase_ + dune_phase_off_[i]); // <= ~1 sample
                    mod += dshift;
                }
                y[i] = d_[i].Read(delay_samps_[i] + mod);
                phase_[i] += 2.f * 3.14159265f * (mod_rate_ / sr_);
                if (phase_[i] > 2.f * 3.14159265f)
                    phase_[i] -= 2.f * 3.14159265f;
            }

            // Spatial + damping + tone
            mix_.Apply(y);
            tb_.ProcessArray<N>(y);

            // Dynamics spectral tilt
            float dyn = 1.f - (dyn_amount_ * env_);
            shf_.SetHighGain(0.85f * dyn + 0.15f);

            for (size_t i = 0; i < N; i++)
                y[i] = shf_.Process(y[i]);

            // Base feedback
            T sum = 0;
            for (size_t i = 0; i < N; i++)
                sum += y[i];
            T base = ((freeze_ ? 1.f : feedback_) / T(N)) * sum;

            // Shimmer
            if (shimmer_on_)
            {
                float mono = sum * (1.f / float(N));

                float p1 = sh1_.Process(mono);
                shimmer_lp1_ = 0.92f * shimmer_lp1_ + 0.08f * p1;
                float shA = shimmer_lp1_;

                float p2 = sh2_.Process(mono);
                shimmer_lp2_ = 0.94f * shimmer_lp2_ + 0.06f * p2;

                chorus_phase_ += (chorus_rate_ / sr_);
                if (chorus_phase_ >= 1.f)
                    chorus_phase_ -= 1.f;
                float det = 1.f + chorus_depth_ * std::sin(2.f * 3.14159265f * chorus_phase_);
                float shB = shimmer_lp2_ * det;

                float sh_final = (0.8f * shA) + (0.35f * shB);

                // for (size_t i = 0; i < N; i++)
                //     y[i] += T(shimmer_send_) * T(sh_final) * T(shimmer_spread_[i]);
                // CHORAL SPLIT: send shimmer only to two opposite lines
                y[1] += shimmer_send_ * sh_final;
                y[5] += shimmer_send_ * sh_final;
            }

            // Write feedback
            for (size_t i = 0; i < N; i++)
            {
                T fb = base - T(0.5f) * y[i];
                T flt = (1.f - a_[i]) * fb + a_[i] * ylp_[i];
                ylp_[i] = flt;
                if (freeze_)
                    d_[i].Write(flt);
                else
                    d_[i].Write(x_delayed + flt);
            }
            for (size_t i = 0; i < N; i++)
                d_[i].Advance();

            // Mix stereo
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

            // Bloom diffusion
            {
                float a = 0.38f + 0.45f * bloom_time_;
                auto AP = [&](float x, float &z)
                { float y=-a*x+z; z=x+a*y; return y; };
                outL = AP(outL, bL1_);
                outL = AP(outL, bL2_);
                outL = AP(outL, bL3_);
                outL = AP(outL, bL4_);
                outR = AP(outR, bR1_);
                outR = AP(outR, bR2_);
                outR = AP(outR, bR3_);
                outR = AP(outR, bR4_);
            }

            // HOLO Dream Space (phase-lattice halo in Mid/Side)
            if (holo_amount_ > 0.f)
            {
                float M = 0.70710678f * (outL + outR);
                float S = 0.70710678f * (outL - outR);

                // frequency-tilted phase lattice (2-stage per branch)
                // coefficients lean brighter with tilt, darker when tilt < 0.5
                float t = holo_tilt_;
                float aM1 = 0.60f + 0.25f * t; // mid branch slightly darker
                float aM2 = 0.50f + 0.20f * t;
                float aS1 = 0.70f + 0.25f * t; // side branch a bit brighter
                float aS2 = 0.55f + 0.20f * t;

                // allpass: y = -a*x + z; z = x + a*y;
                auto AP1 = [](float x, float a, float &z)
                { float y = -a*x + z; z = x + a*y; return y; };

                float Mm = AP1(M, aM1, holo_m1_);
                Mm = AP1(Mm, aM2, holo_m2_);
                float Ss = AP1(S, aS1, holo_s1_);
                Ss = AP1(Ss, aS2, holo_s2_);

                // crossfeed (subtle)
                float xf = holo_xfeed_;
                float Mc = Mm + xf * Ss;
                float Sc = Ss + xf * Mm;

                // blend holo with dry via amount
                float dryL = outL, dryR = outR;
                float holoL = 0.70710678f * (Mc + Sc);
                float holoR = 0.70710678f * (Mc - Sc);

                outL = (1.f - holo_amount_) * dryL + holo_amount_ * holoL;
                outR = (1.f - holo_amount_) * dryR + holo_amount_ * holoR;
            }
        }

    private:
        float sr_;

        DelayBufferExt<T, max_size> d_[N];
        float delay_samps_[N], phase_[N], ylp_[N], a_[N];
        float shimmer_spread_[N];

        MixMatrix<N> mix_;
        TriBandDamping tb_;
        ShelfPair shf_;

        ShimmerPitchShift<max_size> sh1_, sh2_;
        float shimmer_lp1_ = 0.f, shimmer_lp2_ = 0.f;
        bool shimmer_on_ = false;
        float shimmer_send_ = 0.12f;
        float chorus_phase_ = 0.f, chorus_rate_ = 0.12f, chorus_depth_ = 0.0025f;

        bool freeze_ = false;
        float feedback_ = 0.8f, mod_rate_ = 0.06f, mod_depth_ = 2.f;

        // predelay
        size_t predelay_len_ = 1, predelay_wptr_ = 0;
        float predelay_buf_[2048] = {}; // ~42ms at 48k; increase if desired.

        // dynamics
        float env_ = 0.f, dyn_amount_ = 0.25f, env_attack_ = 0.005f, env_release_ = 0.2f;

        // bloom diffusion
        float bloom_time_ = 0.35f;
        float bL1_ = 0.f, bL2_ = 0.f, bL3_ = 0.f, bL4_ = 0.f;
        float bR1_ = 0.f, bR2_ = 0.f, bR3_ = 0.f, bR4_ = 0.f;

        // DUNE (Nebula Drift)
        bool dune_on_ = false;
        float dune_rate_base_ = 0.10f;
        float dune_rate_ = 0.10f;        // Hz (runtime-modulated)
        float dune_space_depth_ = 0.18f; // MixMatrix rotation depth
        float dune_size_base_ = 0.75f;   // samples
        float dune_size_depth_ = 0.75f;  // runtime-modulated
        float dune_phase_ = 0.0f;
        float dune_phase_off_[N];

        // WANDER meta-mod
        bool wander_on_ = false;
        float wander_phase_rate_ = 0.0f;
        float wander_phase_size_ = 0.0f;
        float wander_rate_speed_ = 0.00012f; // very slow (cycles over minutes)
        float wander_size_speed_ = 0.00007f; // even slower
        float wander_rate_depth_ = 0.12f;    // ±12%
        float wander_size_depth_ = 0.18f;    // ±18%

        // HOLO dream space (mid/side phase lattice)
        float holo_amount_ = 0.35f;
        float holo_tilt_ = 0.60f;
        float holo_xfeed_ = 0.12f;
        float holo_m1_ = 0.f, holo_m2_ = 0.f, holo_s1_ = 0.f, holo_s2_ = 0.f;
    };
}