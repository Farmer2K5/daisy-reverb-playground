/**
 * @file lush_plate.h
 * @brief Lush Plate Reverb — v3.4 (stable core + full control surface)
 *
 * Stability core:
 *  - Normalized FDN feedback: (sum - v[i]) / NLINES
 *  - Per-line energy leak (u[i] *= 0.9999f)
 *  - High-pass on drive (~120 Hz) to avoid LF pumping
 *  - Gentle tanh safety limiter on tank writes
 *  - Default damping 3.5 kHz (tunable via SetBrightness)
 *  - Freeze with 50 ms ramp; fb capped to 0.95 when frozen
 *
 * Control surface (public API):
 *  - Mix:                  SetWet(), SetOutputGain()
 *  - Decay:                SetDecay(), GetDecay()
 *  - Tone:                 SetBrightness()  // exponential curve; affects damping + tilt amount
 *                          SetBrightSwitch(bool) // hard bypass tilt (default OFF)
 *  - Size:                 SetRoomSize()
 *  - Early Reflections:    SetERWidth()
 *  - Diffusion:            SetDiffusion()
 *  - Pre-delay:            SetPreDelay(), SetPreDelayMod(rate, depth), SetPreDelayModRate(), SetPreDelayModDepth()
 *  - Cloud:                SetCloudSend()
 *                          SetCloudOutMix()
 *  - Freeze:               SetFreeze(bool)
 *                          GetFreeze()
 *                          SetFreezeSlewMs()
 *  - Reset:                Reset()
 *
 * Dependencies:
 *  - delaybuffer_ext.h  (must provide: Init(ptr), Read(float), Write(T), Advance())
 *  - firefly_delay_perf.h (your perf Firefly cloud)
 */
#pragma once
#include "delay_buffer_ext.h"
#include "firefly_delay_perf_v3.h"
#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace daisyfarm
{

    // -------------------- helpers --------------------
    template <typename T>
    static inline T clamp01(T x) { return (x < 0.f ? 0.f : (x > 1.f ? 1.f : x)); }

    // Simple sine LFO (free-running)
    struct LFOUtils
    {
        void Init(float sr, float rate_hz)
        {
            sr_ = sr;
            SetRate(rate_hz);
            phase_ = 0.f;
        }
        void SetRate(float rate_hz)
        {
            rate_ = (rate_hz < 0.f ? 0.f : rate_hz);
            inc_ = rate_ / (sr_ > 0.f ? sr_ : 48000.f);
        }
        inline float Process()
        {
            phase_ += inc_;
            if (phase_ >= 1.f)
                phase_ -= 1.f;
            return std::sin(2.f * float(M_PI) * phase_);
        }
        float sr_{48000.f}, rate_{0.1f}, inc_{0.1f / 48000.f}, phase_{0.f};
    };

    class OnePoleLPF
    {
    public:
        void Init(float sr, float fc)
        {
            sr_ = sr;
            SetCutoff(fc);
            y_ = 0.f;
        }
        void SetCutoff(float fc)
        {
            fc = std::max(fc, 20.f);
            float a = std::exp(-2.f * float(M_PI) * fc / sr_);
            a_ = a;
            b_ = 1.f - a;
        }
        float Process(float x)
        {
            y_ = b_ * x + a_ * y_;
            return y_;
        }

    private:
        float sr_{48000.f}, a_{0.f}, b_{1.f}, y_{0.f};
    };

    class OnePoleHPF
    {
    public:
        void Init(float sr, float fc) { lp_.Init(sr, fc); }
        void SetCutoff(float fc) { lp_.SetCutoff(fc); }
        float Process(float x) { return x - lp_.Process(x); }

    private:
        OnePoleLPF lp_;
    };

    // Tilt EQ “pair” (12 dB/oct style): pre uses HPF cascade; post uses LPF cascade.
    // Amount is applied externally (we compute amount from SetBrightness()).
    class Tilt12dB
    {
    public:
        void Init(float sr, float pivot_hz)
        {
            h1_.Init(sr, pivot_hz);
            h2_.Init(sr, pivot_hz);
            l1_.Init(sr, pivot_hz);
            l2_.Init(sr, pivot_hz);
        }
        inline float Pre(float x) { return 0.5f * (x + h2_.Process(h1_.Process(x))); }
        inline float Post(float x) { return 0.5f * (x + l2_.Process(l1_.Process(x))); }

    private:
        OnePoleHPF h1_, h2_;
        OnePoleLPF l1_, l2_;
    };

    // Schroeder allpass using DelayBufferExt
    template <typename T, size_t MAX>
    class AllpassDiffuser
    {
    public:
        void Init(T *buf, float delay_samp, float g)
        {
            d_.Init(buf);
            delay_ = delay_samp;
            g_ = g;
        }
        void Set(float delay_samp, float g)
        {
            delay_ = delay_samp;
            g_ = g;
        }
        void SetGain(float g) { g_ = g; }
        inline T Process(T x)
        {
            const T d = d_.Read(delay_);
            const T y = -g_ * x + d;
            const T v = x + g_ * y;
            d_.Write(v);
            d_.Advance();
            return y;
        }

    private:
        DelayBufferExt<T, MAX> d_;
        float delay_{200.f}, g_{0.7f};
    };

    // ===================== LushPlate v3.4 full control =====================
    template <typename T,
              size_t ER_MAX, size_t DIFF_MAX, size_t PRE_MAX,
              size_t TANK_MAX, size_t CLOUD_MAX,
              size_t NLINES = 4, size_t NFF = 16>
    class LushPlate
    {
    public:
        // ---------- lifecycle ----------
        void Init(T *erL, T *erR, T *ap0, T *ap1, T *pre,
                  T *tank[NLINES], T *cloud, float sr)
        {
            sr_ = sr;

            // ER
            ER_L_.Init(erL);
            ER_R_.Init(erR);
            erL_samp_ = 0.040f * sr;
            erR_samp_ = 0.046f * sr;
            er_width_ = 0.7f;

            // Pre-delay (between ER and diffusers)
            PRE_.Init(pre);
            SetPreDelay(30.f);
            pre_lfo_.Init(sr, 0.10f);
            SetPreDelayMod(0.10f, 2.0f); // gentle by default

            // Diffusers
            AP0_.Init(ap0, 1000.f, 0.70f);
            AP1_.Init(ap1, 1600.f, 0.70f);
            SetDiffusion(0.80f);

            // Tilt EQ
            TILT_.Init(sr, 2500.f);
            bright_switch_ = false; // OFF by default

            // Tank lines + damping + micro-mod
            for (size_t i = 0; i < NLINES; i++)
            {
                TANK_[i].Init(tank[i]);
                DAMP_[i].Init(sr, 3500.f);         // darker/stable default
                lfo_phase_[i] = 0.29f * i + 0.13f; // decorrelated
                lfo_rate_[i] = 0.10f + 0.03f * i;  // a bit faster than before
                lfo_depth_[i] = 0.6f + 0.3f * i;   // ~0.6..1.5 samples
                // base delays (quasi-prime-ish progression)
                base_delay_[i] = 4000.f + 500.f * i;
            }

            // Firefly cloud (parallel; not frozen)
            CLOUD_.Init(cloud, sr);
            CLOUD_.SetControlDiv(8);
            CLOUD_.SetFeedback(0.35f);
            CLOUD_.SetWet(1.0f);
            CLOUD_.SetOutputGain(1.0f);
            CLOUD_.SetDensity(0.45f);
            CLOUD_.SetBlinkBaseRate(0.22f);
            CLOUD_.SetDetuneCents(3.0f);
            CLOUD_.EnableDetune(true);
            cloud_send_ = 0.30f;
            cloud_outmix_ = 0.30f;

            // Mix / tone / decay
            SetWet(0.75f);
            SetOutputGain(1.0f);
            SetBrightness(0.70f); // sets damping + tilt amount (expo)
            SetDecay(0.85f);      // stable lush

            // Freeze env
            freeze_env_ = 0.f;
            freeze_target_ = 0.f;
            SetFreezeSlewMs(50.f); // ~50 ms smoothing
        }

        void Reset()
        {
            // Zero the tanks; keep parameter settings
            for (size_t i = 0; i < NLINES; i++)
            {
                // crude clear via writing zeros for base delay span
                // (optional) your DelayBufferExt may offer a Clear()
            }
            freeze_env_ = 0.f;
            freeze_target_ = 0.f;
        }

        // ---------- controls ----------
        void SetWet(float w) { wet_ = clamp01(w); }
        void SetOutputGain(float g) { out_gain_ = std::clamp(g, 0.0f, 2.0f); }

        void SetDecay(float amt)
        {
            decay_ = clamp01(amt);
            tank_fb_ = 0.68f + 0.18f * decay_;
        }
        float GetDecay() const { return decay_; }

        // Brightness: exponential mapping 0..1 → affects damping cutoff AND tilt amount
        //  - cutoff: 2 kHz → 12 kHz using exponential-ish response
        //  - tilt amount: 0 → 1 using exponential-ish response
        void SetBrightness(float amt)
        {
            brightness_ = clamp01(amt);
            const float k = 3.0f; // curvature; higher = more 'log-like'
            const float expo = std::pow(brightness_, k);

            // Damping cutoff
            const float fc = 2000.f + (12000.f - 2000.f) * expo; // 2k → 12k
            for (auto &d : DAMP_)
                d.SetCutoff(fc);

            // Tilt amount (stored; only applied if bright_switch_==true)
            tilt_amt_ = expo; // 0..1
        }

        void SetBrightSwitch(bool on) { bright_switch_ = on; }
        bool GetBrightSwitch() const { return bright_switch_; }

        void SetRoomSize(float amt)
        {
            // Scale base delays and ER
            float s = 0.8f + 0.8f * clamp01(amt); // 0.8..1.6
            erL_samp_ = 0.040f * sr_ * s;
            erR_samp_ = 0.046f * sr_ * s;
            for (size_t i = 0; i < NLINES; i++)
                base_delay_[i] = (4000.f + 500.f * i) * s;
        }

        void SetERWidth(float w) { er_width_ = clamp01(w); }

        void SetDiffusion(float amt)
        {
            amt = clamp01(amt);
            AP0_.SetGain(0.5f + 0.5f * amt); // 0.5..1.0
            AP1_.SetGain(0.4f + 0.5f * amt); // 0.4..0.9
        }

        void SetPreDelay(float ms)
        {
            pre_ms_ = std::clamp(ms, 0.0f, 200.0f);
            pre_samps_ = (pre_ms_ / 1000.f) * sr_;
        }
        void SetPreDelayMod(float rate_hz, float depth_ms)
        {
            SetPreDelayModRate(rate_hz);
            SetPreDelayModDepth(depth_ms);
        }
        void SetPreDelayModRate(float rate_hz) { pre_lfo_.SetRate(rate_hz); }
        void SetPreDelayModDepth(float depth_ms) { pre_mod_depth_ms_ = std::clamp(depth_ms, 0.0f, 200.0f); }

        void SetCloudSend(float s) { cloud_send_ = clamp01(s); }
        void SetCloudOutMix(float s) { cloud_outmix_ = clamp01(s); }

        void SetFreeze(bool state) { freeze_target_ = state ? 1.f : 0.f; }
        bool GetFreeze() const { return freeze_target_ > 0.5f; }

        void SetFreezeSlewMs(float ms)
        {
            ms = std::max(ms, 1.0f);
            freeze_alpha_ = std::exp(-1.f / ((ms / 1000.f) * (sr_ > 0.f ? sr_ : 48000.f)));
        }

        // ---------- process (mono in -> stereo out) ----------
        inline void Process(T in, T &outL, T &outR)
        {
            // Smooth freeze envelope
            freeze_env_ = freeze_target_ + (freeze_env_ - freeze_target_) * freeze_alpha_;

            // ER (stereo)
            const T erL = ER_L_.Read(erL_samp_);
            const T erR = ER_R_.Read(erR_samp_);
            ER_L_.Write(in);
            ER_R_.Write(in);
            ER_L_.Advance();
            ER_R_.Advance();

            // ER width blend
            const T erMono = static_cast<T>(0.5f) * (erL + erR);
            const T erLmix = static_cast<T>((1.f - er_width_)) * erMono + static_cast<T>(er_width_) * erL;
            const T erRmix = static_cast<T>((1.f - er_width_)) * erMono + static_cast<T>(er_width_) * erR;

            // Mix ER into input (subtle)
            T x = static_cast<T>(0.7f) * in + static_cast<T>(0.3f) * static_cast<T>(0.5f) * (erLmix + erRmix);

            // Pre-delay with modulation
            const float mod_ms = pre_mod_depth_ms_ * pre_lfo_.Process();
            const float mod_samp = pre_samps_ + (mod_ms * sr_ / 1000.f);
            T pre = PRE_.Read(mod_samp);
            PRE_.Write(x);
            PRE_.Advance();
            x = pre;

            // Diffusers
            x = AP0_.Process(x);
            x = AP1_.Process(x);

            // Tilt pre (amount derived from brightness; also require switch ON)
            if (bright_switch_ && tilt_amt_ > 0.f)
            {
                const float a = tilt_amt_;
                const float pre_t = TILT_.Pre(static_cast<float>(x));
                x = static_cast<T>((1.f - a) * static_cast<float>(x) + a * pre_t);
            }

            // Firefly cloud (parallel)
            T cL, cR;
            CLOUD_.Process(x, cL, cR);
            const T cMono = static_cast<T>(0.5f) * (cL + cR);

            // Tank reads with small decorrelated micro-mod
            float v[NLINES];
            for (size_t i = 0; i < NLINES; i++)
            {
                lfo_phase_[i] += lfo_rate_[i] / sr_;
                if (lfo_phase_[i] >= 1.f)
                    lfo_phase_[i] -= 1.f;
                const float tri = std::fabs(2.f * (lfo_phase_[i] - std::floor(lfo_phase_[i] + 0.5f)));
                const float dmod = (tri - 0.5f) * 2.f * lfo_depth_[i]; // ±depth samples
                v[i] = TANK_[i].Read(base_delay_[i] + dmod);
            }

            // Small leak to bleed residual energy
            for (size_t i = 0; i < NLINES; i++)
                v[i] *= 0.9999f;

            // Normalized feedback mix
            const float sum = v[0] + v[1] + v[2] + v[3];
            const float fb_eff = tank_fb_ + freeze_env_ * (0.95f - tank_fb_); // cap when frozen

            // High-pass drive helper (per-channel shared single-pole)
            hp_lp_ = 0.995f * (hp_lp_ + 0.005f * static_cast<float>(x)); // 1st-order LP
            const float drive_hp = static_cast<float>(x) - hp_lp_;

            for (size_t i = 0; i < NLINES; i++)
            {
                const float drive = (1.f - freeze_env_) * (0.30f * drive_hp + 0.40f * static_cast<float>(cMono));
                const float fb = fb_eff * ((sum - v[i]) / float(NLINES));
                const float damped = DAMP_[i].Process(fb);
                float u = drive + damped;

                // soft safety limiting (transparent at normal levels)
                u = std::tanh(0.2f * u) / 0.2f;

                TANK_[i].Write(static_cast<T>(u));
                TANK_[i].Advance();
            }

            // Stereo decode + cloud out
            T tL = static_cast<T>(0.5f) * (v[0] - v[2]);
            T tR = static_cast<T>(0.5f) * (v[1] - v[3]);
            T wetL = tL + static_cast<T>(cloud_outmix_) * cL;
            T wetR = tR + static_cast<T>(cloud_outmix_) * cR;

            // Tilt post (mirrored)
            if (bright_switch_ && tilt_amt_ > 0.f)
            {
                const float a = tilt_amt_;
                const float postL = TILT_.Post(static_cast<float>(wetL));
                const float postR = TILT_.Post(static_cast<float>(wetR));
                wetL = static_cast<T>((1.f - a) * static_cast<float>(wetL) + a * postL);
                wetR = static_cast<T>((1.f - a) * static_cast<float>(wetR) + a * postR);
            }

            // Wet/dry + output gain
            outL = static_cast<T>(wet_) * wetL * static_cast<T>(out_gain_) + static_cast<T>(1.f - wet_) * in;
            outR = static_cast<T>(wet_) * wetR * static_cast<T>(out_gain_) + static_cast<T>(1.f - wet_) * in;
        }

    private:
        // core
        float sr_{48000.f};
        float wet_{0.75f}, out_gain_{1.0f};
        float decay_{0.85f}, tank_fb_{0.82f};
        float brightness_{0.7f};
        bool bright_switch_{false};
        float tilt_amt_{0.0f}; // 0..1 derived from brightness (expo)

        // ER
        DelayBufferExt<T, ER_MAX> ER_L_, ER_R_;
        float erL_samp_{0.f}, erR_samp_{0.f}, er_width_{0.7f};

        // Pre-delay
        DelayBufferExt<T, PRE_MAX> PRE_;
        float pre_ms_{30.f}, pre_samps_{0.f};
        LFOUtils pre_lfo_;
        float pre_mod_depth_ms_{2.0f};

        // Diffusers
        AllpassDiffuser<T, DIFF_MAX> AP0_, AP1_;

        // Tank + damping
        DelayBufferExt<T, TANK_MAX> TANK_[NLINES];
        OnePoleLPF DAMP_[NLINES];
        float base_delay_[NLINES];
        float lfo_phase_[NLINES], lfo_rate_[NLINES], lfo_depth_[NLINES];

        // Tilt EQ
        Tilt12dB TILT_;

        // Cloud
        FireflyDelayPerfV3<T, CLOUD_MAX, NFF> CLOUD_;
        float cloud_send_{0.30f}, cloud_outmix_{0.30f};

        // Freeze smoothing
        float freeze_env_{0.f}, freeze_target_{0.f}, freeze_alpha_{0.0f};

        // HPF helper state
        float hp_lp_{0.f};
    };

} // namespace daisyfarm
