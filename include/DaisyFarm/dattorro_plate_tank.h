#pragma once
#include <cmath>
#include <cstdint>

namespace daisyfarm
{

    // -----------------------------------------------------------------------------
    // Dattorro Plate Tank (self-contained, static/stable; no time-varying parts)
    // -----------------------------------------------------------------------------
    // Design goals:
    // - Smooth, lush plate without smear/wind (no time-varying modulation inside)
    // - Drop-in alternative "tank" for a modular reverb pipeline
    // - Stereo output from mono (or stereo-summed) input
    // - Controls: Feedback, Input/Tank Diffusion, Damping, Predelay
    //
    // Notes:
    // - All delays are implemented on a single circular buffer per line (templated max_size).
    // - Choose max_size large enough for your longest line; 12000–20000 @48k is typical.
    // - No RANDOM_ORTHO here; this topology is inherently stable/clear.
    //
    // API (common subset):
    //   template<typename T, size_t max_size>
    //   class DattorroPlateTank {
    //     void Init(float sr);
    //     void SetFeedback(float fb);            // 0..0.95 typical
    //     void SetInputDiffusion(float a1, float a2); // 0..0.7 (allpass coeffs)
    //     void SetTankDiffusion(float a1, float a2);  // 0..0.7
    //     void SetDamping(float lo, float mid, float hi); // simple tri-band scaler
    //     void SetPreDelayMs(float ms);
    //     void Process(T x, T& outL, T& outR);
    //   };
    //
    // -----------------------------------------------------------------------------

    // Simple circular delay with linear interpolation
    template <typename T, size_t max_size>
    class SimpleDelay
    {
    public:
        void Init()
        {
            write_ = 0;
            for (size_t i = 0; i < max_size; i++)
                buf_[i] = T(0);
        }
        inline void Write(T s) { buf_[write_] = s; }
        inline void Advance()
        {
            if (++write_ >= max_size)
                write_ = 0;
        }
        inline T Read(float delay_samps) const
        {
            // Clamp (assumes delay < max_size-2)
            if (delay_samps < 1.f)
                delay_samps = 1.f;
            if (delay_samps > float(max_size - 2))
                delay_samps = float(max_size - 2);
            int di = int(delay_samps);
            float frac = delay_samps - float(di);
            size_t idxA = Wrap(write_ + max_size - di);
            size_t idxB = Wrap(idxA + 1);
            const T a = buf_[idxA];
            const T b = buf_[idxB];
            return a + (b - a) * frac;
        }

    private:
        inline size_t Wrap(size_t i) const { return (i >= max_size ? i - max_size : i); }
        T buf_[max_size];
        size_t write_;
    };

    // First-order allpass: y = -a*x + z; z = x + a*y
    struct AP1
    {
        float a{0.5f};
        float z{0.f};
        inline float Process(float x)
        {
            float y = -a * x + z;
            z = x + a * y;
            return y;
        }
    };

    // One-pole lowpass utility
    struct OnePoleLP
    {
        float a{0.5f};
        float y{0.f};
        inline float Process(float x)
        {
            y = a * y + (1.f - a) * x;
            return y;
        }
        inline void SetFc(float fc_norm)
        { // fc_norm = fc / sr
            // crude mapping to smoothing coefficient
            float x = std::exp(-2.f * 3.14159265f * fc_norm);
            a = x;
        }
    };

    // Tri-band damping (very lightweight): split into low/mid/high by two LPs
    struct TriBand
    {
        OnePoleLP lpL, lpH;
        float gL{1.f}, gM{1.f}, gH{1.f};
        float sr{48000.f};
        void Init(float sr_)
        {
            sr = sr_;
            lpL.SetFc(200.f / sr);
            lpH.SetFc(4000.f / sr);
        }
        void SetDecay(float dL, float dM, float dH)
        {
            gL = dL;
            gM = dM;
            gH = dH;
        }
        inline float Process(float x)
        {
            float l = lpL.Process(x);
            float hpre = x - l;
            float m = lpH.Process(hpre);
            float h = hpre - m;
            return gL * l + gM * m + gH * h;
        }
    };

    // -----------------------------------------------------------------------------
    // Dattorro Plate Tank
    // -----------------------------------------------------------------------------
    template <typename T, size_t max_size>
    class DattorroPlateTank
    {
    public:
        void Init(float sr)
        {
            sr_ = sr;
            // Clear delays
            pd_.Init();
            dA_.Init();
            dB_.Init();
            dC_.Init();
            dD_.Init();
            tapL_.Init();
            tapR_.Init();
            // Default coefficients (musical, safe)
            fb_ = 0.86f;
            in1_.a = 0.65f;
            in2_.a = 0.60f;
            t1L_.a = 0.58f;
            t2L_.a = 0.60f;
            t1R_.a = 0.58f;
            t2R_.a = 0.60f;
            // Damping
            damp_.Init(sr_);
            damp_.SetDecay(1.04f, 0.92f, 0.78f);
            // Predelay default
            SetPreDelayMs(25.f);
            // Default structural lengths (48k-friendly, prime-ish)
            // Core loop delays:
            dA_len_ = 1687.f; // L path 1
            dB_len_ = 1601.f; // R path 1
            dC_len_ = 2053.f; // L path 2
            dD_len_ = 2251.f; // R path 2
            // Output taps (post-tank)
            tL_len_ = 353.f;
            tR_len_ = 529.f;
        }

        // --- Controls
        void SetFeedback(float fb)
        {
            if (fb < 0.f)
                fb = 0.f;
            if (fb > 0.98f)
                fb = 0.98f;
            fb_ = fb;
        }
        void SetInputDiffusion(float a1, float a2)
        {
            in1_.a = ClampAllpass(a1);
            in2_.a = ClampAllpass(a2);
        }
        void SetTankDiffusion(float a1, float a2)
        {
            float c1 = ClampAllpass(a1), c2 = ClampAllpass(a2);
            t1L_.a = c1;
            t2L_.a = c2;
            t1R_.a = c1;
            t2R_.a = c2;
        }
        void SetDamping(float lo, float mid, float hi) { damp_.SetDecay(lo, mid, hi); }
        void SetPreDelayMs(float ms)
        {
            float s = (ms * 0.001f) * sr_;
            if (s < 1.f)
                s = 1.f;
            if (s > float(max_size - 2))
                s = float(max_size - 2);
            pd_len_ = s;
        }

        // Optional: adjust structural times if desired
        void SetStructure(float dA, float dB, float dC, float dD)
        {
            dA_len_ = dA;
            dB_len_ = dB;
            dC_len_ = dC;
            dD_len_ = dD;
        }

        // Process mono input → stereo output
        void Process(T x, T &outL, T &outR)
        {
            // --- Predelay
            pd_.Write(x);
            pd_.Advance();
            float xin = pd_.Read(pd_len_);

            // --- Input diffusion (two AP in series)
            float di = in1_.Process(xin);
            di = in2_.Process(di);

            // --- Tank structure (two intertwined loops)
            // Left path:
            float a = dA_.Read(dA_len_);
            float l1 = t1L_.Process(di + fb_ * a);
            dA_.Write(l1);
            dA_.Advance();

            float c = dC_.Read(dC_len_);
            float l2 = t2L_.Process(l1);
            l2 = damp_.Process(l2 + 0.0f * c); // simple feed; cross is implicit via stereo return
            dC_.Write(l2);
            dC_.Advance();

            // Right path:
            float b = dB_.Read(dB_len_);
            float r1 = t1R_.Process(di + fb_ * b);
            dB_.Write(r1);
            dB_.Advance();

            float d = dD_.Read(dD_len_);
            float r2 = t2R_.Process(r1);
            r2 = damp_.Process(r2 + 0.0f * d);
            dD_.Write(r2);
            dD_.Advance();

            // --- Output taps (classic plate uses several; we do 1+mix)
            float tapL = tapL_.Read(tL_len_);
            float tapR = tapR_.Read(tR_len_);

            // Feed taps from inner nodes to add richness
            tapL_.Write(l2 + 0.3f * a);
            tapL_.Advance();
            tapR_.Write(r2 + 0.3f * b);
            tapR_.Advance();

            // --- Stereo out mix (gentle cross for dimension)
            outL = 0.6f * l2 + 0.4f * tapL + 0.25f * r1;
            outR = 0.6f * r2 + 0.4f * tapR + 0.25f * l1;
        }

    private:
        inline float ClampAllpass(float a)
        {
            if (a < -0.7f)
                a = -0.7f;
            if (a > 0.7f)
                a = 0.7f;
            return a;
        }

        float sr_{48000.f};

        // Predelay
        SimpleDelay<T, max_size> pd_;
        float pd_len_{1.f};

        // Input diffusion (two AP)
        AP1 in1_, in2_;

        // Tank delays (two per side)
        SimpleDelay<T, max_size> dA_, dB_, dC_, dD_;
        float dA_len_{1687.f}, dB_len_{1601.f}, dC_len_{2053.f}, dD_len_{2251.f};

        // Tank diffusion allpasses (per side)
        AP1 t1L_, t2L_, t1R_, t2R_;

        // Output tap delays
        SimpleDelay<T, max_size> tapL_, tapR_;
        float tL_len_{353.f}, tR_len_{529.f};

        // Damping
        TriBand damp_;

        // Global feedback
        float fb_{0.86f};
    };

} // namespace daisyfarm