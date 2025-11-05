#pragma once
#include "delay_buffer_ext.h" // fractional reads, hermite also available
#include <cmath>
#include <cstdint>

namespace daisyfarm
{

    /**
     * @brief Schroeder allpass diffuser with time-varying delay:
     *        y = d.Read( delay_base + mod(t) );
     *        v = x + g*y;  d.Write(v); d.Advance();  out = -g*v + y;
     *
     * Mod sources:
     *  - Sine LFO (rate_hz, depth_samps)
     *  - Slow wander/jitter (random walk), prevents stationary chorus beating
     *
     * Safe for feedback loops when depth is small (a few samples).
     *
     * T:       sample type (float)
     * max_size delay line storage length (samples)
     */
    template <typename T, size_t max_size>
    class ModulatedDiffuser
    {
    public:
        ModulatedDiffuser()
            : sample_rate_(48000.0f),
              delay_base_(120.0f),
              g_(0.7f),
              // sine LFO
              lfo_rate_hz_(0.10f),
              lfo_depth_samps_(1.5f),
              lfo_phase_(0.0f),
              // wander
              wander_amt_samps_(0.3f),
              wander_tc_hz_(0.25f),
              wander_state_(0.0f),
              rng_(0x12345678u)
        {
        }

        void Init(T *external_buffer, float sample_rate)
        {
            d_.Init(external_buffer);
            sample_rate_ = sample_rate;
            Reset();
        }

        void Reset()
        {
            d_.Reset();
            lfo_phase_ = 0.0f;
            wander_state_ = 0.0f;
        }

        // --- parameters ---
        void SetBase(float delay_samps, float g)
        {
            delay_base_ = delay_samps;
            g_ = g;
        }

        void SetLFO(float rate_hz, float depth_samps)
        {
            lfo_rate_hz_ = (rate_hz < 0.f ? 0.f : rate_hz);
            lfo_depth_samps_ = (depth_samps < 0.f ? 0.f : depth_samps);
        }

        /** Wander is a slow random walk low-passed to ~wander_tc_hz_ */
        void SetWander(float amount_samps, float time_constant_hz = 0.25f)
        {
            wander_amt_samps_ = (amount_samps < 0.f ? 0.f : amount_samps);
            wander_tc_hz_ = (time_constant_hz <= 0.f ? 0.25f : time_constant_hz);
        }

        inline T Process(T x)
        {
            // --- build modulation ---
            // 1) sine LFO
            float lfo = std::sin(lfo_phase_) * lfo_depth_samps_;
            lfo_phase_ += 2.0f * 3.14159265358979f * (lfo_rate_hz_ / sample_rate_);
            if (lfo_phase_ > 2.0f * 3.14159265358979f)
                lfo_phase_ -= 2.0f * 3.14159265358979f;

            // 2) slow wander (first-order noise lowpass)
            // white ~ [-1,1]
            float wn = 2.0f * (RandFloat01() - 0.5f);
            float a = std::exp(-2.0f * 3.14159265358979f * wander_tc_hz_ / sample_rate_);
            wander_state_ = a * wander_state_ + (1.0f - a) * wn * wander_amt_samps_;

            float delay_samps = delay_base_ + lfo + wander_state_;

            // --- allpass core (linear read; swap to ReadHermite for smoother mod) ---
            const T y = d_.Read(delay_samps);
            const T v = x + static_cast<T>(g_) * y;
            d_.Write(v);
            d_.Advance();
            return static_cast<T>(-g_) * v + y;
        }

        // Optional: Hermite-variant for smoother wide/deeper modulation
        inline T ProcessHermite(T x)
        {
            const T y = d_.ReadHermite(delay_base_ + std::sin(lfo_phase_) * lfo_depth_samps_ + wander_state_);
            const T v = x + static_cast<T>(g_) * y;
            d_.Write(v);
            d_.Advance();
            lfo_phase_ += 2.0f * 3.14159265358979f * (lfo_rate_hz_ / sample_rate_);
            if (lfo_phase_ > 2.0f * 3.14159265358979f)
                lfo_phase_ -= 2.0f * 3.14159265358979f;
            // wander update (same as Process)
            float wn = 2.0f * (RandFloat01() - 0.5f);
            float a = std::exp(-2.0f * 3.14159265358979f * wander_tc_hz_ / sample_rate_);
            wander_state_ = a * wander_state_ + (1.0f - a) * wn * wander_amt_samps_;
            return static_cast<T>(-g_) * v + y;
        }

    private:
        // Very small xorshift PRNG, returns [0,1)
        inline float RandFloat01()
        {
            uint32_t x = rng_;
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            rng_ = x;
            return (x & 0x00FFFFFFu) * (1.0f / 16777216.0f);
        }

        DelayBufferExt<T, max_size> d_;

        float sample_rate_;
        float delay_base_;
        float g_;

        // sine LFO
        float lfo_rate_hz_;
        float lfo_depth_samps_;
        float lfo_phase_;

        // wander
        float wander_amt_samps_;
        float wander_tc_hz_;
        float wander_state_;
        uint32_t rng_;
    };

} // namespace daisyfarm
