#pragma once
#include <cmath>

namespace daisyfarm
{

    /**
     * @brief Dual first-order shelving filter (low + high shelves).
     *
     * Intended for reverb tanks, diffusion returns, or wet bus tone shaping.
     *
     * Parameters:
     *  - low_gain_  : linear gain to apply below cutoff (suggest range 0.5 .. 2.0)
     *  - high_gain_ : linear gain to apply above cutoff (suggest range 0.5 .. 2.0)
     *  - cutoff_    : pivot frequency in Hz (e.g., 600–6000 depending on space size)
     *
     * State:
     *  - y_lp_ : low-shelf integrator
     *  - y_hp_ : high-shelf integrator
     *
     * All stable for 0 < a < 1, where a = exp(-2*pi*fc/fs).
     */
    class ShelfPair
    {
    public:
        ShelfPair()
            : sample_rate_(48000.0f), cutoff_(1200.0f), low_gain_(1.0f), high_gain_(1.0f)
        {
            y_lp_ = y_hp_ = 0.0f;
            Recompute();
        }

        void Init(float sample_rate)
        {
            sample_rate_ = sample_rate;
            Recompute();
            y_lp_ = y_hp_ = 0.0f;
        }

        void SetCutoff(float hz)
        {
            cutoff_ = (hz < 20.0f ? 20.0f : hz);
            Recompute();
        }

        // Gain in *linear* units (use exp(dB * ln(10)/20) to convert from dB)
        void SetLowGain(float g) { low_gain_ = g; }
        void SetHighGain(float g) { high_gain_ = g; }

        inline float Process(float x)
        {
            // Split signal into low and high via first-order smoothing
            // LP branch
            y_lp_ = lp_a_ * y_lp_ + (1.0f - lp_a_) * x;
            // HP branch
            float hp = x - y_lp_;

            // Apply shelving gains
            return (low_gain_ * y_lp_) + (high_gain_ * hp);
        }

    private:
        void Recompute()
        {
            const float a = std::exp(-2.0f * 3.14159265358979f * cutoff_ / sample_rate_);
            lp_a_ = a; // core coefficient
        }

        float sample_rate_;
        float cutoff_;
        float low_gain_, high_gain_;
        float lp_a_;

        float y_lp_; // low-shelf integrator state
        float y_hp_; // unused but kept for symmetry if extended later
    };

} // namespace daisyfarm
