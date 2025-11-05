#pragma once
#include <cmath>

namespace daisyfarm
{

    /**
     * @brief Tri-band decay shaping filter for reverb feedback paths.
     *
     * Splits signal into:
     *   low  (< fc_low)
     *   mid  (fc_low .. fc_high)
     *   high (> fc_high)
     *
     * Applies per-band decay multipliers (0..1) to control how fast
     * energy dies out in each band.
     *
     * Designed to be used inside FDN loops, diffusion returns, or wet bus.
     * Fully stable (first-order crossovers).
     */
    class TriBandDamping
    {
    public:
        TriBandDamping()
            : sample_rate_(48000.0f),
              fc_low_(300.0f),
              fc_high_(6000.0f),
              d_low_(1.0f),
              d_mid_(1.0f),
              d_high_(1.0f)
        {
            lp1_ = lp2_ = 0.0f;
            Recompute();
        }

        void Init(float sample_rate)
        {
            sample_rate_ = sample_rate;
            lp1_ = lp2_ = 0.0f;
            Recompute();
        }

        void SetCrossover(float fc_low, float fc_high)
        {
            fc_low_ = (fc_low < 20 ? 20 : fc_low);
            fc_high_ = (fc_high < fc_low_ + 50 ? fc_low_ + 50 : fc_high); // ensure spacing
            Recompute();
        }

        // decay multipliers, 0..1 (recommend 0.5..1.0 for natural results)
        void SetDecay(float low, float mid, float high)
        {
            d_low_ = low;
            d_mid_ = mid;
            d_high_ = high;
        }

        /** Process mono sample */
        inline float Process(float x)
        {
            // 1) Low split
            lp1_ = a_low_ * lp1_ + (1.0f - a_low_) * x;
            float low = lp1_;

            // 2) High split from first stage
            float high_candidate = x - lp1_;

            // 3) Second LP to isolate mid
            lp2_ = a_high_ * lp2_ + (1.0f - a_high_) * high_candidate;
            float mid = lp2_;

            // 4) Final high
            float high = high_candidate - lp2_;

            // 5) Apply decay shaping
            low *= d_low_;
            mid *= d_mid_;
            high *= d_high_;

            // 6) Recombine
            return low + mid + high;
        }

        /** Process N channels (e.g., 8 FDN lines) */
        template <size_t N>
        inline void ProcessArray(float x[N])
        {
            for (size_t i = 0; i < N; i++)
                x[i] = Process(x[i]);
        }

    private:
        void Recompute()
        {
            a_low_ = std::exp(-2.0f * 3.14159265358979f * fc_low_ / sample_rate_);
            a_high_ = std::exp(-2.0f * 3.14159265358979f * fc_high_ / sample_rate_);
        }

        float sample_rate_;
        float fc_low_, fc_high_;
        float d_low_, d_mid_, d_high_;

        float a_low_, a_high_;
        float lp1_, lp2_;
    };

} // namespace daisyfarm
