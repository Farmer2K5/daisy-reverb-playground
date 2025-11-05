#pragma once
#include "delay_buffer_ext.h"
#include <algorithm>
#include <cmath>

namespace daisyfarm
{
    // =============================================================
    // ShimmerPitchShift (Dual-Tap, Doppler Crossfaded Pitch Shifter)
    //
    // Features:
    //  - Stable dual-cycle modulating read pointers
    //  - Smooth Hann crossfade
    //  - Safe delay bounds + sweep constraints
    //  - Cents/semitones control
    // =============================================================

    template <size_t max_size>
    class ShimmerPitchShift
    {
    public:
        // ---------------------------------------------------------
        // Init:
        //  external   = buffer in SDRAM or heap
        //  sr         = sample rate
        // ---------------------------------------------------------
        void Init(float *external, float sr)
        {
            d_.Init(external);
            sr_ = sr;
            SetParams(+12.0f, 0.06f, 30.0f, 20.0f); // defaults
            phase_ = 0.0f;
        }

        // ---------------------------------------------------------
        // SetParams:
        //  semi      = pitch in semitones (+12 = octave up)
        //  rate_hz   = sweep LFO rate
        //  sweep_ms  = total sweep excursion (ms)
        //  min_ms    = lowest delay base (ms)
        // ---------------------------------------------------------
        void SetParams(float semi, float rate_hz, float sweep_ms, float min_ms)
        {
            semi_ = semi;
            rate_ = rate_hz;
            sweep_ = sweep_ms * 0.001f * sr_;
            min_ = std::max(1.0f, min_ms * 0.001f * sr_);

            // Pitch scaling (read pointer sweep depth gives pitch shift)
            ratio_ = std::pow(2.0f, semi_ / 12.0f);
        }

        // ---------------------------------------------------------
        // Process one sample
        // ---------------------------------------------------------
        inline float Process(float x)
        {
            d_.Write(x);
            d_.Advance();

            // Advance phase & wrap
            phase_ += (rate_ / sr_);
            while (phase_ >= 1.0f)
                phase_ -= 1.0f;

            float p1 = phase_;
            float p2 = phase_ + 0.5f;
            if (p2 >= 1.0f)
                p2 -= 1.0f;

            // Sweep clamping for max_size safety
            float sweep = sweep_;
            if (min_ + sweep + 4 >= max_size)
                sweep = std::max(1.0f, max_size - min_ - 4);

            // Compute moving read positions
            float d1 = min_ + sweep * (1.0f - p1);
            float d2 = min_ + sweep * (1.0f - p2);

            // Hann blend window
            auto hann = [](float t)
            {
                return 0.5f - 0.5f * std::cos(2.0f * float(M_PI) * t);
            };

            float w1 = hann(p1);
            float w2 = hann(p2);

            // Read delayed samples
            float y1 = d_.Read(d1);
            float y2 = d_.Read(d2);

            // Normalize safely
            float wsum = w1 + w2 + 1e-6f;
            return (w1 * y1 + w2 * y2) / wsum;
        }

    private:
        DelayBufferExt<float, max_size> d_;

        float sr_ = 48000.0f;
        float semi_ = 12.0f;
        float ratio_ = 2.0f;
        float rate_ = 0.1f;
        float sweep_ = 100.0f;
        float min_ = 20.0f;
        float phase_ = 0.0f;
    };
}
