#pragma once
#include <cmath>

namespace daisyfarm
{

    // Velvet Hall Tank (Cinematic VH-A)
    // Mono-in, stereo-out. Requires: TriBand damping if desired.
    template <typename T, size_t max_size>
    class VelvetHallTank
    {
    public:
        void Init(float sr)
        {
            sr_ = sr;
            for (size_t i = 0; i < max_size; i++)
            {
                delay_.buf[i] = 0;
            }
            idx_ = 0;
            fb_ = 0.88f;
            tapSpacing_ = 173;
            damp_ = 0.92f;
        }
        void SetFeedback(float fb) { fb_ = fb; }
        void SetDamping(float d) { damp_ = d; }
        void SetTapSpacing(size_t s) { tapSpacing_ = s; }

        void Process(T x, T &L, T &R)
        {
            delay_.buf[idx_] = x + fb_ * lastMix_;
            idx_ = (idx_ + 1) % max_size;
            // Velvet cluster: 6 staggered taps
            T s = 0;
            for (int k = 0; k < 6; k++)
            {
                size_t t = (idx_ + k * tapSpacing_) % max_size;
                s += delay_.buf[t];
            }
            s /= 6.f;
            s = damp_ * s;
            lastMix_ = s;
            // Stereo decorrelation
            L = s;
            R = -s;
        }

    private:
        struct
        {
            T buf[max_size];
        } delay_;
        size_t idx_ = 0;
        float sr_ = 48000;
        float fb_ = 0.88f;
        size_t tapSpacing_ = 173;
        T lastMix_ = 0;
        float damp_ = 0.92f;
    };

} // namespace daisyfarm
