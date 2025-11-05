#pragma once
#include <cmath>

namespace daisyfarm
{

    // Scattering Chamber Tank (SR-L)
    // 4-line sparse scattering lattice, large chamber behavior.
    template <typename T, size_t max_size>
    class ScatteringRoomTank
    {
    public:
        void Init(float sr)
        {
            sr_ = sr;
            for (int i = 0; i < 4; i++)
            {
                idx_[i] = 0;
                for (size_t s = 0; s < max_size; s++)
                    buf_[i][s] = 0;
            }
            fb_ = 0.82f;
            damp_ = 0.95f;
            // Line lengths (large chamber)
            len_[0] = 3101;
            len_[1] = 4421;
            len_[2] = 5039;
            len_[3] = 6229;
        }
        void SetFeedback(float fb) { fb_ = fb; }
        void SetDamping(float d) { damp_ = d; }
        void SetLine(size_t i, size_t len)
        {
            if (i < 4)
                len_[i] = len;
        }

        void Process(T x, T &L, T &R)
        {
            // Read lines
            T v[4];
            for (int i = 0; i < 4; i++)
            {
                size_t r = (idx_[i] + max_size - len_[i]) % max_size;
                v[i] = buf_[i][r];
            }
            // Scatter
            T s0 = (v[1] - v[2]) * fb_ + x;
            T s1 = (v[0] - v[3]) * fb_ + x;
            T s2 = (-v[0] + v[3]) * fb_;
            T s3 = (-v[1] + v[2]) * fb_;
            // Damping
            s0 *= damp_;
            s1 *= damp_;
            s2 *= damp_;
            s3 *= damp_;
            // Write / advance
            buf_[0][idx_[0]] = s0;
            buf_[1][idx_[1]] = s1;
            buf_[2][idx_[2]] = s2;
            buf_[3][idx_[3]] = s3;
            for (int i = 0; i < 4; i++)
                idx_[i] = (idx_[i] + 1) % max_size;
            // Stereo out
            L = (v[0] + v[2]) * 0.5f;
            R = (v[1] + v[3]) * 0.5f;
        }

    private:
        float sr_;
        float fb_;
        float damp_;
        T buf_[4][max_size];
        size_t idx_[4];
        size_t len_[4];
    };

} // namespace daisyfarm