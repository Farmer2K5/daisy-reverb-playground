#pragma once
#include <cmath>
#include <cstdint>

namespace daisyfarm
{

    template <size_t N>
    class MixMatrix
    {
    public:
        enum Mode
        {
            HOUSEHOLDER,
            HADAMARD,
            RANDOM_ORTHO
        };

        void Init(float sample_rate)
        {
            sample_rate_ = sample_rate;
            SetMode(HOUSEHOLDER);
            rng_ = 0x12345678u;
            ro_phase_ = 0.0f;
        }

        void SetMode(Mode m)
        {
            mode_ = m;
            if (mode_ == HOUSEHOLDER)
                SetHouseholder();
            else if (mode_ == HADAMARD)
                SetHadamard(); // Only valid for N = 4 or 8
            else
                SetIdentity(); // RANDOM_ORTHO evolves during Apply()
        }

        void SetROParams(float rate_hz, float depth)
        {
            ro_rate_hz_ = rate_hz;
            ro_depth_ = depth;
        }

        inline void Apply(float y[N])
        {
            if (mode_ == RANDOM_ORTHO)
                StepRandomOrtho();

            float tmp[N];
            for (size_t r = 0; r < N; r++)
            {
                float s = 0.f;
                for (size_t c = 0; c < N; c++)
                    s += M_[r][c] * y[c];
                tmp[r] = s;
            }
            for (size_t i = 0; i < N; i++)
                y[i] = tmp[i];
        }

    private:
        float M_[N][N];
        Mode mode_;
        float sample_rate_;
        float ro_rate_hz_ = 0.05f;
        float ro_depth_ = 0.12f;
        float ro_phase_ = 0.0f;
        uint32_t rng_ = 1;

        // ------- Matrix Builders ------- //

        void SetIdentity()
        {
            for (size_t r = 0; r < N; r++)
                for (size_t c = 0; c < N; c++)
                    M_[r][c] = (r == c ? 1.f : 0.f);
        }

        void SetHouseholder()
        {
            const float a = -2.0f / float(N);
            for (size_t r = 0; r < N; r++)
                for (size_t c = 0; c < N; c++)
                    M_[r][c] = (r == c ? 1.f : 0.f) + a;
        }

        // Only valid for N = 4 or N = 8
        void SetHadamard()
        {
            SetIdentity(); // fallback default

            if constexpr (N == 4)
            {
                const float s = 1.f / 2.f;
                static const float H[4][4] =
                    {
                        {1, 1, 1, 1},
                        {1, -1, 1, -1},
                        {1, 1, -1, -1},
                        {1, -1, -1, 1}};
                for (int r = 0; r < 4; r++)
                    for (int c = 0; c < 4; c++)
                        M_[r][c] = s * H[r][c];
            }
            else if constexpr (N == 8)
            {
                const float s = 1.f / std::sqrt(8.f);
                static const float H[8][8] =
                    {
                        {1, 1, 1, 1, 1, 1, 1, 1},
                        {1, -1, 1, -1, 1, -1, 1, -1},
                        {1, 1, -1, -1, 1, 1, -1, -1},
                        {1, -1, -1, 1, 1, -1, -1, 1},
                        {1, 1, 1, 1, -1, -1, -1, -1},
                        {1, -1, 1, -1, -1, 1, -1, 1},
                        {1, 1, -1, -1, -1, -1, 1, 1},
                        {1, -1, -1, 1, -1, 1, 1, -1}};
                for (int r = 0; r < 8; r++)
                    for (int c = 0; c < 8; c++)
                        M_[r][c] = s * H[r][c];
            }
        }

        // ------- Time-Varying Spatial Drift ------- //

        void StepRandomOrtho()
        {
            ro_phase_ += 2.0f * 3.141592653f * (ro_rate_hz_ / sample_rate_);
            if (ro_phase_ > 2.0f * 3.141592653f)
                ro_phase_ -= 2.0f * 3.141592653f;
            float theta = ro_depth_ * std::sin(ro_phase_);
            float c = std::cos(theta);
            float s = std::sin(theta);

            // Select two rows to rotate
            static size_t i = 0;
            if (RandSigned() > 0.98f)
                i = (i + 1) % N;
            size_t j = (i + (N / 2)) % N;

            for (size_t col = 0; col < N; col++)
            {
                float a = M_[i][col];
                float b = M_[j][col];
                M_[i][col] = c * a - s * b;
                M_[j][col] = s * a + c * b;
            }
        }

        inline float RandSigned()
        {
            rng_ ^= rng_ << 13;
            rng_ ^= rng_ >> 17;
            rng_ ^= rng_ << 5;
            return (float(int(rng_ & 0xFFFF)) / 32768.f) - 1.f;
        }
    };

} // namespace daisyfarm
