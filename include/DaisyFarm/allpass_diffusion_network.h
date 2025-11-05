/**
 * @file diffusion_stage.h
 * @brief Allpass diffusion stages built on DelayBufferExt.
 */
#pragma once
#include "allpass_diffuser.h"
#include <cstddef>

namespace daisyfarm
{

    /** Stack of N allpass diffusers in series. */
    template <typename T, size_t max_size, size_t N>
    class AllpassDiffusionNetwork
    {
    public:
        void Init(T *external_buffers[N])
        {
            for (size_t i = 0; i < N; i++)
                ap_[i].Init(external_buffers[i]);
        }

        void Reset()
        {
            for (size_t i = 0; i < N; i++)
                ap_[i].Reset();
        }

        void SetStage(size_t i, float delay_samps, float g)
        {
            if (i < N)
                ap_[i].Set(delay_samps, g);
        }

        inline T Process(T x)
        {
            for (size_t i = 0; i < N; i++)
                x = ap_[i].Process(x);
            return x;
        }

    private:
        AllpassDiffuser<T, max_size> ap_[N];
    };

} // namespace daisyfarm
