/**
 * @file allpass_diffuser.h
 * @brief Allpass diffusion stages built on DelayBufferExt.
 */
#pragma once
#include "delay_buffer_ext.h"
#include <cstddef>

namespace daisyfarm
{

    /** Single Schroeder allpass diffuser. */
    template <typename T, size_t max_size>
    class AllpassDiffuser
    {
    public:
        AllpassDiffuser() : delay_samps_(100.0f), g_(0.7f) {}

        void Init(T *external_buffer) { d_.Init(external_buffer); }

        void Reset() { d_.Reset(); }

        void Set(float delay_samps, float g)
        {
            delay_samps_ = delay_samps;
            g_ = g;
        }

        inline T Process(T x)
        {
            const T y = d_.Read(delay_samps_);
            const T v = x + g_ * y;
            d_.Write(v);
            d_.Advance();
            return -g_ * v + y;
        }

    private:
        DelayBufferExt<T, max_size> d_;
        float delay_samps_;
        float g_;
    };

} // namespace daisyfarm
