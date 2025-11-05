/**
 * @file early_reflections.h
 * @brief Early reflection generator using a single DelayBufferExt with multi-taps.
 */
#pragma once
#include "delay_buffer_ext.h"
#include <cstddef>

namespace daisyfarm
{

    /**
     * @tparam T Sample type (usually float)
     * @tparam max_size Buffer length in samples
     * @tparam N_TAPS Number of early reflection taps
     */
    template <typename T, size_t max_size, size_t N_TAPS>
    class EarlyReflections
    {
    public:
        EarlyReflections() : sample_rate_(48000.0f), num_taps_(0) {}

        void Init(T *external_buffer, float sample_rate)
        {
            delay_.Init(external_buffer);
            sample_rate_ = sample_rate;
            num_taps_ = 0;
        }

        void Reset()
        {
            delay_.Reset();
            num_taps_ = 0;
        }

        /** @brief Set one tap.
         *  @param i      Tap index
         *  @param ms     Delay in milliseconds
         *  @param gain   Linear gain
         *  @param pan    -1 (L) .. +1 (R)
         */
        void SetTap(size_t i, float ms, float gain, float pan)
        {
            if (i >= N_TAPS)
                return;
            taps_[i].delay_samps = ms * 0.001f * sample_rate_;
            taps_[i].gain = gain;
            taps_[i].pan = (pan < -1.0f) ? -1.0f : (pan > 1.0f ? 1.0f : pan);
            if (i >= num_taps_)
                num_taps_ = i + 1;
        }

        /** @brief Process one input sample to stereo ER sum. */
        void Process(T in, T &outL, T &outR)
        {
            T l = 0, r = 0;
            // Read all taps from history first
            for (size_t i = 0; i < num_taps_; i++)
            {
                const T y = delay_.Read(taps_[i].delay_samps);
                const float gl = 0.5f * taps_[i].gain * (1.0f - taps_[i].pan);
                const float gr = 0.5f * taps_[i].gain * (1.0f + taps_[i].pan);
                l += gl * y;
                r += gr * y;
            }
            // Write current input and advance
            delay_.Write(in);
            delay_.Advance();
            outL = l;
            outR = r;
        }

    private:
        struct Tap
        {
            float delay_samps, gain, pan;
        };

        DelayBufferExt<T, max_size> delay_;
        Tap taps_[N_TAPS];
        float sample_rate_;
        size_t num_taps_;
    };

} // namespace daisyfarm
