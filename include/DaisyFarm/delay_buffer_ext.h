/**
 * @file delaybuffer_ext.h
 * @brief External-buffer circular delay buffer optimized for SDRAM/SRAM.
 *
 * Provides precise control over read/write order for feedback and FDN systems.
 * Derived from DaisySP-style DelayLine design, refactored for clarity and SDRAM use.
 */
#pragma once
#include <cstddef>
#include <cstdint>

namespace daisyfarm
{

    template <typename T, size_t max_size>
    class DelayBufferExt
    {
    public:
        DelayBufferExt() : buffer_(nullptr), write_ptr_(0) {}

        void Init(T *external_buffer)
        {
            buffer_ = external_buffer;
            Reset();
        }

        void Reset()
        {
            if (buffer_)
            {
                for (size_t i = 0; i < max_size; i++)
                    buffer_[i] = static_cast<T>(0);
            }
            write_ptr_ = 0;
        }

        /** @brief Write a new sample (no advance). */
        inline void Write(const T sample) { buffer_[write_ptr_] = sample; }

        /** @brief Advance write pointer manually. */
        inline void Advance()
        {
            if (++write_ptr_ >= max_size)
                write_ptr_ = 0;
        }

        /** @brief Combined write and advance (for feed-forward use). */
        inline void WriteAndAdvance(const T sample)
        {
            Write(sample);
            Advance();
        }

        /** @brief Fractional-sample read using linear interpolation. */
        inline T Read(float delay) const
        {
            int32_t delay_i = static_cast<int32_t>(delay);
            float frac = delay - static_cast<float>(delay_i);

            size_t idxA = Wrap(write_ptr_ + max_size - delay_i);
            size_t idxB = Wrap(idxA + 1);

            const T a = buffer_[idxA];
            const T b = buffer_[idxB];
            return a + (b - a) * frac;
        }

        /** @brief High-quality Hermite-interpolated read. */
        inline T ReadHermite(float delay) const
        {
            int32_t delay_i = static_cast<int32_t>(delay);
            float frac = delay - static_cast<float>(delay_i);
            size_t base = Wrap(write_ptr_ + max_size - delay_i);

            const T xm1 = buffer_[Wrap(base - 1)];
            const T x0 = buffer_[Wrap(base)];
            const T x1 = buffer_[Wrap(base + 1)];
            const T x2 = buffer_[Wrap(base + 2)];

            const float c = (x1 - xm1) * 0.5f;
            const float v = x0 - x1;
            const float w = c + v;
            const float a = w + v + (x2 - x0) * 0.5f;
            const float b_neg = w + a;
            const float f = frac;
            return (((a * f) - b_neg) * f + c) * f + x0;
        }

    private:
        inline size_t Wrap(size_t i) const { return (i >= max_size) ? i - max_size : i; }

        T *buffer_;
        size_t write_ptr_;
    };

} // namespace daisyfarm
