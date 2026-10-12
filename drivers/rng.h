#pragma once

#include <atomic>
#include <cstdint>
#include <optional>
#include "stm32u385xx.h"

namespace stm32u3::rng
{
    class peripheral
    {
    public:

        static inline void init() noexcept
        {
            RCC->AHB2ENR1 |= RCC_AHB2ENR1_RNGEN;
            __DSB();
            (void)RCC->AHB2ENR1;
            __DSB();
            __ISB();

            RNG->CR &= ~RNG_CR_RNGEN;
            RNG->CR |= RNG_CR_CONDRST;
            RNG->CR &= ~RNG_CR_CONDRST;

            while (RNG->CR & RNG_CR_CONDRST) { /* wait */ }

            RNG->CR |= RNG_CR_RNGEN;

            ready_.store(false, std::memory_order_relaxed);

            NVIC_ClearPendingIRQ(RNG_IRQn);
            NVIC_EnableIRQ(RNG_IRQn);
        }

        static inline void start() noexcept
        {
            ready_.store(false, std::memory_order_relaxed);
            RNG->CR |= RNG_CR_IE;
        }

        [[nodiscard]]
        static inline bool available() noexcept
        {
            return ready_.load(std::memory_order_acquire);
        }

        // Atomically checks-and-consumes the pending value, closing the
        // available()-then-get() race the old two-call API had.
        [[nodiscard]]
        static inline std::optional<uint32_t> try_get() noexcept
        {
            if (!ready_.load(std::memory_order_acquire))
                return std::nullopt;

            const uint32_t v = value_.load(std::memory_order_relaxed);
            ready_.store(false, std::memory_order_relaxed);
            return v;
        }

        static inline void irq_handler() noexcept
        {
            const uint32_t sr = RNG->SR;

            if (sr & RNG_SR_SEIS)
            {
                recover();
                return;
            }

            if (sr & RNG_SR_CEIS)
            {
                RNG->SR &= ~(RNG_SR_CEIS | RNG_SR_SEIS);
                return;
            }

            if (sr & RNG_SR_DRDY)
            {
                RNG->CR &= ~RNG_CR_IE;
                // value_ must be visible before ready_ is observed true:
                // relaxed store here, paired with the release below,
                // relaxed load in try_get() gated by the acquire above.
                value_.store(RNG->DR, std::memory_order_relaxed);
                ready_.store(true, std::memory_order_release);
            }
        }

    private:

        static inline void recover() noexcept
        {
            RNG->CR &= ~RNG_CR_RNGEN;

            RNG->CR |= RNG_CR_CONDRST;
            RNG->CR &= ~RNG_CR_CONDRST;

            while (RNG->CR & RNG_CR_CONDRST) { /* wait */ }

            RNG->CR |= RNG_CR_RNGEN;
            RNG->CR |= RNG_CR_IE;
        }

        static constinit inline std::atomic<uint32_t> value_{0};
        static constinit inline std::atomic<bool>     ready_{false};
    };
}