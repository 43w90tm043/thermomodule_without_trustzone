#pragma once

#include <atomic>
#include <chrono>
#include <concepts>
#include <cstdint>
#include "stm32u3xx.h"

namespace stm32u3::clock
{
    constinit inline std::atomic<std::uint32_t> ticks{0};


    static inline void init() noexcept
    {
        NVIC_SetPriorityGrouping(3U);

        /* Enable PWR clock */
        RCC->AHB1ENR2 |= RCC_AHB1ENR2_PWREN;
        __DSB();
        (void)RCC->AHB1ENR2;
        __DSB();
        __ISB();

        /* 1. Select SMPS power supply */
        PWR->CR3 |= PWR_CR3_REGSEL;
        __DSB();
        __ISB();

        while ((PWR->SVMSR & PWR_SVMSR_REGS) == 0U) { /* wait */ }

        /* 2. EPOD booster clock config */
        RCC->CFGR4 =
            (RCC->CFGR4 &
             ~(RCC_CFGR4_BOOSTSEL | RCC_CFGR4_BOOSTDIV))
            | RCC_CFGR4_BOOSTSEL_0;
        __DSB();
        __ISB();

        /* 3. Enable EPOD booster */
        PWR->VOSR |= PWR_VOSR_BOOSTEN;
        __DSB();
        __ISB();

        while ((PWR->VOSR & PWR_VOSR_BOOSTRDY) == 0U) { /* wait */ }

        /* 4. Voltage scaling VOS Range 1 */
        PWR->VOSR =
            (PWR->VOSR &
             ~(PWR_VOSR_R2EN | PWR_VOSR_R1EN))
            | PWR_VOSR_R1EN;
        __DSB();
        __ISB();

        while ((PWR->VOSR & PWR_VOSR_R1RDY) == 0U) { /* wait */ }

        /* 5. Flash configuration */
        RCC->AHB1ENR1 |= RCC_AHB1ENR1_FLASHEN;
        __DSB();
        __ISB();

        FLASH->ACR =
            (FLASH->ACR & ~FLASH_ACR_LATENCY)
            | FLASH_ACR_LATENCY_1
            | FLASH_ACR_PRFTEN;

        __DSB();
        __ISB();

        /* 6.1 HSE 16 MHz */
        RCC->CR |= RCC_CR_HSEBYP;
        RCC->CR |= RCC_CR_HSEON;
        __DSB();
        __ISB();

        while ((RCC->CR & RCC_CR_HSERDY) == 0U) { /* wait */ }

        /* 6.2 LSI */
        RCC->CSR |= RCC_CSR_LSION;
        __DSB();
        __ISB();

        while ((RCC->CSR & RCC_CSR_LSIRDY) == 0U) { /* wait */ }

        /* 7. MSIS */
        RCC->CR |= RCC_CR_MSISON;
        __DSB();
        __ISB();

        while ((RCC->CR & RCC_CR_MSISRDY) == 0U) { /* wait */ }

        RCC->ICSCR1 =
            (RCC->ICSCR1 &
             ~(RCC_ICSCR1_MSISSEL | RCC_ICSCR1_MSISDIV))
            | RCC_ICSCR1_MSIRGSEL;

        RCC->ICSCR1 =
            (RCC->ICSCR1 & ~RCC_ICSCR1_MSIHSINDIV)
            | RCC_ICSCR1_MSIPLL0SEL;

        RCC->CR |= RCC_CR_MSIPLL0EN;
        __DSB();
        __ISB();

        while ((RCC->CR & RCC_CR_MSIPLL0RDY) == 0U) { /* wait */ }

        /* 8. Bus prescalers */
        RCC->CFGR2 &=
            ~(RCC_CFGR2_HPRE |
              RCC_CFGR2_PPRE1 |
              RCC_CFGR2_PPRE2);

        RCC->CFGR3 &= ~RCC_CFGR3_PPRE3;

        __DSB();
        __ISB();

        /* 9. Switch SYSCLK */
        RCC->CFGR1 &= ~RCC_CFGR1_SW;
        __DSB();
        __ISB();

        while ((RCC->CFGR1 & RCC_CFGR1_SWS) != 0U) { /* wait */ }

        SystemCoreClockUpdate();

        /* 10. ICACHE */
        ICACHE->CR &= ~ICACHE_CR_EN;
        ICACHE->CR = ICACHE_CR_WAYSEL | ICACHE_CR_CACHEINV;

        while (ICACHE->SR & ICACHE_SR_BUSYF) { /* wait */ }

        ICACHE->CR |= ICACHE_CR_EN;
    }


    static inline void dwt_init() noexcept
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0U;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        __DSB();
        __ISB();
    }


    static inline void dwt_reset() noexcept
    {
        DWT->CYCCNT = 0U;

        __DSB();
        __ISB();
    }


    [[nodiscard]]
    static inline std::uint32_t cycles() noexcept
    {
        return DWT->CYCCNT;
    }


    [[nodiscard]]
    static inline std::uint32_t cycles_elapsed(
        std::uint32_t start) noexcept
    {
        return DWT->CYCCNT - start;
    }


    [[nodiscard]]
    static inline std::chrono::microseconds cycles_to_us(
        std::uint32_t cycle_count) noexcept
    {
        return std::chrono::microseconds{
            (static_cast<std::uint64_t>(cycle_count) * 1'000'000ULL)
            / SystemCoreClock
        };
    }
    template<std::uint32_t Hz>
    requires (Hz > 0U) && (Hz <= 1'000'000U)
    static inline void systick_init() noexcept
    {
        SysTick_Config(SystemCoreClock / Hz);
    }


    static inline void systick_irq_enable() noexcept
    {
        SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
    }


    static inline void systick_irq_disable() noexcept
    {
        SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
    }

    [[nodiscard]]
    static inline std::chrono::milliseconds millis() noexcept
    {
        return std::chrono::milliseconds{ticks.load(std::memory_order_relaxed)};
    }
    
    static inline void delay_ms(std::uint32_t ms) noexcept
    {
        const std::uint32_t start = ticks.load(std::memory_order_relaxed);

        while ((ticks.load(std::memory_order_relaxed) - start) < ms)
        {
            __NOP();
        }
    }


    static inline void delay_ms(std::chrono::milliseconds ms) noexcept
    {
        delay_ms(static_cast<std::uint32_t>(ms.count()));
    }

    static inline void delay_us(std::uint32_t us) noexcept
    {
        std::uint64_t remaining =
            (static_cast<std::uint64_t>(us) * SystemCoreClock) / 1'000'000ULL;

        constexpr std::uint64_t max_chunk = 0x7FFF'FFFFULL;

        while (remaining != 0U)
        {
            const std::uint32_t chunk = static_cast<std::uint32_t>(
                remaining > max_chunk ? max_chunk : remaining);

            const std::uint32_t start = DWT->CYCCNT;

            while ((DWT->CYCCNT - start) < chunk)
            {
                __NOP();
            }

            remaining -= chunk;
        }
    }

    static inline void delay_us(std::chrono::microseconds us) noexcept
    {
        delay_us(static_cast<std::uint32_t>(us.count()));
    }

    template<typename Rep, typename Period>
    static inline void delay(std::chrono::duration<Rep, Period> d) noexcept
    {
        delay_us(std::chrono::duration_cast<std::chrono::microseconds>(d));
    }
}


extern "C"
void SysTick_Handler()
{
    stm32u3::clock::ticks.fetch_add(1U, std::memory_order_relaxed);
}