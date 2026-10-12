#pragma once
#include <cstdint>
#include "stm32u3xx.h"

namespace stm32u3::wdg
{
    namespace detail
{
    [[gnu::always_inline]]
    static inline void iwdg_start() noexcept
    {
        IWDG->KR = 0xCCCCu;
    }

    [[gnu::always_inline]]
    static inline void iwdg_unlock() noexcept
    {
        IWDG->KR = 0x5555u;
    }

    [[gnu::always_inline]]
    static inline void iwdg_reload() noexcept
    {
        IWDG->KR = 0xAAAAu;
    }

    [[gnu::always_inline]]
    static inline void wait_prescaler() noexcept
    {
        while (IWDG->SR & IWDG_SR_PVU) { /* wait */ }
    }

    [[gnu::always_inline]]
    static inline void wait_reload() noexcept
    {
        while (IWDG->SR & IWDG_SR_RVU) { /* wait */ }
    }

    [[gnu::always_inline]]
    static inline void wait_window() noexcept
    {
        while (IWDG->SR & IWDG_SR_WVU) { /* wait */ }
    }
}

    enum class IwdgPrescaler : std::uint32_t
    {
        div4    = 0x00000000u,
        div8    = IWDG_PR_PR_0,
        div16   = IWDG_PR_PR_1,
        div32   = IWDG_PR_PR_1 | IWDG_PR_PR_0,
        div64   = IWDG_PR_PR_2,
        div128  = IWDG_PR_PR_2 | IWDG_PR_PR_0,
        div256  = IWDG_PR_PR_2 | IWDG_PR_PR_1,
        div512  = IWDG_PR_PR_2 | IWDG_PR_PR_1 | IWDG_PR_PR_0,
        div1024 = IWDG_PR_PR_3
    };

    enum class WwdgPrescaler : std::uint32_t
    {
        div1   = 0x00000000u,
        div2   = WWDG_CFR_WDGTB_0,
        div4   = WWDG_CFR_WDGTB_1,
        div8   = WWDG_CFR_WDGTB_1 | WWDG_CFR_WDGTB_0,
        div16  = WWDG_CFR_WDGTB_2,
        div32  = WWDG_CFR_WDGTB_2 | WWDG_CFR_WDGTB_0,
        div64  = WWDG_CFR_WDGTB_2 | WWDG_CFR_WDGTB_1,
        div128 = WWDG_CFR_WDGTB
    };

    enum class Ewi : std::uint32_t
    {
        disable = 0,
        enable  = IWDG_EWCR_EWIE
    };

    enum class WwdgEwi : std::uint32_t
    {
        disable = 0x00000000u,
        enable  = WWDG_CFR_EWI
    };

    struct Iwdg
    {
        template<
            IwdgPrescaler Prescaler,
            std::uint16_t Reload,
            std::uint16_t Window,
            std::uint16_t EarlyWakeup = 0,
            Ewi EarlyInterrupt = Ewi::disable>
        requires (Reload <= 0x0FFF) &&
                 (Window <= 0x0FFF) &&
                 (EarlyWakeup <= 0x0FFF)
        [[gnu::always_inline]]
        static inline void init() noexcept
        {
            detail::iwdg_start();
            __DSB();
            __ISB();
            detail::iwdg_unlock();
            __DSB();
            __ISB();
            IWDG->PR  = static_cast<std::uint32_t>(Prescaler);
            __DSB();
            __ISB();
            IWDG->RLR = Reload;
            __DSB();
            __ISB();
            detail::wait_prescaler();
            __DSB();
            __ISB();
            detail::wait_reload();
            __DSB();
            __ISB();

            if constexpr (EarlyInterrupt == Ewi::disable)
            {
                IWDG->EWCR = IWDG_EWCR_EWIC;
                __DSB();
                __ISB();
            }
            else
            {
                IWDG->EWCR =
                    IWDG_EWCR_EWIE |
                    IWDG_EWCR_EWIC |
                    EarlyWakeup;
                __DSB();
                __ISB();
            }

            if constexpr (Window != 0x0FFF)
            {
                IWDG->WINR = Window;
                detail::wait_window();
                __DSB();
                __ISB();
            }
            else
            {
                refresh();
                __DSB();
                __ISB();
            }
        }

        [[gnu::always_inline]]
        static inline void refresh() noexcept
        {
            detail::iwdg_reload();
        }
    };

    struct Wwdg
    {
        template<
            std::uint8_t Counter,
            std::uint8_t Window,
            WwdgPrescaler Prescaler,
            WwdgEwi EwiMode = WwdgEwi::disable>
        requires (Counter >= 0x40 && Counter <= 0x7F) &&
                 (Window  >= 0x40 && Window  <= 0x7F)
        [[gnu::always_inline]]
        static inline void init() noexcept
        {
            RCC->APB1ENR1 |= RCC_APB1ENR1_WWDGEN;
            __DSB();
            __ISB();
            WWDG->CR = WWDG_CR_WDGA | Counter;
            __DSB();
            __ISB();
            WWDG->CFR =
                static_cast<std::uint32_t>(Prescaler) |
                Window |
                static_cast<std::uint32_t>(EwiMode);
            __DSB();
            __ISB();
        }

        template<std::uint8_t Counter>
        requires (Counter >= 0x40 && Counter <= 0x7F)
        [[gnu::always_inline]]
        static inline void refresh() noexcept
        {
            WWDG->CR = Counter;
        }
    };
}