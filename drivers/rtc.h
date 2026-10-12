#pragma once

#include <chrono>
#include <cstdint>
#include "stm32u3xx.h"

namespace stm32u3::rtc
{
    using namespace std::chrono_literals;
    using Time = std::chrono::hh_mm_ss<std::chrono::seconds>;

    enum class CalibrationSign : uint8_t { Negative, Positive };

    class Driver
    {
        struct WriteGuard
        {
            WriteGuard() noexcept  { RTC->WPR = 0xCA; RTC->WPR = 0x53; }
            ~WriteGuard() noexcept { RTC->WPR = 0xFF; }
            WriteGuard(const WriteGuard&) = delete;
            WriteGuard& operator=(const WriteGuard&) = delete;
        };
        struct InitModeGuard
        {
            InitModeGuard() noexcept
            {
                RTC->ICSR |= RTC_ICSR_INIT;
                while (!(RTC->ICSR & RTC_ICSR_INITF)) { /* wait */ }
            }
            ~InitModeGuard() noexcept { RTC->ICSR &= ~RTC_ICSR_INIT; }
            InitModeGuard(const InitModeGuard&) = delete;
            InitModeGuard& operator=(const InitModeGuard&) = delete;
        };

        static constexpr uint32_t async_prescaler = 99u;
        static constexpr uint32_t sync_prescaler  = 4999u;

    public:

        static inline void force_reinit() noexcept
        {
            RCC->APB1ENR1 |= RCC_APB1ENR1_RTCAPBEN;

            PWR->DBPR |= PWR_DBPR_DBP;
            while (!(PWR->DBPR & PWR_DBPR_DBP)) { /* wait */ }

            RCC->BDCR |= RCC_BDCR_BDRST;
            RCC->BDCR &= ~RCC_BDCR_BDRST;
        }

        static inline void init() noexcept
        {
            RCC->APB1ENR1 |= RCC_APB1ENR1_RTCAPBEN;

            PWR->DBPR |= PWR_DBPR_DBP;
            while (!(PWR->DBPR & PWR_DBPR_DBP)) { /* wait */ }

            if (RCC->BDCR & RCC_BDCR_RTCEN)
                return;

            RCC->BDCR &= ~RCC_BDCR_RTCSEL;
            RCC->BDCR |= (RCC_BDCR_RTCSEL_1 | RCC_BDCR_RTCSEL_0);
            RCC->BDCR |= RCC_BDCR_RTCEN;

            {
                WriteGuard wg;
                InitModeGuard ig;
                RTC->PRER = (async_prescaler << 16) | sync_prescaler;
                RTC->CR  |= RTC_CR_BYPSHAD;
            }

            RTC->ICSR &= ~RTC_ICSR_RSF;
            while (!(RTC->ICSR & RTC_ICSR_RSF)) { /* wait */ }
        }

        static inline void set_time(
            std::chrono::hours h,
            std::chrono::minutes m,
            std::chrono::seconds s) noexcept
        {
            const auto hh = static_cast<uint32_t>(h.count());
            const auto mm = static_cast<uint32_t>(m.count());
            const auto ss = static_cast<uint32_t>(s.count());

            uint32_t tr = 0;
            tr |= ((hh / 10) << RTC_TR_HT_Pos)  & RTC_TR_HT_Msk;
            tr |= ((hh % 10) << RTC_TR_HU_Pos)  & RTC_TR_HU_Msk;
            tr |= ((mm / 10) << RTC_TR_MNT_Pos) & RTC_TR_MNT_Msk;
            tr |= ((mm % 10) << RTC_TR_MNU_Pos) & RTC_TR_MNU_Msk;
            tr |= ((ss / 10) << RTC_TR_ST_Pos)  & RTC_TR_ST_Msk;
            tr |= ((ss % 10) << RTC_TR_SU_Pos)  & RTC_TR_SU_Msk;

            WriteGuard wg;
            {
                InitModeGuard ig;
                RTC->TR = tr;
            }
            while (RTC->ICSR & RTC_ICSR_INITF) { /* wait */ }
        }

        static inline void calibrate(CalibrationSign sign, uint16_t calm) noexcept
        {
            WriteGuard wg;
            while (RTC->ICSR & RTC_ICSR_RECALPF) { /* wait */ }

            uint32_t calr = calm & RTC_CALR_CALM_Msk;
            if (sign == CalibrationSign::Positive)
                calr |= RTC_CALR_CALP;

            RTC->CALR = calr;
        }

        [[nodiscard]]
        static inline Time time() noexcept
        {
            const uint32_t tr = RTC->TR;
            [[maybe_unused]] const uint32_t dr = RTC->DR; // required read to unlock the shadow regs (RM)

            const uint32_t h = ((tr & RTC_TR_HT_Msk)  >> RTC_TR_HT_Pos)  * 10 + ((tr & RTC_TR_HU_Msk)  >> RTC_TR_HU_Pos);
            const uint32_t m = ((tr & RTC_TR_MNT_Msk) >> RTC_TR_MNT_Pos) * 10 + ((tr & RTC_TR_MNU_Msk) >> RTC_TR_MNU_Pos);
            const uint32_t s = ((tr & RTC_TR_ST_Msk)  >> RTC_TR_ST_Pos)  * 10 + ((tr & RTC_TR_SU_Msk)  >> RTC_TR_SU_Pos);

            return Time{std::chrono::hours{h} + std::chrono::minutes{m} + std::chrono::seconds{s}};
        }
    };
}