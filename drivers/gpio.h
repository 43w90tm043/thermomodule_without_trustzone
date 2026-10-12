#pragma once

#include <concepts>
#include <cstdint>
#include "stm32u3xx.h"

namespace stm32u3::gpio
{

// ------------------------- enums -------------------------

enum class mode : uint32_t
{
    input  = 0,
    output = 1,
    af     = 2,
    analog = 3
};

enum class otype : uint32_t
{
    push_pull  = 0,
    open_drain = 1
};

enum class speed : uint32_t
{
    low       = 0,
    medium    = 1,
    high      = 2,
    very_high = 3
};

enum class pull : uint32_t
{
    none = 0,
    up   = 1,
    down = 2
};

enum class exti_mode : uint32_t
{
    none      = 0,
    interrupt = 1,
    event     = 2,
    both      = 3
};

enum class exti_trigger : uint32_t
{
    none           = 0,
    rising         = 1,
    falling        = 2,
    rising_falling = 3
};

#if defined(CPU_IN_SECURE_STATE)
enum class secure_attr : uint32_t
{
    non_secure = 0,
    secure     = 1
};
#endif

// ------------------------- port traits -------------------------

template<class P>
concept port_like = requires
{
    { P::index } -> std::convertible_to<uint32_t>;
    { P::reg() } -> std::same_as<GPIO_TypeDef*>;
    P::enable_rcc();
};

template<uint32_t Index, uint32_t RccMask>
struct port_base
{
    static constexpr uint32_t index = Index;

    static inline void enable_rcc() noexcept
    {
        RCC->AHB2ENR1 |= RccMask;
        __DSB();
    }
};

struct port_a : port_base<0, RCC_AHB2ENR1_GPIOAEN> { [[nodiscard]] static GPIO_TypeDef* reg() noexcept { return GPIOA; } };
struct port_b : port_base<1, RCC_AHB2ENR1_GPIOBEN> { [[nodiscard]] static GPIO_TypeDef* reg() noexcept { return GPIOB; } };
struct port_c : port_base<2, RCC_AHB2ENR1_GPIOCEN> { [[nodiscard]] static GPIO_TypeDef* reg() noexcept { return GPIOC; } };
struct port_d : port_base<3, RCC_AHB2ENR1_GPIODEN> { [[nodiscard]] static GPIO_TypeDef* reg() noexcept { return GPIOD; } };
struct port_e : port_base<4, RCC_AHB2ENR1_GPIOEEN> { [[nodiscard]] static GPIO_TypeDef* reg() noexcept { return GPIOE; } };
//struct port_f : port_base<5, RCC_AHB2ENR1_GPIOFEN> { [[nodiscard]] static GPIO_TypeDef* reg() noexcept { return GPIOF; } };
struct port_g : port_base<6, RCC_AHB2ENR1_GPIOGEN> { [[nodiscard]] static GPIO_TypeDef* reg() noexcept { return GPIOG; } };
struct port_h : port_base<7, RCC_AHB2ENR1_GPIOHEN> { [[nodiscard]] static GPIO_TypeDef* reg() noexcept { return GPIOH; } };

// ------------------------- pin configuration -------------------------
//   using spi_sck = gpio::pin<gpio::port_a, 5,
//       gpio::pin_config{ .mode = gpio::mode::af,
//                         .speed = gpio::speed::very_high,
//                         .af = 5 }>;

struct pin_config
{
    gpio::mode         mode    = gpio::mode::input;
    gpio::otype        otype   = gpio::otype::push_pull;
    gpio::speed        speed   = gpio::speed::low;
    gpio::pull         pull    = gpio::pull::none;
    uint8_t            af      = 0;
    gpio::exti_mode    exti    = gpio::exti_mode::none;
    gpio::exti_trigger trigger = gpio::exti_trigger::none;
};

// ------------------------- pin abstraction -------------------------

template<port_like Port, uint32_t Pin, pin_config Cfg = pin_config{}>
requires (Pin < 16U) &&
         (Cfg.af <= 15U) &&
         (Cfg.mode == mode::af || Cfg.af == 0U)
class pin
{
private:
    [[nodiscard]] static GPIO_TypeDef* port() noexcept { return Port::reg(); }

    static constexpr uint32_t pos2 = Pin * 2U;
    static constexpr uint32_t pos4 = (Pin & 7U) * 4U;
    static constexpr uint32_t afr_idx = Pin >> 3U;

    static constexpr uint32_t exticr_idx   = Pin >> 2U;
    static constexpr uint32_t exticr_shift = (Pin & 3U) * 8U;

    static constexpr uint32_t lckk = 1U << 16;

    static constexpr uint16_t mask = static_cast<uint16_t>(1U << Pin);
    static constexpr uint32_t mode_bits  = static_cast<uint32_t>(Cfg.mode);
    static constexpr uint32_t pull_bits  = static_cast<uint32_t>(Cfg.pull);
    static constexpr uint32_t otype_bit  = static_cast<uint32_t>(Cfg.otype);
    static constexpr uint32_t speed_bits = static_cast<uint32_t>(Cfg.speed);
    static constexpr uint32_t af_bits    = static_cast<uint32_t>(Cfg.af);

    static constexpr bool is_af     = (Cfg.mode == mode::af);
    static constexpr bool is_out_af = (Cfg.mode == mode::output) || is_af;
    static constexpr bool has_exti  = (Cfg.exti != exti_mode::none);
    static constexpr bool rising    = (Cfg.trigger == exti_trigger::rising) ||
                                      (Cfg.trigger == exti_trigger::rising_falling);
    static constexpr bool falling   = (Cfg.trigger == exti_trigger::falling) ||
                                      (Cfg.trigger == exti_trigger::rising_falling);
    static constexpr bool exti_int  = (Cfg.exti == exti_mode::interrupt) ||
                                      (Cfg.exti == exti_mode::both);
    static constexpr bool exti_evt  = (Cfg.exti == exti_mode::event) ||
                                      (Cfg.exti == exti_mode::both);

public:

    static void init() noexcept
    {
        Port::enable_rcc();

        port()->MODER = (port()->MODER & ~(3U << pos2)) | (mode_bits << pos2);
        port()->PUPDR = (port()->PUPDR & ~(3U << pos2)) | (pull_bits << pos2);

        if constexpr (is_out_af)
        {
            port()->OTYPER  = (port()->OTYPER  & ~(1U << Pin)) | (otype_bit  << Pin);
            port()->OSPEEDR = (port()->OSPEEDR & ~(3U << pos2)) | (speed_bits << pos2);
        }

        if constexpr (is_af)
        {
            port()->AFR[afr_idx] =
                (port()->AFR[afr_idx] & ~(0xFU << pos4)) | (af_bits << pos4);
        }

        if constexpr (has_exti)
        {
            EXTI->EXTICR[exticr_idx] =
                (EXTI->EXTICR[exticr_idx] & ~(0xFFU << exticr_shift)) |
                (Port::index << exticr_shift);

            if constexpr (rising)   { EXTI->RTSR1 |= mask; } else { EXTI->RTSR1 &= ~mask; }
            if constexpr (falling)  { EXTI->FTSR1 |= mask; } else { EXTI->FTSR1 &= ~mask; }
            if constexpr (exti_int) { EXTI->IMR1  |= mask; } else { EXTI->IMR1  &= ~mask; }
            if constexpr (exti_evt) { EXTI->EMR1  |= mask; } else { EXTI->EMR1  &= ~mask; }
        }
    }

    static void deinit() noexcept
    {
        if constexpr (Cfg.exti != exti_mode::none)
        {
            EXTI->IMR1  &= ~mask;
            EXTI->EMR1  &= ~mask;
            EXTI->RTSR1 &= ~mask;
            EXTI->FTSR1 &= ~mask;
            EXTI->RPR1   = mask;
            EXTI->FPR1   = mask;
            EXTI->EXTICR[exticr_idx] &= ~(0xFFU << exticr_shift);
        }

        port()->MODER   |= (3U << pos2);   // analog
        port()->PUPDR   &= ~(3U << pos2);
        port()->OSPEEDR &= ~(3U << pos2);
        port()->OTYPER  &= ~(1U << Pin);
        port()->AFR[afr_idx] &= ~(0xFU << pos4);
    }

    [[nodiscard]] static inline bool read() noexcept { return (port()->IDR & mask) != 0U; }
    static inline void set()   noexcept { port()->BSRR = mask; }
    static inline void reset() noexcept { port()->BRR  = mask; }
    static inline void write(bool state) noexcept { if (state) set(); else reset(); }
    static inline void toggle() noexcept
    {
        const uint32_t odr = port()->ODR;
        port()->BSRR = ((odr & mask) << 16) | (~odr & mask);
    }
    [[nodiscard]] static bool lock() noexcept
    {
        const uint32_t tmp = lckk | mask;

        port()->LCKR = tmp;
        port()->LCKR = mask;
        port()->LCKR = tmp;
        (void)port()->LCKR;

        return (port()->LCKR & lckk) != 0U;
    }

    static inline void enable_high_speed_low_voltage()  noexcept { port()->HSLVR |= mask; }
    static inline void disable_high_speed_low_voltage() noexcept { port()->HSLVR &= ~static_cast<uint32_t>(mask); }

    [[nodiscard]] static inline bool exti_rising_pending()  noexcept requires (Cfg.exti != exti_mode::none) { return (EXTI->RPR1 & mask) != 0U; }
    [[nodiscard]] static inline bool exti_falling_pending() noexcept requires (Cfg.exti != exti_mode::none) { return (EXTI->FPR1 & mask) != 0U; }
    [[nodiscard]] static inline bool exti_pending()         noexcept requires (Cfg.exti != exti_mode::none) { return exti_rising_pending() || exti_falling_pending(); }

    static inline void exti_clear_rising()  noexcept requires (Cfg.exti != exti_mode::none) { EXTI->RPR1 = mask; }
    static inline void exti_clear_falling() noexcept requires (Cfg.exti != exti_mode::none) { EXTI->FPR1 = mask; }
    static inline void exti_clear()         noexcept requires (Cfg.exti != exti_mode::none) { exti_clear_rising(); exti_clear_falling(); }

    static inline void exti_generate_swi()  noexcept requires (Cfg.exti != exti_mode::none) { EXTI->SWIER1 = mask; }

#if defined(CPU_IN_SECURE_STATE)
    // Confirm SECCFGR offset against the device header
    static inline void set_secure_attr(secure_attr attr) noexcept
    {
        if (attr == secure_attr::secure)
            port()->SECCFGR |= mask;
        else
            port()->SECCFGR &= ~static_cast<uint32_t>(mask);
    }

    [[nodiscard]] static inline secure_attr get_secure_attr() noexcept
    {
        return (port()->SECCFGR & mask) != 0U ? secure_attr::secure : secure_attr::non_secure;
    }
#endif
};

// ------------------------- group operations -------------------------
template<port_like Port>
inline void configure_pin(uint32_t pin_no, const pin_config& c) noexcept
{
    Port::enable_rcc();
    GPIO_TypeDef* const p = Port::reg();

    const uint32_t pos2 = pin_no * 2U;
    const uint32_t pos4 = (pin_no & 7U) * 4U;
    const uint32_t idx  = pin_no >> 3U;

    p->MODER = (p->MODER & ~(3U << pos2)) | (static_cast<uint32_t>(c.mode) << pos2);
    p->PUPDR = (p->PUPDR & ~(3U << pos2)) | (static_cast<uint32_t>(c.pull) << pos2);

    if (c.mode == mode::output || c.mode == mode::af)
    {
        p->OTYPER  = (p->OTYPER  & ~(1U << pin_no)) | (static_cast<uint32_t>(c.otype) << pin_no);
        p->OSPEEDR = (p->OSPEEDR & ~(3U << pos2))   | (static_cast<uint32_t>(c.speed) << pos2);
    }
    if (c.mode == mode::af)
    {
        p->AFR[idx] = (p->AFR[idx] & ~(0xFU << pos4)) | (static_cast<uint32_t>(c.af) << pos4);
    }
}

template<class... Pins>
struct group
{
    static void init_all()   noexcept { (Pins::init(), ...); }
    static void deinit_all() noexcept { (Pins::deinit(), ...); }
    static void set_all()    noexcept { (Pins::set(), ...); }
    static void reset_all()  noexcept { (Pins::reset(), ...); }
    static void toggle_all() noexcept { (Pins::toggle(), ...); }
};

// ------------------------- atomic multi-pin write -------------------------

template<port_like Port>
inline void write_multiple_state(uint16_t set_mask, uint16_t reset_mask) noexcept
{
    Port::reg()->BSRR = (static_cast<uint32_t>(reset_mask) << 16) | set_mask;
}

// ------------------------- EXTI IRQ dispatch -------------------------
//   gpio::exti_irq_handler(0x00FF,
//       [](uint16_t m) { /* rising  */ },
//       [](uint16_t m) { /* falling */ });

template<std::invocable<uint16_t> OnRising, std::invocable<uint16_t> OnFalling>
inline void exti_irq_handler(uint16_t line_mask, OnRising&& on_rising, OnFalling&& on_falling)
{
    const uint32_t rising = EXTI->RPR1 & line_mask;
    if (rising != 0U)
    {
        EXTI->RPR1 = rising;
        on_rising(static_cast<uint16_t>(rising));
    }

    const uint32_t falling = EXTI->FPR1 & line_mask;
    if (falling != 0U)
    {
        EXTI->FPR1 = falling;
        on_falling(static_cast<uint16_t>(falling));
    }
}

} // namespace stm32u3::gpio