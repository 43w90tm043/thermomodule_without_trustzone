#pragma once

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>
#include "stm32u3xx.h"

namespace stm32u3
{
  namespace crc {

//------------------------------------------------------------------------------
// Common CRC constants
//------------------------------------------------------------------------------

inline constexpr uint32_t polynomial_crc32 = 0x04C11DB7UL;
inline constexpr uint32_t polynomial_crc16 = 0x1021UL;
inline constexpr uint32_t polynomial_crc8 = 0x07UL;

inline constexpr uint32_t init_crc32 = 0xFFFFFFFFUL;
inline constexpr uint32_t init_crc16 = 0xFFFFUL;
inline constexpr uint32_t init_crc8 = 0xFFUL;

//------------------------------------------------------------------------------
// Configuration enums
//------------------------------------------------------------------------------

enum class polynomial_size : uint32_t
{
    bits32 = 0u << CRC_CR_POLYSIZE_Pos,
    bits16 = 1u << CRC_CR_POLYSIZE_Pos,
    bits8  = 2u << CRC_CR_POLYSIZE_Pos,
    bits7  = 3u << CRC_CR_POLYSIZE_Pos,
};

enum class input_reverse : uint32_t
{
    none     = 0,

    byte     = (1u << CRC_CR_RTYPE_IN_Pos) |
               (1u << CRC_CR_REV_IN_Pos),

    halfword = (1u << CRC_CR_RTYPE_IN_Pos) |
               (2u << CRC_CR_REV_IN_Pos),

    word     = (0u << CRC_CR_RTYPE_IN_Pos) |
               (3u << CRC_CR_REV_IN_Pos),
};

enum class output_reverse : uint32_t
{
    disable = 0,

    enable  = (0u << CRC_CR_RTYPE_OUT_Pos) |
              (1u << CRC_CR_REV_OUT_Pos),
};

//------------------------------------------------------------------------------
// Configuration
//------------------------------------------------------------------------------

struct config
{
    uint32_t        polynomial;
    uint32_t        init;
    polynomial_size poly_size;
    input_reverse   input;
    output_reverse  output;
};

template<typename T>
concept crc_word =
    std::same_as<T, uint8_t>  ||
    std::same_as<T, uint16_t> ||
    std::same_as<T, uint32_t>;

//------------------------------------------------------------------------------
// Driver
//------------------------------------------------------------------------------

class peripheral
{
public:

    static inline void init(const config& cfg) noexcept
    {
        RCC->AHB1ENR1 |= RCC_AHB1ENR1_CRCEN;

        __DSB();
        __ISB();

        uint32_t cr = CRC->CR;

        cr &= ~(CRC_CR_POLYSIZE |
                CRC_CR_RTYPE_IN | CRC_CR_REV_IN |
                CRC_CR_RTYPE_OUT | CRC_CR_REV_OUT);

        cr |= static_cast<uint32_t>(cfg.poly_size);
        cr |= static_cast<uint32_t>(cfg.input);
        cr |= static_cast<uint32_t>(cfg.output);

        CRC->POL  = cfg.polynomial;
        CRC->INIT = cfg.init;
        CRC->CR   = cr;

        reset();
    }

    static inline void reset() noexcept
    {
        CRC->CR |= CRC_CR_RESET;
    }
    
    template<crc_word T>
    static inline void accumulate(std::span<const T> data) noexcept
    {
        volatile T* dr = reinterpret_cast<volatile T*>(&CRC->DR);
        for (const T value : data)
        {
            *dr = value;
        }
    }

    template<crc_word T>
    static inline void accumulate(const T* data, std::size_t count) noexcept
    {
        accumulate(std::span<const T>(data, count));
    }

    template<crc_word T>
    [[nodiscard]]
    static inline uint32_t calculate(std::span<const T> data) noexcept
    {
        reset();
        accumulate(data);
        return value();
    }

    template<crc_word T>
    [[nodiscard]]
    static inline uint32_t calculate(const T* data, std::size_t count) noexcept
    {
        return calculate(std::span<const T>(data, count));
    }

    [[nodiscard]]
    static inline uint32_t value() noexcept
    {
        return CRC->DR;
    }
};

//------------------------------------------------------------------------------
// Ready-to-use configurations
//------------------------------------------------------------------------------

inline constexpr config crc32 =
{
    .polynomial = polynomial_crc32,
    .init       = init_crc32,
    .poly_size  = polynomial_size::bits32,
    .input      = input_reverse::byte,
    .output     = output_reverse::enable
};

inline constexpr config crc16 =
{
    .polynomial = polynomial_crc16,
    .init       = init_crc16,
    .poly_size  = polynomial_size::bits16,
    .input      = input_reverse::none,
    .output     = output_reverse::disable
};

inline constexpr config crc8 =
{
    .polynomial = polynomial_crc8,
    .init       = init_crc8,
    .poly_size  = polynomial_size::bits8,
    .input      = input_reverse::none,
    .output     = output_reverse::disable
};

} // namespace crc
} // namespace stm32u3
//    crc::peripheral::init(crc::crc8);
//    x = crc::peripheral::calculate<uint8_t>(rx, sizeof(rx));
//    x = crc::peripheral::calculate(std::span{rx});