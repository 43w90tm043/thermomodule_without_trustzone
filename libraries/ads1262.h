#pragma once

#include "stm32u3xx.h"
#include "clock.h"

#include <array>
#include <atomic>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>
#include <type_traits>

namespace stm32u3::drivers
{

// ============================================================================
// Constants
// ============================================================================
namespace ads1262_constants
{

inline constexpr std::string_view version = "4.0.0-cpp20";

enum class command : uint8_t
{
    nop     = 0x00,
    reset   = 0x06,
    start   = 0x08,
    stop    = 0x0A,
    rdata   = 0x12,
    syocal1 = 0x16, // system offset calibration (ADC1)
    sygcal1 = 0x17, // system gain calibration (ADC1)
    sfocal1 = 0x19, // self offset calibration (ADC1)
    rreg    = 0x20,
    wreg    = 0x40
};

enum class reg : uint8_t
{
    id        = 0x00,
    power     = 0x01,
    interface = 0x02,
    mode0     = 0x03,
    mode1     = 0x04,
    mode2     = 0x05,
    inpmux    = 0x06,
    ofcal0    = 0x07,
    ofcal1    = 0x08,
    ofcal2    = 0x09,
    fscal0    = 0x0A,
    fscal1    = 0x0B,
    fscal2    = 0x0C,
    idacmux   = 0x0D,
    idacmag   = 0x0E,
    refmux    = 0x0F,
    tdacp     = 0x10,
    tdacn     = 0x11,
    gpiocon   = 0x12,
    gpiodir   = 0x13,
    gpiodat   = 0x14
};

inline constexpr uint8_t inpmux_temperature_sensor     = 0xBB;
inline constexpr uint8_t inpmux_analog_supply_monitor  = 0xCC;
inline constexpr uint8_t inpmux_digital_supply_monitor = 0xDD;

// INTERFACE register (0x02): CRC[1:0], STATUS byte disabled.
enum class crc_mode : uint8_t { off = 0x00, checksum = 0x01, crc = 0x02 };

inline constexpr uint8_t interface_crc_mask   = 0x03;
inline constexpr uint8_t checksum_mode_offset = 0x9B;
inline constexpr uint8_t crc8_atm_polynomial  = 0x07;
inline constexpr std::size_t frame_len = 6;
inline constexpr uint8_t gpio_mask = 0x0F; // ADS1262 has GPIO0..GPIO3

// ---- CRC-8 (poly 0x07, init 0), table generated at compile time -----------
inline constexpr std::array<uint8_t, 256> crc8_table = []() constexpr
{
    std::array<uint8_t, 256> t{};
    for (std::size_t i = 0; i < t.size(); ++i)
    {
        auto c = static_cast<uint8_t>(i);
        for (int bit = 0; bit < 8; ++bit)
        {
            c = static_cast<uint8_t>(
                ((c & 0x80U) != 0U) ? ((c << 1) ^ crc8_atm_polynomial)
                                    : (c << 1));
        }
        t[i] = c;
    }
    return t;
}();

[[nodiscard]] constexpr uint8_t crc8(std::span<const uint8_t> data) noexcept
{
    uint8_t c = 0;
    for (const uint8_t b : data)
    {
        c = crc8_table[c ^ b];
    }
    return c;
}

[[nodiscard]] constexpr uint8_t ads_crc(std::span<const uint8_t> data) noexcept
{
    return static_cast<uint8_t>(crc8(data));
}

namespace detail
{
inline constexpr std::array<uint8_t, 9> crc_check{
    '1', '2', '3', '4', '5', '6', '7', '8', '9'};
}

static_assert(crc8(detail::crc_check) == 0xF4);

} // namespace ads1262_constants

// ============================================================================
// Configuration types
// ============================================================================

enum class data_rate : uint8_t
{
    sps_2_5 = 0x00, sps_5 = 0x01, sps_10 = 0x02, sps_16_6 = 0x03,
    sps_20  = 0x04, sps_50 = 0x05, sps_60 = 0x06, sps_100  = 0x07,
    sps_400 = 0x08, sps_1200 = 0x09, sps_2400 = 0x0A, sps_4800 = 0x0B,
    sps_7200 = 0x0C, sps_14400 = 0x0D, sps_19200 = 0x0E, sps_38400 = 0x0F
};

// Data rate in units of 0.1 SPS.
inline constexpr std::array<uint32_t, 16> data_rate_sps_x10{
    25, 50, 100, 166, 200, 500, 600, 1000,
    4000, 12000, 24000, 48000, 72000, 144000, 192000, 384000};

[[nodiscard]] constexpr uint32_t conversion_period_ms(data_rate dr) noexcept
{
    const uint32_t x10 = data_rate_sps_x10[static_cast<std::size_t>(dr)];
    return (10'000U + x10 - 1U) / x10; // ceil, >= 1
}

enum class gain : uint8_t { x1 = 0, x2 = 1, x4 = 2, x8 = 3, x16 = 4, x32 = 5 };

[[nodiscard]] constexpr uint32_t gain_factor(gain g) noexcept
{
    return 1U << static_cast<uint8_t>(g);
}

enum class input_channel : uint8_t
{
    ain0 = 0x00, ain1 = 0x01, ain2 = 0x02, ain3 = 0x03, ain4 = 0x04,
    ain5 = 0x05, ain6 = 0x06, ain7 = 0x07, ain8 = 0x08, ain9 = 0x09,
    aincom       = 0x0A,
    temperature  = 0x0B,
    avdd_monitor = 0x0C,
    dvdd_monitor = 0x0D,
    tdac_test    = 0x0E,
    floating     = 0x0F
};

[[nodiscard]] constexpr uint8_t refmux_pair(uint8_t sel) noexcept
{
    return static_cast<uint8_t>((sel << 3) | sel); 
}

enum class reference : uint8_t
{
    internal_2_5v      = refmux_pair(0),
    external_ain0_ain1 = refmux_pair(1),
    external_ain2_ain3 = refmux_pair(2),
    external_ain4_ain5 = refmux_pair(3),
    internal_avdd_avss = refmux_pair(4)
};

[[nodiscard]] constexpr std::optional<float> nominal_vref(reference r) noexcept
{
    switch (r)
    {
    case reference::internal_2_5v:      return 2.5F;
    case reference::internal_avdd_avss: return 5.0F;
    default:                            return std::nullopt;
    }
}

enum class filter : uint8_t { sinc1 = 0, sinc2 = 1, sinc3 = 2, sinc4 = 3, fir = 4 };

struct config
{
    data_rate     dr  = data_rate::sps_2_5;
    gain          g   = gain::x1;
    reference     ref = reference::internal_avdd_avss;
    filter        flt = filter::sinc4;
    bool          pga_bypass = false;
    bool          internal_reference = true; 
    input_channel positive_input = input_channel::ain0;
    input_channel negative_input = input_channel::ain1;
    uint8_t settle_samples = 5;
    bool use_start_pin = false;
    float vref_volts = 0.0F;
};

struct hw_options
{
    bool     spi_drdy_gated = false;
    uint32_t cs_setup_us    = 1;
};

// ============================================================================
// Results
// ============================================================================

enum class [[nodiscard]] error : uint8_t
{
    ok = 0,
    not_initialized,
    invalid_config,
    invalid_argument,
    busy,              // async transfer in flight / SPI master busy
    not_ready,         // DRDY not asserted (non-gated mode)
    timeout,
    spi_failure,
    crc_failure,
    register_mismatch 
};

template <class T>
struct [[nodiscard]] result
{
    error err = error::ok;
    T     value{};

    [[nodiscard]] constexpr explicit operator bool() const noexcept
    {
        return err == error::ok;
    }
};

enum class read_status : uint8_t
{
    ok,
    settling,       
    crc_error,
    transfer_error
};

struct sample
{
    int32_t     code   = 0;
    read_status status = read_status::ok;
};

struct statistics
{
    uint32_t samples;
    uint32_t crc_errors;
    uint32_t transfer_errors;
};

// ============================================================================
// Concepts
// ============================================================================

template <class S>
concept spi_master = requires(std::span<const uint8_t> tx,
                              std::span<uint8_t> rx,
                              typename S::callback_t cb)
{
    { S::transmit(tx) }                    noexcept -> std::same_as<bool>;
    { S::transmit_receive(tx, rx) }        noexcept -> std::same_as<bool>;
    { S::transmit_receive_dma(tx, rx, cb) } noexcept -> std::same_as<bool>;
    { S::busy() }                          noexcept -> std::same_as<bool>;
};

template <class T>
concept output_pin = requires { T::set(); T::reset(); };

template <class T>
concept input_pin = requires { { T::read() } -> std::same_as<bool>; };

// ============================================================================
// Driver
// ============================================================================

template <spi_master  SpiMaster,
          output_pin  CsPin,
          input_pin   DrdyPin,
          class       StartPin = void,
          class       PwdnPin  = void,
          hw_options  Opt      = hw_options{}>
    requires (std::is_void_v<StartPin> || output_pin<StartPin>) &&
             (std::is_void_v<PwdnPin>  || output_pin<PwdnPin>)
class ads1262
{
    using command = ads1262_constants::command;
    using reg     = ads1262_constants::reg;

    static constexpr bool has_start = !std::is_void_v<StartPin>;
    static constexpr bool has_pwdn  = !std::is_void_v<PwdnPin>;

    static constexpr std::size_t frame_len = ads1262_constants::frame_len;
    static constexpr std::size_t xfer_len  = frame_len + 1; // +1 пустой байт

    static constexpr uint32_t power_up_settle_ms = 100;
    static constexpr uint32_t reset_delay_ms     = 100;
    static constexpr uint32_t inter_command_us   = 10;
    static constexpr uint8_t  max_read_attempts  = 3;

    static constexpr float lsb_scale = 1.0F / 2147483648.0F;

    enum class async_mode : uint8_t { idle, one_shot, streaming };

public:
    using callback_t = void (*)(const sample&) noexcept;

    ads1262() = delete;

    // ------------------------------------------------------------------
    // Lifecycle
    // ------------------------------------------------------------------
    [[nodiscard]] static error init(const config& cfg) noexcept
    {
        if (!valid(cfg))
        {
            return error::invalid_config;
        }
        if (mode_.load(std::memory_order_acquire) != async_mode::idle ||
            SpiMaster::busy())
        {
            return error::busy;
        }

        cfg_         = cfg;
        vref_        = resolve_vref(cfg);
        initialized_ = false;
        converting_  = false;
        settle_remaining_.store(cfg.settle_samples, std::memory_order_relaxed);
        reset_statistics();
        init_pin<CsPin>();
        init_pin<DrdyPin>();
        if constexpr (has_start) { init_pin<StartPin>(); }
        if constexpr (has_pwdn)  { init_pin<PwdnPin>(); }

        CsPin::set();
        if constexpr (has_start) { StartPin::reset(); }

        power_up();

        if (const error e = command_raw(command::reset); e != error::ok) { return e; }
        clock::delay_ms(reset_delay_ms);

        if (const error e = apply_configuration(); e != error::ok) { return e; }

        initialized_ = true;
        return error::ok;
    }

    [[nodiscard]] static error deinit() noexcept
    {
        if (!initialized_) { return error::ok; }
        if (mode_.load(std::memory_order_acquire) != async_mode::idle)
        {
            return error::busy;
        }

        const error e = stop_conversion();
        power_down();
        initialized_ = false;
        return e;
    }

    [[nodiscard]] static error reset() noexcept
    {
        if (const error e = guard_idle(); e != error::ok) { return e; }

        if (const error e = command_raw(command::reset); e != error::ok) { return e; }
        clock::delay_ms(reset_delay_ms);
        converting_ = false;
        return apply_configuration();
    }

    static void power_up() noexcept
    {
        if constexpr (has_pwdn)
        {
            PwdnPin::set();
            clock::delay_ms(power_up_settle_ms);
        }
    }

    static void power_down() noexcept
    {
        if constexpr (has_pwdn) { PwdnPin::reset(); }
        converting_ = false;
    }

    // ------------------------------------------------------------------
    // Conversion control
    // ------------------------------------------------------------------
    [[nodiscard]] static error start_conversion() noexcept
    {
        if (const error e = guard_idle(); e != error::ok) { return e; }
        return start_conversion_raw();
    }

    [[nodiscard]] static error stop_conversion() noexcept
    {
        if (const error e = guard_idle(); e != error::ok) { return e; }

        if constexpr (has_start) { StartPin::reset(); }

        const error e = command_raw(command::stop);
        converting_ = false;
        return e;
    }

    [[nodiscard]] static bool is_data_ready() noexcept { return !DrdyPin::read(); }

    // ------------------------------------------------------------------
    // Settings (config is the source of truth; registers are rewritten whole)
    // ------------------------------------------------------------------
    [[nodiscard]] static error set_data_rate(data_rate dr) noexcept
    {
        if (const error e = guard_idle(); e != error::ok) { return e; }
        config next = cfg_;
        next.dr = dr;
        return commit_mode2(next);
    }

    [[nodiscard]] static error set_gain(gain g) noexcept
    {
        if (const error e = guard_idle(); e != error::ok) { return e; }
        config next = cfg_;
        next.g = g;
        return commit_mode2(next);
    }

    [[nodiscard]] static error set_pga_bypass(bool bypass) noexcept
    {
        if (const error e = guard_idle(); e != error::ok) { return e; }
        config next = cfg_;
        next.pga_bypass = bypass;
        return commit_mode2(next);
    }

    [[nodiscard]] static error set_input_mux(input_channel positive,
                                             input_channel negative) noexcept
    {
        if (const error e = guard_idle(); e != error::ok) { return e; }

        config next = cfg_;
        next.positive_input = positive;
        next.negative_input = negative;

        if (const error e = write_reg_raw(reg::inpmux, build_inpmux(next));
            e != error::ok)
        {
            return e;
        }

        cfg_ = next;
        settle_remaining_.store(cfg_.settle_samples, std::memory_order_relaxed);
        return error::ok;
    }

    [[nodiscard]] static error set_vref(float volts) noexcept
    {
        if (!(volts > 0.0F)) { return error::invalid_argument; }
        vref_ = volts;
        return error::ok;
    }

    // ------------------------------------------------------------------
    // Blocking reads
    // ------------------------------------------------------------------
    [[nodiscard]] static result<int32_t> read_raw() noexcept
    {
        if (const error e = guard_idle(); e != error::ok) { return {e, 0}; }
        return read_raw_unchecked();
    }

    [[nodiscard]] static result<float> read_voltage() noexcept
    {
        const auto raw = read_raw();
        if (!raw) { return {raw.err, 0.0F}; }
        return {error::ok, counts_to_voltage(raw.value)};
    }

    [[nodiscard]] static result<float> read_temperature_celsius() noexcept
    {
        if (const error e = guard_idle(); e != error::ok) { return {e, 0.0F}; }

        config temp_cfg = cfg_;
        temp_cfg.g = gain::x1;

        error e = write_reg_raw(reg::mode2, build_mode2(temp_cfg));
        if (e == error::ok)
        {
            e = write_reg_raw(reg::inpmux,
                              ads1262_constants::inpmux_temperature_sensor);
        }

        result<int32_t> raw{e, 0};
        if (e == error::ok)
        {
            settle_remaining_.store(cfg_.settle_samples, std::memory_order_relaxed);
            raw = read_raw_unchecked();
        }

        const error r1 = write_reg_raw(reg::mode2, build_mode2(cfg_));
        const error r2 = write_reg_raw(reg::inpmux, build_inpmux(cfg_));
        settle_remaining_.store(cfg_.settle_samples, std::memory_order_relaxed);

        if (!raw)           { return {raw.err, 0.0F}; }
        if (r1 != error::ok) { return {r1, 0.0F}; }
        if (r2 != error::ok) { return {r2, 0.0F}; }

        const float microvolts =
            counts_to_voltage(raw.value, gain::x1) * 1.0e6F;
        return {error::ok, (microvolts - 122'400.0F) / 420.0F + 25.0F};
    }

    // ------------------------------------------------------------------
    // Async (DMA) reads. Callbacks run in the SPI IRQ context.
    // ------------------------------------------------------------------

    [[nodiscard]] static error read_async(callback_t cb) noexcept
    {
        return begin_async(async_mode::one_shot, cb);
    }

    [[nodiscard]] static error start_streaming(callback_t cb) noexcept
        requires (Opt.spi_drdy_gated)
    {
        return begin_async(async_mode::streaming, cb);
    }

    static void stop_streaming() noexcept
    {
        stop_requested_.store(true, std::memory_order_release);
    }

    [[nodiscard]] static bool async_active() noexcept
    {
        return mode_.load(std::memory_order_acquire) != async_mode::idle;
    }

    [[nodiscard]] static bool streaming() noexcept
    {
        return mode_.load(std::memory_order_acquire) == async_mode::streaming;
    }

    [[nodiscard]] static error wait_idle(uint32_t timeout_ms) noexcept
    {
        const uint32_t start = now_ms();

        while (async_active())
        {
            if (now_ms() - start >= timeout_ms) { return error::timeout; }
        }
        return error::ok;
    }

    // ------------------------------------------------------------------
    // Calibration (blocking; wait for DRDY with a rate-dependent timeout)
    // ------------------------------------------------------------------
    [[nodiscard]] static error self_offset_calibration() noexcept
    {
        return calibrate(command::sfocal1);
    }

    [[nodiscard]] static error system_offset_calibration() noexcept
    {
        return calibrate(command::syocal1);
    }

    [[nodiscard]] static error system_gain_calibration() noexcept
    {
        return calibrate(command::sygcal1);
    }

    // ------------------------------------------------------------------
    // GPIO (ADS1262: GPIO0..3). GPIODIR: 1 = input, 0 = output.
    // Connecting a GPIO disconnects the analog input sharing the pin.
    // ------------------------------------------------------------------
    enum class gpio_direction : uint8_t { output = 0, input = 1 };

    [[nodiscard]] static error gpio_configure(uint8_t pin_mask,
                                              gpio_direction dir) noexcept
    {
        if ((pin_mask & ~ads1262_constants::gpio_mask) != 0U)
        {
            return error::invalid_argument;
        }
        if (const error e = guard_idle(); e != error::ok) { return e; }

        uint8_t con = 0;
        if (const error e = read_reg_raw(reg::gpiocon, con); e != error::ok) { return e; }
        if (const error e = write_reg_raw(
                reg::gpiocon, static_cast<uint8_t>(con | pin_mask));
            e != error::ok)
        {
            return e;
        }

        uint8_t d = 0;
        if (const error e = read_reg_raw(reg::gpiodir, d); e != error::ok) { return e; }

        d = (dir == gpio_direction::input)
                ? static_cast<uint8_t>(d | pin_mask)
                : static_cast<uint8_t>(d & ~pin_mask);
        return write_reg_raw(reg::gpiodir, d);
    }

    [[nodiscard]] static error gpio_write(uint8_t pin_mask, bool level) noexcept
    {
        if ((pin_mask & ~ads1262_constants::gpio_mask) != 0U)
        {
            return error::invalid_argument;
        }
        if (const error e = guard_idle(); e != error::ok) { return e; }

        uint8_t dat = 0;
        if (const error e = read_reg_raw(reg::gpiodat, dat); e != error::ok) { return e; }

        dat = level ? static_cast<uint8_t>(dat | pin_mask)
                    : static_cast<uint8_t>(dat & ~pin_mask);
        return write_reg_raw(reg::gpiodat, dat);
    }

    [[nodiscard]] static result<uint8_t> gpio_read() noexcept
    {
        return read_register(reg::gpiodat);
    }

    // ------------------------------------------------------------------
    // Raw register access
    // ------------------------------------------------------------------
    [[nodiscard]] static error write_register(reg r, uint8_t value) noexcept
    {
        if (const error e = guard_idle(); e != error::ok) { return e; }
        return write_reg_raw(r, value);
    }

    [[nodiscard]] static result<uint8_t> read_register(reg r) noexcept
    {
        if (const error e = guard_idle(); e != error::ok) { return {e, 0}; }

        uint8_t v = 0;
        const error e = read_reg_raw(r, v);
        return {e, v};
    }

    // ------------------------------------------------------------------
    // Conversion helpers / diagnostics
    // ------------------------------------------------------------------
    [[nodiscard]] static float counts_to_voltage(int32_t counts) noexcept
    {
        return counts_to_voltage(counts, cfg_.g);
    }

    [[nodiscard]] static float vref() noexcept { return vref_; }

    [[nodiscard]] static const config& current_config() noexcept { return cfg_; }

    [[nodiscard]] static statistics stats() noexcept
    {
        return {samples_.load(std::memory_order_relaxed),
                crc_errors_.load(std::memory_order_relaxed),
                transfer_errors_.load(std::memory_order_relaxed)};
    }

    static void reset_statistics() noexcept
    {
        samples_.store(0, std::memory_order_relaxed);
        crc_errors_.store(0, std::memory_order_relaxed);
        transfer_errors_.store(0, std::memory_order_relaxed);
        last_crc_ok_.store(true, std::memory_order_relaxed);
    }

    [[nodiscard]] static bool last_read_crc_ok() noexcept
    {
        return last_crc_ok_.load(std::memory_order_relaxed);
    }
    
    struct mismatch_info { uint8_t reg; uint8_t expected; uint8_t actual; };
[[nodiscard]] static mismatch_info last_mismatch() noexcept { return mismatch_; }

    [[nodiscard]] static constexpr std::string_view library_version() noexcept
    {
        return ads1262_constants::version;
    }

private:
    // ------------------------------------------------------------------
    // Configuration helpers
    // ------------------------------------------------------------------
    static inline mismatch_info mismatch_{};
    
    [[nodiscard]] static constexpr bool valid(const config& c) noexcept
    {
        if (c.use_start_pin && !has_start) { return false; }
        if (c.vref_volts < 0.0F)           { return false; }
        if (!(c.vref_volts > 0.0F) && !nominal_vref(c.ref)) { return false; }
        return true;
    }

    [[nodiscard]] static constexpr float resolve_vref(const config& c) noexcept
    {
        return (c.vref_volts > 0.0F) ? c.vref_volts : *nominal_vref(c.ref);
    }

    [[nodiscard]] static constexpr uint8_t build_mode1(const config& c) noexcept
    {
        return static_cast<uint8_t>(static_cast<uint8_t>(c.flt) << 5);
    }

    [[nodiscard]] static constexpr uint8_t build_mode2(const config& c) noexcept
    {
        return static_cast<uint8_t>(
            (static_cast<uint8_t>(c.dr) & 0x0FU) |
            ((static_cast<uint8_t>(c.g) & 0x07U) << 4) |
            (c.pga_bypass ? 0x80U : 0x00U));
    }

    [[nodiscard]] static constexpr uint8_t build_inpmux(const config& c) noexcept
    {
        return static_cast<uint8_t>(
            (static_cast<uint8_t>(c.positive_input) << 4) |
            static_cast<uint8_t>(c.negative_input));
    }

    [[nodiscard]] static float counts_to_voltage(int32_t counts, gain g) noexcept
    {
        return static_cast<float>(counts) *
               (vref_ / static_cast<float>(gain_factor(g))) * lsb_scale;
    }

    struct reg_write
    {
        reg     r;
        uint8_t value;
        bool    verify;
    };

    // Writes the whole register set and verifies it by readback.
    [[nodiscard]] static error apply_configuration() noexcept
    {
        if constexpr (has_start) { StartPin::reset(); }
        converting_ = false;

        const std::array<reg_write, 12> sequence{{
            {reg::refmux,    static_cast<uint8_t>(cfg_.ref), true},
            {reg::power, static_cast<uint8_t>(cfg_.internal_reference ? 0x01U : 0x00U), true},
            {reg::interface, static_cast<uint8_t>(ads1262_constants::crc_mode::crc), true},
            {reg::mode0,     0x00U, true},
            {reg::mode1,     build_mode1(cfg_), true},
            {reg::mode2,     build_mode2(cfg_), true},
            {reg::inpmux,    build_inpmux(cfg_), true},
            {reg::ofcal0,    0x00U, false},
            {reg::ofcal1,    0x00U, false},
            {reg::ofcal2,    0x00U, false},
            {reg::fscal0,    0x00U, false},
            {reg::fscal1,    0x00U, false},
        }};

        for (const reg_write& w : sequence)
        {
            if (const error e = write_reg_raw(w.r, w.value); e != error::ok)
            {
                return e;
            }
        }

        if (const error e = write_reg_raw(reg::fscal2, 0x40U); e != error::ok)
        {
            return e;
        }

        for (const reg_write& w : sequence)
        {
            if (!w.verify) { continue; }

            uint8_t v = 0;
            if (const error e = read_reg_raw(w.r, v); e != error::ok) { return e; }
            if (v != w.value)
        {
            mismatch_ = {static_cast<uint8_t>(w.r), w.value, v};
            return error::register_mismatch;
        }
        }


        settle_remaining_.store(cfg_.settle_samples, std::memory_order_relaxed);
        return error::ok;
    }

    [[nodiscard]] static error commit_mode2(const config& next) noexcept
    {
        if (const error e = write_reg_raw(reg::mode2, build_mode2(next));
            e != error::ok)
        {
            return e;
        }

        cfg_ = next;
        settle_remaining_.store(cfg_.settle_samples, std::memory_order_relaxed);
        return error::ok;
    }

    // ------------------------------------------------------------------
    // State guards
    // ------------------------------------------------------------------
    [[nodiscard]] static error guard_idle() noexcept
    {
        if (!initialized_) { return error::not_initialized; }
        if (mode_.load(std::memory_order_acquire) != async_mode::idle ||
            SpiMaster::busy())
        {
            return error::busy;
        }
        return error::ok;
    }

    [[nodiscard]] static error start_conversion_raw() noexcept
    {
        if (converting_) { return error::ok; }

        error e = error::ok;

        if constexpr (has_start)
        {
            if (cfg_.use_start_pin)
            {
                StartPin::set();
                clock::delay_us(1U);
            }
            else
            {
                e = command_raw(command::start);
            }
        }
        else
        {
            e = command_raw(command::start);
        }

        if (e != error::ok) { return e; }

        converting_ = true;
        settle_remaining_.store(cfg_.settle_samples, std::memory_order_relaxed);
        return error::ok;
    }

    [[nodiscard]] static bool consume_settle() noexcept
    {
        const uint8_t v = settle_remaining_.load(std::memory_order_relaxed);
        if (v == 0U) { return false; }

        settle_remaining_.store(static_cast<uint8_t>(v - 1U),
                                std::memory_order_relaxed);
        return v > 1U;
    }

    // ------------------------------------------------------------------
    // Low-level SPI (chip-select framing, no guards)
    // ------------------------------------------------------------------
    template <class Pin>
    static void init_pin() noexcept
    {
        if constexpr (requires { Pin::init(); }) { Pin::init(); }
    }

    [[nodiscard]] static error spi_write(std::span<const uint8_t> tx) noexcept
    {
        CsPin::reset();
        clock::delay_us(Opt.cs_setup_us);
        const bool ok = SpiMaster::transmit(tx);
        CsPin::set();
        clock::delay_us(inter_command_us);

        if (!ok)
        {
            transfer_errors_.fetch_add(1U, std::memory_order_relaxed);
            return error::spi_failure;
        }
        return error::ok;
    }

    [[nodiscard]] static error spi_transfer(std::span<const uint8_t> tx,
                                            std::span<uint8_t> rx) noexcept
    {
        CsPin::reset();
        clock::delay_us(Opt.cs_setup_us);
        const bool ok = SpiMaster::transmit_receive(tx, rx);
        CsPin::set();
        clock::delay_us(inter_command_us);

        if (!ok)
        {
            transfer_errors_.fetch_add(1U, std::memory_order_relaxed);
            return error::spi_failure;
        }
        return error::ok;
    }

    [[nodiscard]] static error command_raw(command c) noexcept
    {
        const uint8_t b = static_cast<uint8_t>(c);
        return spi_write(std::span<const uint8_t>{&b, 1});
    }

    [[nodiscard]] static error write_reg_raw(reg r, uint8_t value) noexcept
    {
        const std::array<uint8_t, 3> tx{
            static_cast<uint8_t>(static_cast<uint8_t>(command::wreg) |
                                 (static_cast<uint8_t>(r) & 0x1FU)),
            0x00, value};
        return spi_write(tx);
        
    }

    [[nodiscard]] static error read_reg_raw(reg r, uint8_t& out) noexcept
    {
        const std::array<uint8_t, 4> tx{
            static_cast<uint8_t>(static_cast<uint8_t>(command::rreg) |
                                 (static_cast<uint8_t>(r) & 0x1FU)),
            0x00, 0x00, 0x00};
        std::array<uint8_t, 4> rx{};

        if (const error e = spi_transfer(tx, rx); e != error::ok) { return e; }

        out = rx[2];
        return error::ok;
    }

    // ------------------------------------------------------------------
    // Timing
    // ------------------------------------------------------------------
    [[nodiscard]] static uint32_t drdy_timeout_ms() noexcept
    {
        return conversion_period_ms(cfg_.dr) * 8U + 100U;
    }

    [[nodiscard]] static uint32_t now_ms() noexcept
    {
        return static_cast<uint32_t>(clock::millis().count());
    }

    [[nodiscard]] static bool wait_drdy(uint32_t timeout_ms) noexcept
    {
        const uint32_t start = now_ms();

        while (!is_data_ready())
        {
            if (now_ms() - start >= timeout_ms) { return false; }
        }
        return true;
    }


    // ------------------------------------------------------------------
    // Blocking read path
    // ------------------------------------------------------------------
    struct decoded
    {
        int32_t code;
        bool    crc_ok;
    };

    [[nodiscard]] static decoded
    decode(const std::array<uint8_t, xfer_len>& f) noexcept
    {
        const std::span<const uint8_t> data{f.data() + 1, 4};

        const uint32_t raw = (uint32_t{f[1]} << 24) | (uint32_t{f[2]} << 16) |
                             (uint32_t{f[3]} << 8)  |  uint32_t{f[4]};

        return {static_cast<int32_t>(raw),
                f[5] == ads1262_constants::ads_crc(data)};
    }

       static constexpr std::array<uint8_t, xfer_len> rdata_frame{
       static_cast<uint8_t>(command::rdata), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};       

    [[nodiscard]] static error read_frame_blocking(sample& out) noexcept
    {
        if (!wait_drdy(drdy_timeout_ms())) { return error::timeout; }

        for (uint8_t attempt = 0; attempt < max_read_attempts; ++attempt)
        {
            std::array<uint8_t, xfer_len> rx{};

            if (const error e = spi_transfer(rdata_frame, rx); e != error::ok)
            {
                return e;
            }

            const decoded d = decode(rx);
            out.code = d.code;

            if (d.crc_ok)
            {
                out.status = read_status::ok;
                last_crc_ok_.store(true, std::memory_order_relaxed);
                samples_.fetch_add(1U, std::memory_order_relaxed);
                return error::ok;
            }

            crc_errors_.fetch_add(1U, std::memory_order_relaxed);
        }

        last_crc_ok_.store(false, std::memory_order_relaxed);
        out.status = read_status::crc_error;
        return error::crc_failure;
    }

    [[nodiscard]] static result<int32_t> read_raw_unchecked() noexcept
    {
        if (const error e = start_conversion_raw(); e != error::ok) { return {e, 0}; }

        sample s{};
        do
        {
            if (const error e = read_frame_blocking(s); e != error::ok)
            {
                return {e, s.code};
            }
        } while (consume_settle());

        return {error::ok, s.code};
    }
    
[[nodiscard]] static error calibrate(command c) noexcept
{
    if (const error e = guard_idle(); e != error::ok) { return e; }
    if (const error e = command_raw(c); e != error::ok) { return e; }

    return wait_drdy(conversion_period_ms(cfg_.dr) * 1000U)
               ? error::ok
               : error::timeout;
}

[[nodiscard]] static error check_ofcal(int32_t max_abs_code) noexcept
{
    uint8_t b0 = 0, b1 = 0, b2 = 0;
    if (auto e = read_reg_raw(reg::ofcal0, b0); e != error::ok) { return e; }
    if (auto e = read_reg_raw(reg::ofcal1, b1); e != error::ok) { return e; }
    if (auto e = read_reg_raw(reg::ofcal2, b2); e != error::ok) { return e; }

    int32_t v = (int32_t{b2} << 16) | (int32_t{b1} << 8) | b0;
    if (v & 0x800000) { v -= 0x1000000; }          
    if (v > max_abs_code || v < -max_abs_code)
    {
        (void)write_reg_raw(reg::ofcal0, 0);     
        (void)write_reg_raw(reg::ofcal1, 0);
        (void)write_reg_raw(reg::ofcal2, 0);
        return error::register_mismatch;
    }
    return error::ok;
}

    // ------------------------------------------------------------------
    // Async (DMA) read path
    // ------------------------------------------------------------------
    [[nodiscard]] static error release(error e) noexcept
    {
        mode_.store(async_mode::idle, std::memory_order_release);
        return e;
    }

    [[nodiscard]] static error begin_async(async_mode mode, callback_t cb) noexcept
    {
        if (cb == nullptr) { return error::invalid_argument; }
        if (!initialized_) { return error::not_initialized; }

        async_mode expected = async_mode::idle;
        if (!mode_.compare_exchange_strong(expected, mode,
                                           std::memory_order_acq_rel,
                                           std::memory_order_acquire))
        {
            return error::busy;
        }

        if (SpiMaster::busy()) { return release(error::busy); }

        if (const error e = start_conversion_raw(); e != error::ok)
        {
            return release(e);
        }

        if constexpr (!Opt.spi_drdy_gated)
        {
            if (!is_data_ready()) { return release(error::not_ready); }
        }

        cb_ = cb;
        stop_requested_.store(false, std::memory_order_relaxed);

        std::atomic_signal_fence(std::memory_order_seq_cst);

        if (const error e = start_transfer(); e != error::ok)
        {
            cb_ = nullptr;
            return release(e);
        }

        return error::ok;
    }

    [[nodiscard]] static error start_transfer() noexcept
    {
        CsPin::reset();
        clock::delay_us(Opt.cs_setup_us);

        if (!SpiMaster::transmit_receive_dma(tx_frame_, rx_frame_,
                                             &on_spi_complete))
        {
            CsPin::set();
            transfer_errors_.fetch_add(1U, std::memory_order_relaxed);
            return error::spi_failure;
        }

        return error::ok;
    }

    static void on_spi_complete(bool success) noexcept
    {
        CsPin::set();

        sample s{};

        if (!success)
        {
            s.status = read_status::transfer_error;
            transfer_errors_.fetch_add(1U, std::memory_order_relaxed);
        }
        else
        {
            const decoded d = decode(rx_frame_);
            s.code = d.code;

            if (d.crc_ok)
            {
                last_crc_ok_.store(true, std::memory_order_relaxed);
                samples_.fetch_add(1U, std::memory_order_relaxed);
                s.status = consume_settle() ? read_status::settling
                                            : read_status::ok;
            }
            else
            {
                last_crc_ok_.store(false, std::memory_order_relaxed);
                crc_errors_.fetch_add(1U, std::memory_order_relaxed);
                s.status = read_status::crc_error;
            }
        }

        finish(s);
    }

    static void finish(const sample& s) noexcept
    {
        const bool keep_going =
            mode_.load(std::memory_order_acquire) == async_mode::streaming &&
            !stop_requested_.load(std::memory_order_acquire) &&
            s.status != read_status::transfer_error;

        const callback_t cb = cb_;
        const bool restart_failed =
            keep_going && (start_transfer() != error::ok);

        if (!keep_going || restart_failed)
        {
            cb_ = nullptr;
            mode_.store(async_mode::idle, std::memory_order_release);
        }

        if (cb != nullptr)
        {
            cb(s);

            if (restart_failed)
            {
                cb(sample{.code = 0, .status = read_status::transfer_error});
            }
        }
    }

    // ------------------------------------------------------------------
    // State
    // ------------------------------------------------------------------
    static inline config cfg_{};
    static inline bool   initialized_ = false;
    static inline bool   converting_  = false;
    static inline float  vref_        = 2.5F;

    static constinit inline std::atomic<uint8_t>    settle_remaining_{0};
    static constinit inline std::atomic<async_mode> mode_{async_mode::idle};
    static constinit inline std::atomic<bool>       stop_requested_{false};
    static inline callback_t                        cb_ = nullptr;

    static constinit inline std::atomic<uint32_t> samples_{0};
    static constinit inline std::atomic<uint32_t> crc_errors_{0};
    static constinit inline std::atomic<uint32_t> transfer_errors_{0};
    static constinit inline std::atomic<bool>     last_crc_ok_{true};
    alignas(4) static inline std::array<uint8_t, xfer_len> tx_frame_ = rdata_frame;
    alignas(4) static inline std::array<uint8_t, xfer_len> rx_frame_{};
};

} // namespace stm32u3::drivers