#pragma once

#include "ads1262.h"

#include <array>
#include <atomic>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>

namespace stm32u3::sensors
{

// ============================================================================
// Pure math (constexpr, host-testable, no hardware dependencies)
// ============================================================================
namespace pt1000_math
{

// IEC 60751 Callendar-Van Dusen coefficients.
inline constexpr float r0 = 1000.0F;
inline constexpr float a  = 3.9083e-3F;
inline constexpr float b  = -5.775e-7F;
inline constexpr float c  = -4.183e-12F;

// Physical limits of the sensor (-200 .. +850 C) with a small margin.
inline constexpr float r_min_ohms = 180.0F;
inline constexpr float r_max_ohms = 3'920.0F;

inline constexpr uint8_t newton_max_iterations = 8;
inline constexpr float   newton_tolerance_c    = 2.0e-4F;

[[nodiscard]] constexpr float abs_f(float x) noexcept { return (x < 0.0F) ? -x : x; }

// R(T). For T < 0: R0 * (1 + A*T + B*T^2 + C*(T-100)*T^3).
[[nodiscard]] constexpr float resistance(float t) noexcept
{
    if (t >= 0.0F)
    {
        return r0 * (1.0F + t * (a + b * t));
    }
    constexpr float c100 = -100.0F * c;
    return r0 * (1.0F + t * (a + t * (b + t * (c100 + c * t))));
}

// dR/dT.
[[nodiscard]] constexpr float slope(float t) noexcept
{
    if (t >= 0.0F)
    {
        return r0 * (a + 2.0F * b * t);
    }
    constexpr float c300 = -300.0F * c;
    return r0 * (a + t * (2.0F * b + t * (c300 + 4.0F * c * t)));
}

// Inverse R(T) by Newton iteration with a bounded iteration count.
// Precondition: r within [r_min_ohms, r_max_ohms] (slope never approaches 0).
[[nodiscard]] constexpr float temperature(float r) noexcept
{
    float t = (r / r0 - 1.0F) / a; // linear initial guess
    for (uint8_t i = 0; i < newton_max_iterations; ++i)
    {
        const float dt = (resistance(t) - r) / slope(t);
        t -= dt;
        if (abs_f(dt) < newton_tolerance_c) { break; }
    }
    return t;
}

static_assert(resistance(0.0F) == 1000.0F);
static_assert(abs_f(resistance(100.0F) - 1385.055F) < 0.01F);
static_assert(temperature(1000.0F) == 0.0F);
static_assert(abs_f(temperature(resistance(100.0F)) - 100.0F) < 1.0e-2F);
static_assert(abs_f(temperature(resistance(-100.0F)) + 100.0F) < 1.0e-2F);
static_assert(abs_f(temperature(resistance(250.0F)) - 250.0F) < 1.0e-2F);

} // namespace pt1000_math

// ============================================================================
// Public types
// ============================================================================

enum class [[nodiscard]] pt1000_error : uint8_t
{
    ok = 0,
    not_initialized,
    invalid_argument,
    busy,
    not_ready,
    timeout,
    crc_failure,
    adc_failure,   // SPI failure, register mismatch
    sensor_fault,  // open / short circuit, ADC saturated, R out of range
    table_full
};

enum class sample_status : uint8_t
{
    ok,
    settling,        // valid, but taken right after a mux change
    crc_error,
    transfer_error,
    sensor_fault
};

struct reading
{
    static constexpr float nan = std::numeric_limits<float>::quiet_NaN();

    float         celsius            = nan; // calibrated
    float         celsius_raw        = nan; // CVD only, no calibration table
    float         resistance_ohms    = nan;
    sample_status status             = sample_status::transfer_error;
};

template <class T>
struct [[nodiscard]] pt1000_result
{
    pt1000_error err = pt1000_error::ok;
    T            value{};

    [[nodiscard]] constexpr explicit operator bool() const noexcept
    {
        return err == pt1000_error::ok;
    }
};

struct calibration_point
{
    float raw_c; // value measured by this channel (uncalibrated)
    float ref_c; // true value from a reference thermometer
};

// IDAC1 output current (IDACMAG[3:0]).
enum class idac_current : uint8_t
{
    off = 0, ua_50, ua_100, ua_250, ua_500, ua_750,
    ua_1000, ua_1500, ua_2000, ua_2500, ua_3000
};

[[nodiscard]] constexpr float idac_amps(idac_current i) noexcept
{
    constexpr std::array<float, 11> table{
        0.0F, 50.0e-6F, 100.0e-6F, 250.0e-6F, 500.0e-6F, 750.0e-6F,
        1000.0e-6F, 1500.0e-6F, 2000.0e-6F, 2500.0e-6F, 3000.0e-6F};
    return table[static_cast<std::size_t>(i)];
}

struct channel_config
{
    drivers::data_rate dr  = drivers::data_rate::sps_2_5;
    drivers::filter    flt = drivers::filter::sinc4;
    uint8_t settle_samples = 6;
    drivers::input_channel rtd_positive = drivers::input_channel::ain4;
    drivers::input_channel rtd_negative = drivers::input_channel::ain5;
    drivers::reference     rref_reference = drivers::reference::external_ain2_ain3;

    drivers::input_channel idac_pin = drivers::input_channel::ain6;
    idac_current           idac     = idac_current::ua_500;

    float rref_min_ohms = 100.0F;
    float rref_max_ohms = 4'800.0F;

    std::optional<calibration_point> factory_point =
       calibration_point{-1.201484F, 0.0F};
};

// ============================================================================
// ADC backend concept
// ============================================================================
template <class A>
concept adc_backend = requires(const drivers::config& cfg,
                               drivers::ads1262_constants::reg r,
                               uint8_t v,
                               drivers::input_channel ch,
                               uint32_t ms,
                               typename A::callback_t cb)
{
    { A::init(cfg) }                noexcept -> std::same_as<drivers::error>;
    { A::write_register(r, v) }     noexcept -> std::same_as<drivers::error>;
    { A::set_input_mux(ch, ch) }    noexcept -> std::same_as<drivers::error>;
    { A::self_offset_calibration() } noexcept -> std::same_as<drivers::error>;
    { A::start_conversion() }       noexcept -> std::same_as<drivers::error>;
    { A::stop_conversion() }        noexcept -> std::same_as<drivers::error>;
    { A::read_raw() }               noexcept -> std::same_as<drivers::result<int32_t>>;
    { A::read_voltage() }           noexcept -> std::same_as<drivers::result<float>>;
    { A::read_async(cb) }           noexcept -> std::same_as<drivers::error>;
    { A::wait_idle(ms) }            noexcept -> std::same_as<drivers::error>;
    { A::async_active() }           noexcept -> std::same_as<bool>;
    { A::stats() }                  noexcept -> std::same_as<drivers::statistics>;
    A::stop_streaming();
};

// ============================================================================
// Channel
// ============================================================================
template <adc_backend Adc, std::size_t MaxCalibrationPoints = 16>
class pt1000_channel
{
    static_assert(MaxCalibrationPoints >= 2 && MaxCalibrationPoints <= 255);

    using reg = drivers::ads1262_constants::reg;

public:
    using callback_t = void (*)(const reading&) noexcept;

    pt1000_channel() = delete;

    // ------------------------------------------------------------------
    // Lifecycle
    // ------------------------------------------------------------------
    [[nodiscard]] static pt1000_error setup(const channel_config& cc = {}) noexcept
    {
        const auto rref_pins = rref_inputs(cc.rref_reference);
        if (!valid(cc) || !rref_pins) { return pt1000_error::invalid_argument; }

        initialized_ = false;
        const drivers::config cfg{
            .dr                 = cc.dr,
            .g                  = drivers::gain::x1,
            .ref                = drivers::reference::internal_2_5v,
            .flt                = cc.flt,
            .internal_reference = true,
            .positive_input     = rref_pins->positive,
            .negative_input     = rref_pins->negative,
            .settle_samples     = cc.settle_samples};   
        if (const auto e = Adc::init(cfg); e != drivers::error::ok)
        {
            return map_error(e);
        }

        const uint8_t idacmux = static_cast<uint8_t>(
            (idac_not_connected << 4) |
            (static_cast<uint8_t>(cc.idac_pin) & 0x0FU));
        const uint8_t idacmag = static_cast<uint8_t>(cc.idac); 
        if (const auto e = Adc::write_register(reg::idacmux, idacmux);
            e != drivers::error::ok) { return map_error(e); }
        if (const auto e = Adc::write_register(reg::idacmag, idacmag);
            e != drivers::error::ok) { return map_error(e); }
        step_ = 4;
        if (const auto e = Adc::start_conversion(); e != drivers::error::ok)
        {
            return map_error(e);
        }
        if (const auto e = Adc::set_input_mux(drivers::input_channel::floating,
                                          drivers::input_channel::floating);   
        e != drivers::error::ok) { return map_error(e); }
        if (const auto e = Adc::self_offset_calibration();
            e != drivers::error::ok) { return map_error(e); }
        if (const auto e = Adc::set_input_mux(rref_pins->positive,
                                              rref_pins->negative);                
        e != drivers::error::ok) { return map_error(e); }
        if (const auto e = Adc::stop_conversion(); e != drivers::error::ok)
        {
            return map_error(e);
        }
        if (const auto e = Adc::start_conversion(); e != drivers::error::ok)
        {
            return map_error(e);
        }
        step_ = 5;
        if (const auto e = measure_rref(cc); e != pt1000_error::ok) { return e; }
        step_ = 6;
        if (const auto e = Adc::write_register(
                reg::refmux, static_cast<uint8_t>(cc.rref_reference));
            e != drivers::error::ok) { return map_error(e); }
        if (const auto e = Adc::set_input_mux(cc.rtd_positive, cc.rtd_negative);
            e != drivers::error::ok) { return map_error(e); }
        if (const auto e = Adc::start_conversion(); e != drivers::error::ok)
        {
            return map_error(e);
        }

        cal_count_ = 0;
        if (cc.factory_point)
        {
            if (const auto e = insert_point(*cc.factory_point);
                e != pt1000_error::ok) { return e; }
        }

        initialized_ = true;
        return pt1000_error::ok;
    }

    [[nodiscard]] static bool initialized() noexcept { return initialized_; }

    // ------------------------------------------------------------------
    // Blocking API
    // ------------------------------------------------------------------
    [[nodiscard]] static pt1000_result<reading> read() noexcept
    {
        if (!initialized_) { return {pt1000_error::not_initialized, {}}; }

        const auto raw = Adc::read_raw();
        if (!raw) { return {map_error(raw.err), {}}; }

        const reading r = convert(raw.value, sample_status::ok);
        return {(r.status == sample_status::sensor_fault)
                    ? pt1000_error::sensor_fault
                    : pt1000_error::ok,
                r};
    }

    [[nodiscard]] static pt1000_result<float> read_celsius() noexcept
    {
        const auto r = read();
        return {r.err, r ? r.value.celsius : reading::nan};
    }

    // ------------------------------------------------------------------
    // Async API (callbacks run in the SPI/DMA IRQ context)
    // ------------------------------------------------------------------

    // One conversion.
    [[nodiscard]] static pt1000_error read_async(callback_t cb) noexcept
    {
        if (cb == nullptr)   { return pt1000_error::invalid_argument; }
        if (!initialized_)   { return pt1000_error::not_initialized; }
        if (Adc::async_active()) { return pt1000_error::busy; }

        callback_.store(cb, std::memory_order_release);
        const auto e = Adc::read_async(&on_adc_sample);
        if (e != drivers::error::ok)
        {
            callback_.store(nullptr, std::memory_order_release);
            return map_error(e);
        }
        return pt1000_error::ok;
    }

    // Continuous stream; only if the ADC driver is DRDY-gated.
    [[nodiscard]] static pt1000_error start_streaming(callback_t cb) noexcept
        requires requires { Adc::start_streaming(std::declval<typename Adc::callback_t>()); }
    {
        if (cb == nullptr)   { return pt1000_error::invalid_argument; }
        if (!initialized_)   { return pt1000_error::not_initialized; }
        if (Adc::async_active()) { return pt1000_error::busy; }

        callback_.store(cb, std::memory_order_release);
        const auto e = Adc::start_streaming(&on_adc_sample);
        if (e != drivers::error::ok)
        {
            callback_.store(nullptr, std::memory_order_release);
            return map_error(e);
        }
        return pt1000_error::ok;
    }

    // Takes effect after the transfer in flight completes.
    static void stop_streaming() noexcept { Adc::stop_streaming(); }

    [[nodiscard]] static bool busy() noexcept { return Adc::async_active(); }

    [[nodiscard]] static pt1000_error wait_idle(uint32_t timeout_ms) noexcept
    {
        return map_error(Adc::wait_idle(timeout_ms));
    }

    [[nodiscard]] static reading convert(int32_t code, sample_status st) noexcept
    {
        reading r{};
        r.status = st;

        if (st == sample_status::crc_error || st == sample_status::transfer_error)
        {
            return r;
        }

        // Full-scale code: open circuit / input over-range.
        if (code == std::numeric_limits<int32_t>::max() ||
            code == std::numeric_limits<int32_t>::min())
        {
            r.status = sample_status::sensor_fault;
            return r;
        }

        constexpr float adc_scale = 1.0F / 2147483648.0F;
        r.resistance_ohms = static_cast<float>(code) * adc_scale * rref_ohms_;

        if (!(r.resistance_ohms >= pt1000_math::r_min_ohms &&
              r.resistance_ohms <= pt1000_math::r_max_ohms))
        {
            r.status = sample_status::sensor_fault;
            return r;
        }

        r.celsius_raw = pt1000_math::temperature(r.resistance_ohms);
        r.celsius     = apply_calibration(r.celsius_raw);
        return r;
    }

    // ------------------------------------------------------------------
    // Calibration table (piecewise linear, sorted by raw value)
    // ------------------------------------------------------------------
    [[nodiscard]] static pt1000_error add_calibration_point(float raw_measured_c,
                                                            float reference_c) noexcept
    {
        if (!std::isfinite(raw_measured_c) || !std::isfinite(reference_c))
        {
            return pt1000_error::invalid_argument;
        }
        if (Adc::async_active()) { return pt1000_error::busy; }

        return insert_point({raw_measured_c, reference_c});
    }

    [[nodiscard]] static pt1000_error clear_calibration() noexcept
    {
        if (Adc::async_active()) { return pt1000_error::busy; }
        cal_count_ = 0;
        return pt1000_error::ok;
    }

    [[nodiscard]] static std::span<const calibration_point> calibration_points() noexcept
    {
        return {cal_table_.data(), cal_count_};
    }

    [[nodiscard]] static constexpr std::size_t max_calibration_points() noexcept
    {
        return MaxCalibrationPoints;
    }

    // ------------------------------------------------------------------
    // Diagnostics
    // ------------------------------------------------------------------
    [[nodiscard]] static float rref_ohms() noexcept { return rref_ohms_; }

     [[nodiscard]] static uint8_t last_setup_step() noexcept { return step_; }
    [[nodiscard]] static drivers::error last_adc_error() noexcept
    {
        return last_adc_error_;
    }

    [[nodiscard]] static drivers::statistics adc_statistics() noexcept
    {
        return Adc::stats();
    }

private:
    static inline uint8_t step_ = 0;
    static constexpr uint8_t idac_not_connected     = 0x0BU;
    static constexpr float   duplicate_epsilon_c    = 0.01F;
    static constexpr float   min_segment_span_c     = 1.0e-6F;
    static constexpr uint8_t rref_read_attempts     = 3;

    struct input_pair
    {
        drivers::input_channel positive;
        drivers::input_channel negative;
    };

    [[nodiscard]] static constexpr std::optional<input_pair>
    rref_inputs(drivers::reference r) noexcept
    {
        using drivers::input_channel;
        using drivers::reference;
        switch (r)
        {
        case reference::external_ain0_ain1: return input_pair{input_channel::ain0, input_channel::ain1};
        case reference::external_ain2_ain3: return input_pair{input_channel::ain2, input_channel::ain3};
        case reference::external_ain4_ain5: return input_pair{input_channel::ain4, input_channel::ain5};
        case reference::internal_2_5v:
        case reference::internal_avdd_avss: break;
        }
        return std::nullopt;
    }

    [[nodiscard]] static constexpr bool valid(const channel_config& c) noexcept
    {
        return c.idac != idac_current::off &&
               static_cast<uint8_t>(c.idac_pin) <=
                   static_cast<uint8_t>(drivers::input_channel::aincom) &&
               c.rtd_positive != c.rtd_negative &&
               c.rref_min_ohms > 0.0F && c.rref_min_ohms < c.rref_max_ohms;
    }

    [[nodiscard]] static pt1000_error map_error(drivers::error e) noexcept
    {
        using E = drivers::error;
        if (e != E::ok) { last_adc_error_ = e; }
        switch (e)
        {
        case E::ok:                return pt1000_error::ok;
        case E::not_initialized:   return pt1000_error::not_initialized;
        case E::invalid_config:
        case E::invalid_argument:  return pt1000_error::invalid_argument;
        case E::busy:              return pt1000_error::busy;
        case E::not_ready:         return pt1000_error::not_ready;
        case E::timeout:           return pt1000_error::timeout;
        case E::crc_failure:       return pt1000_error::crc_failure;
        case E::spi_failure:
        case E::register_mismatch: return pt1000_error::adc_failure;
        }
        return pt1000_error::adc_failure;
    }

    [[nodiscard]] static constexpr sample_status map_status(drivers::read_status s) noexcept
    {
        switch (s)
        {
        case drivers::read_status::ok:             return sample_status::ok;
        case drivers::read_status::settling:       return sample_status::settling;
        case drivers::read_status::crc_error:      return sample_status::crc_error;
        case drivers::read_status::transfer_error: return sample_status::transfer_error;
        }
        return sample_status::transfer_error;
    }

    [[nodiscard]] static pt1000_error measure_rref(const channel_config& cc) noexcept
    {
        drivers::result<float> v{};
        for (uint8_t i = 0; i < rref_read_attempts; ++i)
        {
            v = Adc::read_voltage();
            if (v || v.err != drivers::error::crc_failure) { break; }
        }
        if (!v) { return map_error(v.err); }

        const float r = v.value / idac_amps(cc.idac);
        rref_ohms_ = r;
        if (!(r >= cc.rref_min_ohms && r <= cc.rref_max_ohms))
        {
            return pt1000_error::sensor_fault;
        }

        rref_ohms_ = r;
        return pt1000_error::ok;
    }

    // IRQ context.
    static void on_adc_sample(const drivers::sample& s) noexcept
    {
        const callback_t cb = callback_.load(std::memory_order_acquire);
        if (cb == nullptr) { return; }
        cb(convert(s.code, map_status(s.status)));
    }

    // ---- Calibration internals -------------------------------------------
    [[nodiscard]] static pt1000_error insert_point(const calibration_point& p) noexcept
    {
        for (std::size_t i = 0; i < cal_count_; ++i)
        {
            if (pt1000_math::abs_f(cal_table_[i].raw_c - p.raw_c) < duplicate_epsilon_c)
            {
                cal_table_[i].ref_c = p.ref_c;
                return pt1000_error::ok;
            }
        }

        if (cal_count_ >= MaxCalibrationPoints) { return pt1000_error::table_full; }

        std::size_t pos = 0;
        while (pos < cal_count_ && cal_table_[pos].raw_c < p.raw_c) { ++pos; }

        for (std::size_t i = cal_count_; i > pos; --i)
        {
            cal_table_[i] = cal_table_[i - 1];
        }
        cal_table_[pos] = p;
        ++cal_count_;
        return pt1000_error::ok;
    }

    [[nodiscard]] static float interpolate(float raw_c,
                                           const calibration_point& p0,
                                           const calibration_point& p1) noexcept
    {
        const float span = p1.raw_c - p0.raw_c;
        if (pt1000_math::abs_f(span) < min_segment_span_c) { return p0.ref_c; }
        return p0.ref_c + ((raw_c - p0.raw_c) / span) * (p1.ref_c - p0.ref_c);
    }

    [[nodiscard]] static float apply_calibration(float raw_c) noexcept
    {
        const std::size_t n = cal_count_;
        if (n == 0) { return raw_c; }
        if (n == 1) { return raw_c + (cal_table_[0].ref_c - cal_table_[0].raw_c); }

        std::size_t i = 0;
        while (i + 2 < n && raw_c > cal_table_[i + 1].raw_c) { ++i; }
        return interpolate(raw_c, cal_table_[i], cal_table_[i + 1]);
    }

    // ---- State ------------------------------------------------------------
    static inline drivers::error last_adc_error_ = drivers::error::ok;
    static inline bool        initialized_ = false;
    static inline float       rref_ohms_   = 0.0F;
    static inline std::size_t cal_count_   = 0;
    static inline std::array<calibration_point, MaxCalibrationPoints> cal_table_{};

    static constinit inline std::atomic<callback_t> callback_{nullptr};
    static_assert(std::atomic<callback_t>::is_always_lock_free);
};

} // namespace stm32u3::sensors