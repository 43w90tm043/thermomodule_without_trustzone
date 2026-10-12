#pragma once

#if defined(SWO_DEVICE_HEADER)
#  include SWO_DEVICE_HEADER
#endif

#include <concepts>
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cmath>
#include <string_view>
#include <type_traits>

namespace stm32u3::swo {

namespace hw {
    [[nodiscard]] inline unsigned num_comparators() noexcept
    {
        return (DWT->CTRL >> 28) & 0xFU; 
    }
}
enum class WatchMode : uint32_t { PcMatch = 0x4, Read = 0x5, Write = 0x6, ReadWrite = 0x7 };

inline void enable_debug_monitor_exception() noexcept
{
#if defined(CoreDebug_DEMCR_MON_EN_Msk)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_MON_EN_Msk;
#endif
}

template <uint8_t CompIndex>
requires (CompIndex < 4U)
class Watchpoint
{

    struct Regs { volatile uint32_t* comp; volatile uint32_t* function; };

    static inline Regs regs() noexcept
    {
        constexpr uint32_t base = DWT_BASE + 0x20u + (16u * CompIndex);
        return Regs{
            reinterpret_cast<volatile uint32_t*>(base + 0x00u),
            reinterpret_cast<volatile uint32_t*>(base + 0x08u)
        };
    }

public:
  
    static inline void configure(const volatile void* address, WatchMode mode) noexcept
    {
        Regs r = regs();
        *r.comp = reinterpret_cast<uint32_t>(address);
        *r.function = static_cast<uint32_t>(mode); 
    }

    static inline void disable() noexcept
    {
        *regs().function = 0U;
    }

    [[nodiscard]] static inline bool check_and_clear_match() noexcept
    {
        return (*regs().function & (1UL << 24)) != 0U;
    }
};

template <uint8_t PortNo>
requires (PortNo < 32U)
class Stimulus
{
    // ITM provides stimulus ports 0..31.

    static inline bool wait_port() noexcept
    {
        if (!ready()) return false;
        
        uint32_t timeout = 5000;
        while (ITM->PORT[PortNo].u32 == 0UL) {
            if (--timeout == 0 || !ready()) return false;
            __asm volatile("nop");
        }
        return true;
    }

public:
    [[nodiscard]] static inline bool ready() noexcept
    {
        return ((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0U) &&
               ((ITM->TER & (1UL << PortNo)) != 0U);
    }

    static inline void put8(uint8_t byte) noexcept
    {
        if (wait_port()) ITM->PORT[PortNo].u8 = byte;
    }

    static inline void put16(uint16_t half) noexcept
    {
        if (wait_port()) ITM->PORT[PortNo].u16 = half;
    }

    static inline void put32(uint32_t word) noexcept
    {
        if (wait_port()) ITM->PORT[PortNo].u32 = word;
    }

    static inline void write(const void* data, std::size_t len) noexcept
    {
        if (!ready()) return;
        auto* p = static_cast<const uint8_t*>(data);

        while (len >= 4U)
        {
            if (!wait_port()) return;
            uint32_t w;
            std::memcpy(&w, p, 4U);
            ITM->PORT[PortNo].u32 = w;
            p += 4U; len -= 4U;
        }
        if (len >= 2U)
        {
            if (!wait_port()) return;
            uint16_t h;
            std::memcpy(&h, p, 2U);
            ITM->PORT[PortNo].u16 = h;
            p += 2U; len -= 2U;
        }
        if (len == 1U)
        {
            if (!wait_port()) return;
            ITM->PORT[PortNo].u8 = *p;
        }
    }
};

[[gnu::weak]] inline void swo_lock() noexcept {}
[[gnu::weak]] inline void swo_unlock() noexcept {}

struct LockGuard
{
    LockGuard() noexcept { swo_lock(); }
    ~LockGuard() noexcept { swo_unlock(); }
    LockGuard(const LockGuard&) = delete;
    LockGuard& operator=(const LockGuard&) = delete;
};

namespace format {

template <std::unsigned_integral T>
inline unsigned utoa(T value, char* buf) noexcept
{
    char tmp[20];
    unsigned n = 0;
    do { tmp[n++] = static_cast<char>('0' + (value % 10)); value /= 10; } while (value != 0);
    for (unsigned i = 0; i < n; ++i) buf[i] = tmp[n - 1 - i];
    return n;
}

template <std::signed_integral T>
inline unsigned itoa(T value, char* buf) noexcept
{
    using U = std::make_unsigned_t<T>;
    unsigned n = 0;
    U uval;
    if (value < 0) { buf[0] = '-'; n = 1; uval = static_cast<U>(-(value + 1)) + 1U; }
    else           { uval = static_cast<U>(value); }
    n += utoa(uval, buf + n);
    return n;
}

inline unsigned utoa_hex(uint32_t value, char* buf, bool upper = false) noexcept
{
    const char* digits = upper ? "0123456789ABCDEF" : "0123456789abcdef";
    char tmp[8];
    unsigned n = 0;
    do { tmp[n++] = digits[value & 0xF]; value >>= 4; } while (value != 0);
    for (unsigned i = 0; i < n; ++i) buf[i] = tmp[n - 1 - i];
    return n;
}

inline unsigned ftoa(float value, char* buf, unsigned buf_len, unsigned decimals = 6) noexcept
{
    unsigned n = 0;
    auto emit = [&](char c) noexcept { if (n < buf_len) buf[n] = c; ++n; };

    if (std::isnan(value)) { for (char c : std::string_view{"nan"}) emit(c); return n; }
    if (value < 0.0f) { emit('-'); value = -value; }
    if (std::isinf(value) || value >= 4294967296.0f) { for (char c : std::string_view{"inf"}) emit(c); return n; }

    auto ipart = static_cast<uint32_t>(value);
    float frac = value - static_cast<float>(ipart);

    char ibuf[10];
    unsigned ilen = utoa(ipart, ibuf);
    for (unsigned i = 0; i < ilen; ++i) emit(ibuf[i]);

    emit('.');
    for (unsigned i = 0; i < decimals; ++i)
    {
        frac *= 10.0f;
        auto d = static_cast<unsigned>(frac);
        emit(static_cast<char>('0' + d));
        frac -= static_cast<float>(d);
    }
    return n;
}

} // namespace format

struct HexTag { uint32_t value; bool upper; };
[[nodiscard]] inline HexTag hex(uint32_t v, bool upper = true) noexcept { return HexTag{v, upper}; }

template <uint8_t PortNo>
class OStream
{
    using Port = Stimulus<PortNo>;

public:
    inline OStream& operator<<(char c) noexcept { Port::put8(static_cast<uint8_t>(c)); return *this; }

    inline OStream& operator<<(const char* s) noexcept
    {
        LockGuard lg;
        Port::write(s, std::strlen(s));
        return *this;
    }

    inline OStream& operator<<(std::string_view s) noexcept
    {
        LockGuard lg;
        Port::write(s.data(), s.size());
        return *this;
    }

    inline OStream& operator<<(bool b) noexcept { return (*this) << (b ? "true" : "false"); }

    template <typename T>
    requires (std::integral<T> && !std::same_as<T, bool> && !std::same_as<T, char>)
    inline OStream& operator<<(T v) noexcept
    {
        char buf[20];
        unsigned n;

        if constexpr (std::signed_integral<T>)
            n = format::itoa(v, buf);
        else
            n = format::utoa(v, buf);

        LockGuard lg;
        Port::write(buf, n);
        return *this;
    }

    inline OStream& operator<<(float v) noexcept
    {
        char buf[32];
        unsigned n = format::ftoa(v, buf, sizeof(buf));
        unsigned to_write = n < sizeof(buf) ? n : sizeof(buf);
        LockGuard lg;
        Port::write(buf, to_write);
        return *this;
    }

    OStream& operator<<(double) = delete;

    inline OStream& operator<<(const void* p) noexcept
    {
        char buf[10];
        unsigned n = format::utoa_hex(reinterpret_cast<uint32_t>(p), buf);
        LockGuard lg;
        Port::write("0x", 2);
        Port::write(buf, n);
        return *this;
    }

    inline OStream& operator<<(HexTag h) noexcept
    {
        char buf[8];
        unsigned n = format::utoa_hex(h.value, buf, h.upper);
        LockGuard lg;
        Port::write(buf, n);
        return *this;
    }
};

template <uint8_t PortNo>
inline OStream<PortNo> stream;

inline OStream<0>& out = stream<0>;

enum class Level : uint8_t { Trace = 0, Debug = 1, Info = 2, Warn = 3, Error = 4, Off = 5 };

template <Level Active, uint8_t PortNo = 0>
class Logger
{
    using Stream = OStream<PortNo>;

    static inline void tag(Level lvl) noexcept
    {
        Stream s;
        switch (lvl)
        {
            case Level::Trace: s << "[TRACE] "; break;
            case Level::Debug: s << "[DEBUG] "; break;
            case Level::Info:  s << "[INFO ] "; break;
            case Level::Warn:  s << "[WARN ] "; break;
            case Level::Error: s << "[ERROR] "; break;
            default: break;
        }
    }

    template <Level Lvl, typename... Args>
    static inline void emit(const char* file, unsigned line, Args&&... args) noexcept
    {
        if constexpr (Lvl >= Active)
        {
            LockGuard lg; // whole line is one logical message; hold across it
            Stream s;
            tag(Lvl);
            (s << ... << args);
            s << " (" << file << ':' << line << ")\r\n";
        }
    }

public:
    template <typename... Args> static inline void trace(const char* f, unsigned l, Args&&... a) noexcept { emit<Level::Trace>(f, l, std::forward<Args>(a)...); }
    template <typename... Args> static inline void debug(const char* f, unsigned l, Args&&... a) noexcept { emit<Level::Debug>(f, l, std::forward<Args>(a)...); }
    template <typename... Args> static inline void info (const char* f, unsigned l, Args&&... a) noexcept { emit<Level::Info> (f, l, std::forward<Args>(a)...); }
    template <typename... Args> static inline void warn (const char* f, unsigned l, Args&&... a) noexcept { emit<Level::Warn> (f, l, std::forward<Args>(a)...); }
    template <typename... Args> static inline void error(const char* f, unsigned l, Args&&... a) noexcept { emit<Level::Error>(f, l, std::forward<Args>(a)...); }
};

#ifndef SWO_ACTIVE_LEVEL
#  define SWO_ACTIVE_LEVEL ::stm32u3::swo::Level::Debug
#endif

template <uint8_t PortNo>
using log_on = Logger<SWO_ACTIVE_LEVEL, PortNo>;

using log = log_on<0>;

#define SWO_TRACE(...) ::stm32u3::swo::log::trace(__FILE__, __LINE__, __VA_ARGS__)
#define SWO_DEBUG(...) ::stm32u3::swo::log::debug(__FILE__, __LINE__, __VA_ARGS__)
#define SWO_INFO(...)  ::stm32u3::swo::log::info(__FILE__, __LINE__, __VA_ARGS__)
#define SWO_WARN(...)  ::stm32u3::swo::log::warn(__FILE__, __LINE__, __VA_ARGS__)
#define SWO_ERROR(...) ::stm32u3::swo::log::error(__FILE__, __LINE__, __VA_ARGS__)

#define SWO_TRACE_P(PORT, ...) ::stm32u3::swo::log_on<PORT>::trace(__FILE__, __LINE__, __VA_ARGS__)
#define SWO_DEBUG_P(PORT, ...) ::stm32u3::swo::log_on<PORT>::debug(__FILE__, __LINE__, __VA_ARGS__)
#define SWO_INFO_P(PORT, ...)  ::stm32u3::swo::log_on<PORT>::info(__FILE__, __LINE__, __VA_ARGS__)
#define SWO_WARN_P(PORT, ...)  ::stm32u3::swo::log_on<PORT>::warn(__FILE__, __LINE__, __VA_ARGS__)
#define SWO_ERROR_P(PORT, ...) ::stm32u3::swo::log_on<PORT>::error(__FILE__, __LINE__, __VA_ARGS__)

template <uint8_t PortNo = 0>
class ScopedTimer
{
public:
    explicit ScopedTimer(const char* label, uint32_t core_hz) noexcept
        : label_(label), core_hz_(core_hz), start_(DWT->CYCCNT) {}

    ~ScopedTimer() noexcept
    {
        const uint32_t elapsed_cycles = DWT->CYCCNT - start_;
        const uint32_t elapsed_us = core_hz_ != 0U
            ? static_cast<uint32_t>((static_cast<uint64_t>(elapsed_cycles) * 1'000'000ULL) / core_hz_)
            : 0U;
        OStream<PortNo> s;
        s << "[TIMER] " << label_ << ": " << elapsed_us << " us\r\n";
    }

    ScopedTimer(const ScopedTimer&) = delete;
    ScopedTimer& operator=(const ScopedTimer&) = delete;

private:
    const char* label_;
    uint32_t core_hz_;
    uint32_t start_;
};

}