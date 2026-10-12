#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <type_traits>

#include "stm32u3xx.h"

static_assert(__cplusplus >= 202002L, "stm32u3::fdcan requires C++20");

namespace stm32u3::fdcan {

// ============================================================================
// Utilities
// ============================================================================

template <class E>
    requires std::is_enum_v<E>
[[nodiscard]] constexpr std::underlying_type_t<E> to_underlying(E e) noexcept
{
    return static_cast<std::underlying_type_t<E>>(e);
}

namespace detail {

// Volatile-safe read-modify-write helpers (`|=` on volatile is deprecated in C++20).
inline void set_bits(volatile uint32_t& r, uint32_t m) noexcept   { r = r | m; }
inline void clear_bits(volatile uint32_t& r, uint32_t m) noexcept { r = r & ~m; }
inline void modify(volatile uint32_t& r, uint32_t clear, uint32_t set) noexcept
{
    r = (r & ~clear) | set;
}

} // namespace detail

// ============================================================================
// Errors
// ============================================================================

enum class [[nodiscard]] Error : uint8_t
{
    Ok = 0,
    Timeout,
    InvalidLength,
    InvalidId,
    InvalidTiming,
    InvalidFilter,
    TooManyFilters,
    TxFifoFull,
    HardwareFault
};

// Bus / controller events, bit-compatible with FDCAN_IR.
enum class BusEvent : uint32_t
{
    None             = 0U,
    Rx0Full          = FDCAN_IR_RF0F,
    Rx0Lost          = FDCAN_IR_RF0L,
    Rx1Full          = FDCAN_IR_RF1F,
    Rx1Lost          = FDCAN_IR_RF1L,
    BusOff           = FDCAN_IR_BO,
    ErrorPassive     = FDCAN_IR_EP,
    Warning          = FDCAN_IR_EW,
    RamAccessFailure = FDCAN_IR_MRAF,
    ProtocolArb      = FDCAN_IR_PEA,
    ProtocolData     = FDCAN_IR_PED
};

[[nodiscard]] constexpr BusEvent operator|(BusEvent a, BusEvent b) noexcept
{
    return BusEvent{to_underlying(a) | to_underlying(b)};
}

[[nodiscard]] constexpr BusEvent operator&(BusEvent a, BusEvent b) noexcept
{
    return BusEvent{to_underlying(a) & to_underlying(b)};
}

[[nodiscard]] constexpr bool has(BusEvent set, BusEvent flag) noexcept
{
    return (to_underlying(set) & to_underlying(flag)) != 0U;
}

// ============================================================================
// Message RAM layout (STM32 FDCAN fixed layout, 0x350 bytes)
// ============================================================================

namespace hw {

inline constexpr std::size_t std_filter_count = 28; // unused, kept for layout
inline constexpr std::size_t ext_filter_count = 8;
inline constexpr std::size_t rx_fifo_count    = 3;  // per FIFO
inline constexpr std::size_t tx_event_count   = 3;
inline constexpr std::size_t tx_fifo_count    = 3;
inline constexpr std::size_t ram_size_bytes   = 0x350;

struct StdFilterElement { volatile uint32_t f0; };
struct ExtFilterElement { volatile uint32_t f0; volatile uint32_t f1; };
struct TxEventElement   { volatile uint32_t e0; volatile uint32_t e1; };

struct RxElement
{
    volatile uint32_t r0;
    volatile uint32_t r1;
    volatile uint32_t data[16];
};

struct TxElement
{
    volatile uint32_t t0;
    volatile uint32_t t1;
    volatile uint32_t data[16];
};

struct MessageRam
{
    StdFilterElement std_filter[std_filter_count]; // 0x000
    ExtFilterElement ext_filter[ext_filter_count]; // 0x070
    RxElement        rx_fifo0[rx_fifo_count];      // 0x0B0
    RxElement        rx_fifo1[rx_fifo_count];      // 0x188
    TxEventElement   tx_event[tx_event_count];     // 0x260
    TxElement        tx_buffer[tx_fifo_count];     // 0x278
};

static_assert(sizeof(RxElement) == 72 && sizeof(TxElement) == 72);
static_assert(sizeof(ExtFilterElement) == 8 && sizeof(TxEventElement) == 8);
static_assert(offsetof(MessageRam, ext_filter) == 0x070);
static_assert(offsetof(MessageRam, rx_fifo0)   == 0x0B0);
static_assert(offsetof(MessageRam, rx_fifo1)   == 0x188);
static_assert(offsetof(MessageRam, tx_event)   == 0x260);
static_assert(offsetof(MessageRam, tx_buffer)  == 0x278);
static_assert(sizeof(MessageRam) == ram_size_bytes);

// Rx / Tx element bit fields.
namespace elem {
inline constexpr uint32_t ESI  = 0x80000000UL; // R0/T0
inline constexpr uint32_t XTD  = 0x40000000UL; // R0/T0
inline constexpr uint32_t ID   = 0x1FFFFFFFUL; // R0/T0
inline constexpr uint32_t TS   = 0x0000FFFFUL; // R1
inline constexpr uint32_t DLC  = 0x000F0000UL; // R1/T1
inline constexpr uint32_t BRS  = 0x00100000UL; // R1/T1
inline constexpr uint32_t FDF  = 0x00200000UL; // R1/T1
inline constexpr uint32_t FIDX = 0x7F000000UL; // R1
inline constexpr uint32_t ANMF = 0x80000000UL; // R1
inline constexpr uint32_t DLC_POS  = 16U;
inline constexpr uint32_t FIDX_POS = 24U;
} // namespace elem

} // namespace hw

// ============================================================================
// Frame
// ============================================================================

inline constexpr std::size_t   max_payload = 64;
inline constexpr uint32_t      max_extended_id = 0x1FFFFFFFUL;

inline constexpr std::array<uint8_t, 16> dlc_to_length{
    0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

// Valid CAN FD payload length -> DLC. nullopt for illegal lengths.
[[nodiscard]] constexpr std::optional<uint8_t>
dlc_from_length(std::size_t length) noexcept
{
    for (std::size_t dlc = 0; dlc < dlc_to_length.size(); ++dlc)
    {
        if (dlc_to_length[dlc] == length)
        {
            return static_cast<uint8_t>(dlc);
        }
    }
    return std::nullopt;
}

enum class BitRateSwitch : uint8_t { Off = 0, On = 1 };
enum class ErrorStateIndicator : uint8_t { Active = 0, Passive = 1 };

struct Frame
{
    uint32_t id = 0; // 29-bit extended identifier
    std::array<uint8_t, max_payload> data{};
    uint8_t length = 0; // 0..8, 12, 16, 20, 24, 32, 48, 64

    BitRateSwitch       brs = BitRateSwitch::On;
    ErrorStateIndicator esi = ErrorStateIndicator::Active;

    // Rx metadata (ignored on Tx).
    uint16_t timestamp             = 0;
    uint8_t  filter_index          = 0;
    bool     accepted_non_matching = false;

    [[nodiscard]] constexpr std::span<const uint8_t> payload() const noexcept
    {
        return {data.data(), length};
    }
};

static_assert(std::is_trivially_copyable_v<Frame>);

[[nodiscard]] constexpr std::optional<Frame>
make_frame(uint32_t id,
           std::span<const uint8_t> payload,
           BitRateSwitch brs = BitRateSwitch::On) noexcept
{
    if (id > max_extended_id || !dlc_from_length(payload.size()))
    {
        return std::nullopt;
    }

    Frame f{};
    f.id     = id;
    f.brs    = brs;
    f.length = static_cast<uint8_t>(payload.size());
    std::ranges::copy(payload, f.data.begin());
    return f;
}

// ============================================================================
// Modes
// ============================================================================

enum class Mode : uint8_t
{
    Normal,
    RestrictedOperation,
    BusMonitoring,
    InternalLoopback,
    ExternalLoopback
};

// ============================================================================
// Filters (extended)
// ============================================================================
// EFT: 00 range | 01 dual | 10 filter+mask | 11 range without EIDM
// EFEC: 000 off | 001 FIFO0 | 010 FIFO1 | 011 reject | 100 high prio
//       101 high prio + FIFO0 | 110 high prio + FIFO1

enum class FilterType : uint32_t
{
    Range = 0UL, Dual = 1UL, Mask = 2UL, RangeNoEidm = 3UL
};

enum class FilterAction : uint32_t
{
    Disabled = 0UL, StoreFifo0 = 1UL, StoreFifo1 = 2UL, Reject = 3UL,
    HighPriority = 4UL, HighFifo0 = 5UL, HighFifo1 = 6UL
};

struct ExtendedFilter
{
    uint32_t     id1    = 0;
    uint32_t     id2    = 0;
    FilterType   type   = FilterType::Mask;
    FilterAction action = FilterAction::Disabled;

    // Accept frames where (id & mask) == (id_ref & mask).
    [[nodiscard]] static constexpr ExtendedFilter
    mask(uint32_t id_ref, uint32_t mask, FilterAction a) noexcept
    {
        return {id_ref, mask, FilterType::Mask, a};
    }

    // Accept id1 or id2.
    [[nodiscard]] static constexpr ExtendedFilter
    dual(uint32_t id1, uint32_t id2, FilterAction a) noexcept
    {
        return {id1, id2, FilterType::Dual, a};
    }

    [[nodiscard]] static constexpr ExtendedFilter
    exact(uint32_t id, FilterAction a) noexcept
    {
        return dual(id, id, a);
    }

    // Accept id in [from, to].
    [[nodiscard]] static constexpr ExtendedFilter
    range(uint32_t from, uint32_t to, FilterAction a) noexcept
    {
        return {from, to, FilterType::Range, a};
    }

    [[nodiscard]] constexpr bool is_valid() const noexcept
    {
        return id1 <= max_extended_id && id2 <= max_extended_id;
    }

    [[nodiscard]] constexpr uint32_t word0() const noexcept
    {
        return (to_underlying(action) << 29U) | (id1 & max_extended_id);
    }

    [[nodiscard]] constexpr uint32_t word1() const noexcept
    {
        return (to_underlying(type) << 30U) | (id2 & max_extended_id);
    }
};

// What to do with extended frames that match no filter.
enum class NonMatching : uint32_t { Fifo0 = 0UL, Fifo1 = 1UL, Reject = 2UL };

// ============================================================================
// Bit timing
// ============================================================================

struct BitTiming
{
    uint32_t prescaler = 1;
    uint32_t sjw       = 1;
    uint32_t seg1      = 8; // includes propagation segment, in time quanta
    uint32_t seg2      = 3;
};

struct BitTimingLimits
{
    uint32_t max_prescaler;
    uint32_t max_seg1;
    uint32_t max_seg2;
    uint32_t max_sjw;
};

inline constexpr BitTimingLimits nominal_limits{512, 256, 128, 128};
inline constexpr BitTimingLimits data_limits{32, 32, 16, 16};

[[nodiscard]] constexpr bool
is_valid(const BitTiming& t, const BitTimingLimits& l) noexcept
{
    return t.prescaler >= 1 && t.prescaler <= l.max_prescaler &&
           t.seg1 >= 1 && t.seg1 <= l.max_seg1 &&
           t.seg2 >= 1 && t.seg2 <= l.max_seg2 &&
           t.sjw >= 1 && t.sjw <= l.max_sjw && t.sjw <= t.seg2;
}

// Finds exact timing (smallest prescaler = best resolution).
// sample_permille: e.g. 800 == 80.0 %. Usable in constant expressions.
[[nodiscard]] constexpr std::optional<BitTiming>
solve_bit_timing(uint32_t kernel_hz,
                 uint32_t bitrate,
                 uint32_t sample_permille,
                 const BitTimingLimits& lim) noexcept
{
    if (kernel_hz == 0 || bitrate == 0 ||
        sample_permille == 0 || sample_permille >= 1000)
    {
        return std::nullopt;
    }

    for (uint32_t p = 1; p <= lim.max_prescaler; ++p)
    {
        const uint64_t div = static_cast<uint64_t>(p) * bitrate;

        if (kernel_hz % div != 0)
        {
            continue;
        }

        const auto tq = static_cast<uint32_t>(kernel_hz / div);

        if (tq < 3 || tq > 1 + lim.max_seg1 + lim.max_seg2)
        {
            continue;
        }

        // Sample point = (1 + seg1) / tq.
        uint32_t s1 = (tq * sample_permille + 500U) / 1000U;
        s1 = (s1 > 0U) ? s1 - 1U : 0U;

        if (s1 < 1U)      { s1 = 1U; }
        if (s1 > tq - 2U) { s1 = tq - 2U; }
        if (tq - 1U - s1 > lim.max_seg2) { s1 = tq - 1U - lim.max_seg2; }
        if (s1 > lim.max_seg1) { continue; }

        const uint32_t s2  = tq - 1U - s1;
        const uint32_t sjw = std::min(s2, lim.max_sjw);

        const BitTiming t{p, sjw, s1, s2};

        if (is_valid(t, lim))
        {
            return t;
        }
    }

    return std::nullopt;
}

struct BitTimingConfig
{
    BitTiming nominal{1, 2, 8, 3};
    BitTiming data{1, 2, 8, 3};
    bool    tdc        = true;
    uint8_t tdc_filter = 0;
    uint8_t tdc_offset = 0;
};

// ============================================================================
// Config
// ============================================================================

struct Config
{
    BitTimingConfig timing{};
    Mode            mode = Mode::Normal;

    bool automatic_retransmission = true;
    bool transmit_pause           = false; // CCCR.TXP

    NonMatching non_matching = NonMatching::Fifo0;

    // Must outlive init(); filters are copied into message RAM.
    std::span<const ExtendedFilter> filters{};

    uint8_t irq_priority = 5;
};

// ============================================================================
// Lock-free SPSC queue (producer: ISR, consumer: thread)
// ============================================================================

template <class T, std::size_t N>
    requires(std::has_single_bit(N) && std::is_trivially_copyable_v<T>)
class SpscQueue
{
    static_assert(std::atomic<uint32_t>::is_always_lock_free);
    static constexpr uint32_t mask_ = static_cast<uint32_t>(N - 1);

public:
    // Producer side. `fill` writes directly into the slot (no extra copy).
    template <class F>
        requires std::invocable<F, T&>
    bool try_produce(F&& fill) noexcept
    {
        const uint32_t h = head_.load(std::memory_order_relaxed);
        const uint32_t t = tail_.load(std::memory_order_acquire);

        if (h - t == N)
        {
            return false;
        }

        fill(buf_[h & mask_]);
        head_.store(h + 1U, std::memory_order_release);
        return true;
    }

    // Consumer side.
    [[nodiscard]] std::optional<T> try_pop() noexcept
    {
        const uint32_t t = tail_.load(std::memory_order_relaxed);
        const uint32_t h = head_.load(std::memory_order_acquire);

        if (t == h)
        {
            return std::nullopt;
        }

        T out = buf_[t & mask_];
        tail_.store(t + 1U, std::memory_order_release);
        return out;
    }

    [[nodiscard]] bool empty() const noexcept
    {
        return head_.load(std::memory_order_acquire) ==
               tail_.load(std::memory_order_acquire);
    }

private:
    std::array<T, N>      buf_{};
    std::atomic<uint32_t> head_{0};
    std::atomic<uint32_t> tail_{0};
};

// ============================================================================
// Peripheral instance traits
// ============================================================================

template <class T>
concept Instance = requires {
    { T::peripheral_base } -> std::convertible_to<uintptr_t>;
    { T::ram_base }        -> std::convertible_to<uintptr_t>;
    { T::irq }             -> std::convertible_to<IRQn_Type>;
    { T::enable_clock() } noexcept;
};

struct Fdcan1Instance
{
    static constexpr uintptr_t peripheral_base =
        static_cast<uintptr_t>(FDCAN1_BASE);
    static constexpr uintptr_t ram_base =
        static_cast<uintptr_t>(SRAMCAN_BASE);
    static constexpr IRQn_Type irq = FDCAN1_IT0_IRQn;

    static void enable_clock() noexcept
    {
        // System clock as FDCAN kernel clock.
        detail::clear_bits(RCC->CCIPR1, RCC_CCIPR1_FDCANSEL);
        detail::set_bits(RCC->APB1ENR2, RCC_APB1ENR2_FDCANEN);
        (void)RCC->APB1ENR2; // read-back: ensure clock is up before register access
    }
};

// ============================================================================
// Driver
// ============================================================================

struct Stats
{
    uint32_t rx_frames;          // delivered to queue
    uint32_t rx_queue_overruns;  // software queue was full
    uint32_t rx_rejected_format; // not extended-FD frames
};

template <Instance Inst, std::size_t RxQueueDepth = 8>
class Driver
{
public:
    Driver() = delete;

    // ------------------------------------------------------------------
    // Initialization
    // ------------------------------------------------------------------
    [[nodiscard]] static Error init(const Config& cfg = {}) noexcept
    {
        if (!is_valid(cfg.timing.nominal, nominal_limits) ||
            !is_valid(cfg.timing.data, data_limits))
        {
            return Error::InvalidTiming;
        }

        const uint32_t tdco = tdc_offset(cfg.timing);
        if (cfg.timing.tdc && tdco > 127U)
        {
            return Error::InvalidTiming;
        }

        if (cfg.filters.size() > hw::ext_filter_count)
        {
            return Error::TooManyFilters;
        }

        for (const auto& f : cfg.filters)
        {
            if (!f.is_valid())
            {
                return Error::InvalidFilter;
            }
        }

        Inst::enable_clock();
        auto& r = regs();

        // Enter initialization mode, unlock configuration.
        detail::set_bits(r.CCCR, FDCAN_CCCR_INIT);
        if (!wait_for([&] { return (r.CCCR & FDCAN_CCCR_INIT) != 0U; }))
        {
            return Error::Timeout;
        }
        detail::set_bits(r.CCCR, FDCAN_CCCR_CCE);

        // CAN FD + BRS are mandatory in this driver.
        detail::set_bits(r.CCCR, FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE);

        cfg.automatic_retransmission
            ? detail::clear_bits(r.CCCR, FDCAN_CCCR_DAR)
            : detail::set_bits(r.CCCR, FDCAN_CCCR_DAR);

        cfg.transmit_pause
            ? detail::set_bits(r.CCCR, FDCAN_CCCR_TXP)
            : detail::clear_bits(r.CCCR, FDCAN_CCCR_TXP);

        configure_mode(cfg.mode);

        // Maximum kernel clock (divider /1). Note: shared by all FDCAN instances.
        FDCAN_CONFIG->CKDIV = 0U;

        r.NBTP = build_nbtp(cfg.timing.nominal);
        r.DBTP = build_dbtp(cfg.timing);
        r.TDCR = (tdco << FDCAN_TDCR_TDCO_Pos) |
                 (static_cast<uint32_t>(cfg.timing.tdc_filter)
                  << FDCAN_TDCR_TDCF_Pos);

        // Message RAM: clear, then load filters.
        auto* words = reinterpret_cast<volatile uint32_t*>(Inst::ram_base);
        for (std::size_t i = 0; i < hw::ram_size_bytes / sizeof(uint32_t); ++i)
        {
            words[i] = 0U;
        }

        for (std::size_t i = 0; i < cfg.filters.size(); ++i)
        {
            ram().ext_filter[i].f0 = cfg.filters[i].word0();
            ram().ext_filter[i].f1 = cfg.filters[i].word1();
        }

        // No standard filters; standard + remote frames are rejected in HW.
        r.RXGFC =
            (0UL << FDCAN_RXGFC_LSS_Pos) |
            (static_cast<uint32_t>(cfg.filters.size()) << FDCAN_RXGFC_LSE_Pos) |
            (to_underlying(NonMatching::Reject) << FDCAN_RXGFC_ANFS_Pos) |
            (to_underlying(cfg.non_matching) << FDCAN_RXGFC_ANFE_Pos) |
            FDCAN_RXGFC_RRFS | FDCAN_RXGFC_RRFE; // FIFO blocking mode (F0OM/F1OM = 0)

        // Tx FIFO operation (not queue).
        detail::clear_bits(r.TXBC, FDCAN_TXBC_TFQM);

        // Interrupts.
        r.IR  = irq_clear_mask;
        r.IE  = irq_enable_mask;
        r.ILS = 0U;
        detail::set_bits(r.ILE, FDCAN_ILE_EINT0);

        // Leave initialization mode.
        detail::clear_bits(r.CCCR, FDCAN_CCCR_INIT);
        if (!wait_for([&] { return (r.CCCR & FDCAN_CCCR_INIT) == 0U; }))
        {
            return Error::Timeout;
        }

        NVIC_SetPriority(Inst::irq, cfg.irq_priority);
        NVIC_EnableIRQ(Inst::irq);
        return Error::Ok;
    }

    // Call after BusOff event to rejoin the bus (waits 129 x 11 recessive bits).
    static void restart_after_bus_off() noexcept
    {
        detail::clear_bits(regs().CCCR, FDCAN_CCCR_INIT);
    }

    // ------------------------------------------------------------------
    // Rx (consumer side, thread context)
    // ------------------------------------------------------------------
    [[nodiscard]] static std::optional<Frame> receive() noexcept
    {
        return rx_queue_.try_pop();
    }

    [[nodiscard]] static bool rx_available() noexcept
    {
        return !rx_queue_.empty();
    }

    // Returns and clears accumulated bus/controller events.
    [[nodiscard]] static BusEvent take_events() noexcept
    {
        return static_cast<BusEvent>(
            events_.exchange(0U, std::memory_order_acq_rel));
    }

    [[nodiscard]] static Stats stats() noexcept
    {
        return {
            stats_.rx_frames.load(std::memory_order_relaxed),
            stats_.rx_queue_overruns.load(std::memory_order_relaxed),
            stats_.rx_rejected_format.load(std::memory_order_relaxed)};
    }

    // ------------------------------------------------------------------
    // Tx (not reentrant: call from a single context or guard externally)
    // ------------------------------------------------------------------
    [[nodiscard]] static Error send(const Frame& msg) noexcept
    {
        const auto dlc = dlc_from_length(msg.length);
        if (!dlc)
        {
            return Error::InvalidLength;
        }
        if (msg.id > max_extended_id)
        {
            return Error::InvalidId;
        }

        auto& r = regs();
        const uint32_t txfqs = r.TXFQS;

        if ((txfqs & FDCAN_TXFQS_TFQF_Msk) != 0U)
        {
            return Error::TxFifoFull;
        }

        const uint32_t idx =
            (txfqs & FDCAN_TXFQS_TFQPI_Msk) >> FDCAN_TXFQS_TFQPI_Pos;

        if (idx >= hw::tx_fifo_count)
        {
            return Error::HardwareFault;
        }

        auto& e = ram().tx_buffer[idx];

        uint32_t t0 = hw::elem::XTD | (msg.id & hw::elem::ID);
        if (msg.esi == ErrorStateIndicator::Passive)
        {
            t0 |= hw::elem::ESI;
        }

        uint32_t t1 = (static_cast<uint32_t>(*dlc) << hw::elem::DLC_POS) |
                      hw::elem::FDF;
        if (msg.brs == BitRateSwitch::On)
        {
            t1 |= hw::elem::BRS;
        }

        e.t0 = t0;
        e.t1 = t1;

        const std::size_t words = (msg.length + 3U) / 4U;
        for (std::size_t w = 0; w < words; ++w)
        {
            uint32_t v = 0U;
            for (std::size_t b = 0; b < 4U && (w * 4U + b) < msg.length; ++b)
            {
                v |= static_cast<uint32_t>(msg.data[w * 4U + b]) << (8U * b);
            }
            e.data[w] = v;
        }

        __DMB();
        r.TXBAR = (1UL << idx);
        return Error::Ok;
    }

    // ------------------------------------------------------------------
    // IRQ (producer side) — call from FDCANx_IT0_IRQHandler
    // ------------------------------------------------------------------
    static void irq_handler() noexcept
    {
        auto& r = regs();
        const uint32_t ir = r.IR;

        // Clear first, drain afterwards: a frame arriving in-between
        // re-raises the interrupt instead of being lost.
        r.IR = ir & irq_clear_mask;

        const uint32_t ev = ir & event_mask;
        if (ev != 0U)
        {
            events_.fetch_or(ev, std::memory_order_relaxed);
        }

        drain<0>();
        drain<1>();
    }

private:
    // ------------------------------------------------------------------
    // Constants
    // ------------------------------------------------------------------
    static constexpr uint32_t event_mask =
        FDCAN_IR_RF0F | FDCAN_IR_RF0L | FDCAN_IR_RF1F | FDCAN_IR_RF1L |
        FDCAN_IR_BO   | FDCAN_IR_EP   | FDCAN_IR_EW    |
        FDCAN_IR_MRAF | FDCAN_IR_PEA  | FDCAN_IR_PED;

    static constexpr uint32_t irq_clear_mask =
        event_mask | FDCAN_IR_RF0N | FDCAN_IR_RF1N;

    static constexpr uint32_t irq_enable_mask =
        FDCAN_IE_RF0NE | FDCAN_IE_RF0FE | FDCAN_IE_RF0LE |
        FDCAN_IE_RF1NE | FDCAN_IE_RF1FE | FDCAN_IE_RF1LE |
        FDCAN_IE_BOE   | FDCAN_IE_EPE   | FDCAN_IE_EWE   |
        FDCAN_IE_MRAFE | FDCAN_IE_PEAE  | FDCAN_IE_PEDE;

    static constexpr uint32_t sync_spins = 100'000U;

    // ------------------------------------------------------------------
    // Hardware access
    // ------------------------------------------------------------------
    [[nodiscard]] static FDCAN_GlobalTypeDef& regs() noexcept
    {
        return *reinterpret_cast<FDCAN_GlobalTypeDef*>(Inst::peripheral_base);
    }

    [[nodiscard]] static hw::MessageRam& ram() noexcept
    {
        return *reinterpret_cast<hw::MessageRam*>(Inst::ram_base);
    }

    template <class Pred>
    [[nodiscard]] static bool wait_for(Pred&& pred) noexcept
    {
        for (uint32_t i = 0; i < sync_spins; ++i)
        {
            if (pred())
            {
                return true;
            }
            __NOP();
        }
        return false;
    }

    // ------------------------------------------------------------------
    // Register encoding
    // ------------------------------------------------------------------
    [[nodiscard]] static constexpr uint32_t build_nbtp(const BitTiming& t) noexcept
    {
        return ((t.sjw       - 1UL) << FDCAN_NBTP_NSJW_Pos)   |
               ((t.seg1      - 1UL) << FDCAN_NBTP_NTSEG1_Pos) |
               ((t.seg2      - 1UL) << FDCAN_NBTP_NTSEG2_Pos) |
               ((t.prescaler - 1UL) << FDCAN_NBTP_NBRP_Pos);
    }

    [[nodiscard]] static constexpr uint32_t
    build_dbtp(const BitTimingConfig& c) noexcept
    {
        const BitTiming& t = c.data;
        return ((t.sjw       - 1UL) << FDCAN_DBTP_DSJW_Pos)   |
               ((t.seg1      - 1UL) << FDCAN_DBTP_DTSEG1_Pos) |
               ((t.seg2      - 1UL) << FDCAN_DBTP_DTSEG2_Pos) |
               ((t.prescaler - 1UL) << FDCAN_DBTP_DBRP_Pos)   |
               (c.tdc ? FDCAN_DBTP_TDC : 0UL);
    }

    [[nodiscard]] static constexpr uint32_t
    tdc_offset(const BitTimingConfig& c) noexcept
    {
        return (c.tdc_offset != 0U)
                   ? c.tdc_offset
                   : c.data.prescaler * c.data.seg1;
    }

    static void configure_mode(Mode mode) noexcept
    {
        auto& r = regs();

        detail::clear_bits(r.CCCR,
                           FDCAN_CCCR_TEST | FDCAN_CCCR_MON | FDCAN_CCCR_ASM);

        // TEST register is only writable while CCCR.TEST = 1.
        const bool loopback = (mode == Mode::InternalLoopback) ||
                              (mode == Mode::ExternalLoopback);

        detail::set_bits(r.CCCR, FDCAN_CCCR_TEST);
        detail::modify(r.TEST, FDCAN_TEST_LBCK,
                       loopback ? FDCAN_TEST_LBCK : 0U);

        switch (mode)
        {
        case Mode::Normal:
        case Mode::ExternalLoopback:
            break;
        case Mode::RestrictedOperation:
            detail::set_bits(r.CCCR, FDCAN_CCCR_ASM);
            break;
        case Mode::BusMonitoring:
        case Mode::InternalLoopback:
            detail::set_bits(r.CCCR, FDCAN_CCCR_MON);
            break;
        }

        if (!loopback)
        {
            detail::clear_bits(r.CCCR, FDCAN_CCCR_TEST);
        }
    }

    // ------------------------------------------------------------------
    // Rx decoding / draining
    // ------------------------------------------------------------------
    static void decode(const hw::RxElement& e,
                       uint32_t r0,
                       uint32_t r1,
                       Frame& f) noexcept
    {
        const auto dlc = static_cast<uint8_t>(
            (r1 & hw::elem::DLC) >> hw::elem::DLC_POS);

        f.id  = r0 & hw::elem::ID;
        f.esi = (r0 & hw::elem::ESI) ? ErrorStateIndicator::Passive
                                     : ErrorStateIndicator::Active;
        f.brs = (r1 & hw::elem::BRS) ? BitRateSwitch::On
                                     : BitRateSwitch::Off;
        f.length    = dlc_to_length[dlc & 0x0FU];
        f.timestamp = static_cast<uint16_t>(r1 & hw::elem::TS);
        f.filter_index = static_cast<uint8_t>(
            (r1 & hw::elem::FIDX) >> hw::elem::FIDX_POS);
        f.accepted_non_matching = (r1 & hw::elem::ANMF) != 0U;

        for (std::size_t w = 0; w * 4U < f.length; ++w)
        {
            const uint32_t v = e.data[w];
            for (std::size_t b = 0; b < 4U && (w * 4U + b) < f.length; ++b)
            {
                f.data[w * 4U + b] = static_cast<uint8_t>(v >> (8U * b));
            }
        }

        std::fill(f.data.begin() + f.length, f.data.end(), uint8_t{0});
    }

    template <unsigned Fifo>
    static void drain() noexcept
    {
        static_assert(Fifo < 2U);

        auto& r     = regs();
        auto& elems = (Fifo == 0U) ? ram().rx_fifo0 : ram().rx_fifo1;

        for (;;)
        {
            uint32_t level;
            uint32_t idx;

            if constexpr (Fifo == 0U)
            {
                const uint32_t s = r.RXF0S;
                level = s & FDCAN_RXF0S_F0FL_Msk;
                idx   = (s & FDCAN_RXF0S_F0GI_Msk) >> FDCAN_RXF0S_F0GI_Pos;
            }
            else
            {
                const uint32_t s = r.RXF1S;
                level = s & FDCAN_RXF1S_F1FL_Msk;
                idx   = (s & FDCAN_RXF1S_F1GI_Msk) >> FDCAN_RXF1S_F1GI_Pos;
            }

            if (level == 0U)
            {
                return;
            }

            if (idx < hw::rx_fifo_count)
            {
                const hw::RxElement& e = elems[idx];
                const uint32_t r0 = e.r0;
                const uint32_t r1 = e.r1;

                const bool fd_ext = ((r1 & hw::elem::FDF) != 0U) &&
                                    ((r0 & hw::elem::XTD) != 0U);

                if (!fd_ext)
                {
                    stats_.rx_rejected_format.fetch_add(
                        1U, std::memory_order_relaxed);
                }
                else if (rx_queue_.try_produce(
                             [&](Frame& f) { decode(e, r0, r1, f); }))
                {
                    stats_.rx_frames.fetch_add(1U, std::memory_order_relaxed);
                }
                else
                {
                    stats_.rx_queue_overruns.fetch_add(
                        1U, std::memory_order_relaxed);
                }
            }

            __DMB();

            if constexpr (Fifo == 0U) { r.RXF0A = idx; }
            else                      { r.RXF1A = idx; }
        }
    }

    // ------------------------------------------------------------------
    // State
    // ------------------------------------------------------------------
    struct AtomicStats
    {
        std::atomic<uint32_t> rx_frames{0};
        std::atomic<uint32_t> rx_queue_overruns{0};
        std::atomic<uint32_t> rx_rejected_format{0};
    };

    inline static SpscQueue<Frame, RxQueueDepth> rx_queue_{};
    inline static std::atomic<uint32_t>          events_{0};
    inline static AtomicStats                    stats_{};
};

// ============================================================================
// FDCAN1
// ============================================================================

using Fdcan1 = Driver<Fdcan1Instance>;

} // namespace stm32u3::fdcan