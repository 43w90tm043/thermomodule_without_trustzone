#pragma once

#include "octospi.h"
#include "clock.h"

#include <algorithm>
#include <array>
#include <bit>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <type_traits>

namespace stm32u3::qspi::w25q512jv {

// ============================================================================
// Opcodes (W25Q512JV datasheet)
// ============================================================================
namespace opcode {

inline constexpr uint32_t write_enable            = 0x06;
inline constexpr uint32_t write_enable_volatile   = 0x50;
inline constexpr uint32_t write_disable           = 0x04;

inline constexpr uint32_t read_status1            = 0x05;
inline constexpr uint32_t write_status1           = 0x01;
inline constexpr uint32_t read_status2            = 0x35;
inline constexpr uint32_t write_status2           = 0x31;
inline constexpr uint32_t read_status3            = 0x15;
inline constexpr uint32_t write_status3           = 0x11;

inline constexpr uint32_t jedec_id                = 0x9F;

inline constexpr uint32_t read_data_4b               = 0x13;
inline constexpr uint32_t fast_read_4b               = 0x0C;
inline constexpr uint32_t fast_read_quad_output_4b   = 0x6C;
inline constexpr uint32_t fast_read_quad_io_4b       = 0xEC;

inline constexpr uint32_t page_program_4b         = 0x12;
inline constexpr uint32_t quad_page_program_4b    = 0x34;

inline constexpr uint32_t sector_erase_4k_4b      = 0x21;
inline constexpr uint32_t block_erase_64k_4b      = 0xDC;
inline constexpr uint32_t chip_erase              = 0xC7;

inline constexpr uint32_t enable_reset            = 0x66;
inline constexpr uint32_t reset_device            = 0x99;

} // namespace opcode

// ============================================================================
// Status register bits
// ============================================================================
inline constexpr uint8_t sr1_busy = 1u << 0;
inline constexpr uint8_t sr1_wel  = 1u << 1;
inline constexpr uint8_t sr2_qe   = 1u << 1;

// ============================================================================
// JEDEC
// ============================================================================
inline constexpr uint8_t jedec_manufacturer_id = 0xEF;

// -IQ/JQ = 0x4020, -IM/JM = 0x7020 (verify against your part suffix).
inline constexpr std::array<uint16_t, 2> jedec_device_ids{0x4020, 0x7020};

struct jedec_id
{
    uint8_t  manufacturer;
    uint16_t device;
};

// ============================================================================
// Geometry
// ============================================================================
inline constexpr uint32_t page_size_bytes   = 256;
inline constexpr uint32_t sector_size_bytes = 4u * 1024u;
inline constexpr uint32_t block_size_bytes  = 64u * 1024u;
inline constexpr uint32_t total_size_bytes  = 64u * 1024u * 1024u; // 512 Mbit

static_assert(std::has_single_bit(total_size_bytes));

// OCTOSPI DCR1.DEVSIZE: size = 2^(DEVSIZE + 1) bytes  ->  64 MiB = 2^26  ->  25.
inline constexpr uint32_t devsize_field =
    static_cast<uint32_t>(std::countr_zero(total_size_bytes)) - 1U;

static_assert(devsize_field == 25U);

// Clock / CS timing is board-specific: use make_device() to override.
[[nodiscard]] constexpr device_config
make_device(uint32_t clock_prescaler     = 0,
            bool     clock_mode3         = false,
            uint32_t cs_high_time_cycles = 2) noexcept
{
    return device_config{
        .device_size_log2    = devsize_field,
        .cs_high_time_cycles = cs_high_time_cycles,
        .clock_prescaler     = clock_prescaler,
        .clock_mode3         = clock_mode3};
}

inline constexpr device_config device = make_device();

// Type-based handle for qspi::controller (replaces `device_config` NTTP).
// For custom timing:  struct my_dev { static constexpr device_config value = make_device(1); };
struct device_traits
{
    static constexpr device_config value = make_device();
};

// ============================================================================
// Commands
// ============================================================================
namespace detail {

[[nodiscard]] constexpr command simple(uint32_t op) noexcept
{
    return {.instruction = op};
}

[[nodiscard]] constexpr command reg_access(uint32_t op) noexcept
{
    return {.instruction = op, .data_mode = lines::x1};
}

[[nodiscard]] constexpr command addressed(uint32_t op) noexcept
{
    return {.instruction = op,
            .addr_mode   = lines::x1,
            .addr_size   = field_size::_32bit};
}

} // namespace detail

inline constexpr command cmd_write_enable          = detail::simple(opcode::write_enable);
inline constexpr command cmd_write_enable_volatile = detail::simple(opcode::write_enable_volatile);
inline constexpr command cmd_write_disable         = detail::simple(opcode::write_disable);
inline constexpr command cmd_enable_reset          = detail::simple(opcode::enable_reset);
inline constexpr command cmd_reset_device          = detail::simple(opcode::reset_device);
inline constexpr command cmd_chip_erase            = detail::simple(opcode::chip_erase);

inline constexpr command cmd_read_status1  = detail::reg_access(opcode::read_status1);
inline constexpr command cmd_write_status1 = detail::reg_access(opcode::write_status1);
inline constexpr command cmd_read_status2  = detail::reg_access(opcode::read_status2);
inline constexpr command cmd_write_status2 = detail::reg_access(opcode::write_status2);
inline constexpr command cmd_jedec_id      = detail::reg_access(opcode::jedec_id);

// 0x6C: 1-line instruction + 32-bit address, 8 dummy clocks, 4-line data.
// Used for both indirect and memory-mapped reads.
inline constexpr command cmd_fast_read_quad_output_4b{
    .instruction  = opcode::fast_read_quad_output_4b,
    .addr_mode    = lines::x1,
    .addr_size    = field_size::_32bit,
    .dummy_cycles = 8,
    .data_mode    = lines::x4};

// 0xEC: 1-line instruction, 4-line address + mode byte, 4 dummy clocks, 4-line data.
inline constexpr command cmd_fast_read_quad_io_4b{
    .instruction  = opcode::fast_read_quad_io_4b,
    .addr_mode    = lines::x4,
    .addr_size    = field_size::_32bit,
    .alt_mode     = lines::x4,
    .alt_size     = field_size::_8bit,
    .dummy_cycles = 4,
    .data_mode    = lines::x4};

inline constexpr command cmd_page_program_4b{
    .instruction = opcode::page_program_4b,
    .addr_mode   = lines::x1,
    .addr_size   = field_size::_32bit,
    .data_mode   = lines::x1};

inline constexpr command cmd_quad_page_program_4b{
    .instruction = opcode::quad_page_program_4b,
    .addr_mode   = lines::x1,
    .addr_size   = field_size::_32bit,
    .data_mode   = lines::x4};

inline constexpr command cmd_sector_erase_4b    = detail::addressed(opcode::sector_erase_4k_4b);
inline constexpr command cmd_block_erase_64k_4b = detail::addressed(opcode::block_erase_64k_4b);

static_assert(cmd_read_status1.valid() && cmd_jedec_id.valid() &&
              cmd_fast_read_quad_output_4b.valid() &&
              cmd_fast_read_quad_io_4b.valid() &&
              cmd_quad_page_program_4b.valid());

// ---- Mode selection (compile time; enum NTTPs are fine) -------------------
enum class read_mode    : uint8_t { quad_output, quad_io };
enum class program_mode : uint8_t { single, quad };

[[nodiscard]] constexpr const command& read_cmd(read_mode m) noexcept
{
    return (m == read_mode::quad_io) ? cmd_fast_read_quad_io_4b
                                     : cmd_fast_read_quad_output_4b;
}

// Mode byte 0xFF: never enters continuous-read mode.
[[nodiscard]] constexpr uint32_t read_alt_bytes(read_mode m) noexcept
{
    return (m == read_mode::quad_io) ? 0xFFu : 0u;
}

[[nodiscard]] constexpr const command& program_cmd(program_mode m) noexcept
{
    return (m == program_mode::quad) ? cmd_quad_page_program_4b
                                     : cmd_page_program_4b;
}

// ============================================================================
// Errors, options, concepts
// ============================================================================
enum class [[nodiscard]] error : uint8_t
{
    ok = 0,
    invalid_argument,
    out_of_range,
    misaligned,
    crosses_page,
    controller_busy,     // async transfer in flight / controller unavailable
    device_busy,         // flash still programming/erasing (WIP)
    timeout,
    wrong_device,
    write_enable_failed, // WEL did not set
    quad_enable_failed,
    memory_mapped_active
};

// Conservative upper limits; check the datasheet for your speed grade.
struct timeouts_ms
{
    uint32_t page_program_ms = 10;
    uint32_t sector_erase_ms = 1'000;
    uint32_t block_erase_ms  = 3'000;
    uint32_t chip_erase_ms   = 600'000;
    uint32_t status_write_ms = 50;
};

// Timeouts are passed as a type (no class-type NTTP).
// Override:  struct my_to { static constexpr timeouts_ms value{ .page_program_ms = 20 }; };
struct default_timeouts
{
    static constexpr timeouts_ms value{};
};

template <class T>
concept timeouts_traits = requires {
    { T::value } -> std::convertible_to<timeouts_ms>;
};

struct init_options
{
    bool reset_device = true;  // 0x66 + 0x99 before probing
    bool enable_quad  = true;  // set SR2.QE (required for x4 read/program)
};

// Interface of qspi::controller that this driver relies on.
template <class C>
concept flash_controller = requires(const command& cmd) {
    typename C::completion_cb;
    { C::init(static_cast<DLYB_TypeDef*>(nullptr)) } noexcept;
    { C::busy() } noexcept -> std::convertible_to<bool>;
    { C::send(cmd, 0u, 0u) } noexcept -> std::convertible_to<bool>;
    { C::read_blocking(cmd, 0u, 0u, std::span<uint8_t>{}) } noexcept
        -> std::convertible_to<bool>;
    { C::write_blocking(cmd, 0u, 0u, std::span<const uint8_t>{}) } noexcept
        -> std::convertible_to<bool>;
    { C::read_async(cmd, 0u, 0u, std::span<uint8_t>{}, typename C::completion_cb{}) } noexcept
        -> std::convertible_to<bool>;
    { C::write_async(cmd, 0u, 0u, std::span<const uint8_t>{}, typename C::completion_cb{}) } noexcept
        -> std::convertible_to<bool>;
    { C::memory_mapped(cmd, cmd, 0u, 0u) } noexcept -> std::convertible_to<bool>;
    { C::template memory_base<uint8_t>() } noexcept;
    { C::irq_handler() } noexcept;
};

// ============================================================================
// Driver
// ============================================================================
template <flash_controller Controller,
          timeouts_traits  Timeouts = default_timeouts>
class driver
{
public:
    using completion_cb = typename Controller::completion_cb;

    driver() = delete;

    // ------------------------------------------------------------------
    // Bounds
    // ------------------------------------------------------------------
    [[nodiscard]] static constexpr bool
    in_range(uint32_t address, std::size_t length) noexcept
    {
        return address <= total_size_bytes &&
               length <= total_size_bytes - address;
    }

    // ------------------------------------------------------------------
    // Init / reset / identification
    // ------------------------------------------------------------------
    [[nodiscard]] static error
    init(DLYB_TypeDef* dlyb = DLYB_OCTOSPI1,
         const init_options& opt = {}) noexcept
    {
        Controller::init(dlyb);
        s_mapped  = false;
        s_pending = false;

        if (opt.reset_device)
        {
            if (const error e = reset(); e != error::ok) { return e; }
        }

        if (const error e = verify_jedec_id(); e != error::ok) { return e; }

        return opt.enable_quad ? ensure_quad_enabled() : error::ok;
    }

    [[nodiscard]] static error reset() noexcept
    {
        if (const error e = prepare_indirect(); e != error::ok) { return e; }

        if (const error e = do_send(cmd_enable_reset); e != error::ok) { return e; }
        if (const error e = do_send(cmd_reset_device); e != error::ok) { return e; }
        s_pending = false;
        stm32u3::clock::delay_ms(2U);
        return error::ok;
    }

    [[nodiscard]] static std::optional<jedec_id> read_jedec_id() noexcept
    {
        if (prepare_indirect() != error::ok) { return std::nullopt; }

        std::array<uint8_t, 3> id{};
        if (!Controller::read_blocking(cmd_jedec_id, 0, 0, std::span<uint8_t>{id}))
        {
            return std::nullopt;
        }

        return jedec_id{
            id[0],
            static_cast<uint16_t>((static_cast<uint16_t>(id[1]) << 8) | id[2])};
    }

    [[nodiscard]] static error verify_jedec_id() noexcept
    {
        const auto id = read_jedec_id();
        if (!id) { return error::controller_busy; }

        const bool known =
            std::ranges::find(jedec_device_ids, id->device) !=
            jedec_device_ids.end();

        return (id->manufacturer == jedec_manufacturer_id && known)
                   ? error::ok
                   : error::wrong_device;
    }

    [[nodiscard]] static error ensure_quad_enabled() noexcept
    {
        uint8_t sr2 = 0;
        if (const error e = read_reg(cmd_read_status2, sr2); e != error::ok) { return e; }
        if ((sr2 & sr2_qe) != 0U) { return error::ok; }

        if (const error e = write_enable(); e != error::ok) { return e; }

        sr2 = static_cast<uint8_t>(sr2 | sr2_qe);
        if (!Controller::write_blocking(cmd_write_status2, 0, 0,
                                        std::span<const uint8_t>{&sr2, 1}))
        {
            return error::invalid_argument;
        }

        if (const error e = wait_ready(Timeouts::value.status_write_ms); e != error::ok) { return e; }
        if (const error e = read_reg(cmd_read_status2, sr2); e != error::ok) { return e; }

        return ((sr2 & sr2_qe) != 0U) ? error::ok : error::quad_enable_failed;
    }

    // ------------------------------------------------------------------
    // Status
    // ------------------------------------------------------------------
    [[nodiscard]] static std::optional<uint8_t> status1() noexcept
    {
        uint8_t v = 0;
        if (read_reg(cmd_read_status1, v) != error::ok) { return std::nullopt; }
        return v;
    }

    [[nodiscard]] static std::optional<uint8_t> status2() noexcept
    {
        uint8_t v = 0;
        if (read_reg(cmd_read_status2, v) != error::ok) { return std::nullopt; }
        return v;
    }

    // True when the flash is not busy (WIP = 0).
    [[nodiscard]] static bool ready() noexcept
    {
        const auto sr = status1();
        return sr && (*sr & sr1_busy) == 0U;
    }

    // Polls WIP until clear or timeout. Call after async program / unwaited erase.
    [[nodiscard]] static error
    wait_ready(uint32_t timeout_ms = Timeouts::value.page_program_ms) noexcept
    {
        const uint32_t start = now_ms();

        for (;;)
        {
            uint8_t sr = 0;
            if (const error e = read_reg(cmd_read_status1, sr); e != error::ok)
            {
                return e;
            }

            if ((sr & sr1_busy) == 0U)
            {
                s_pending = false;
                return error::ok;
            }

            if (now_ms() - start >= timeout_ms)
            {
                return error::timeout;
            }
        }
    }

    // ------------------------------------------------------------------
    // Read
    // ------------------------------------------------------------------
    template <read_mode Mode = read_mode::quad_output>
    [[nodiscard]] static error
    read(uint32_t address, std::span<uint8_t> dst) noexcept
    {
        if (!in_range(address, dst.size())) { return error::out_of_range; }
        if (dst.empty()) { return error::ok; }

        if (const error e = prepare_indirect(); e != error::ok) { return e; }
        if (const error e = ensure_idle(); e != error::ok) { return e; }

        return Controller::read_blocking(read_cmd(Mode), address,
                                         read_alt_bytes(Mode), dst)
                   ? error::ok
                   : error::invalid_argument;
    }

    // Completion callback runs in IRQ context.
    template <read_mode Mode = read_mode::quad_output>
    [[nodiscard]] static error
    read_async(uint32_t address,
               std::span<uint8_t> dst,
               completion_cb cb) noexcept
    {
        if (dst.empty()) { return error::invalid_argument; }
        if (!in_range(address, dst.size())) { return error::out_of_range; }

        if (const error e = prepare_indirect(); e != error::ok) { return e; }
        if (const error e = ensure_idle(); e != error::ok) { return e; }

        return Controller::read_async(read_cmd(Mode), address,
                                      read_alt_bytes(Mode), dst, cb)
                   ? error::ok
                   : error::controller_busy;
    }

    // ------------------------------------------------------------------
    // Program
    // ------------------------------------------------------------------

    // Blocking, any length: split on page boundaries, waits for completion.
    template <program_mode Mode = program_mode::single>
    [[nodiscard]] static error
    program(uint32_t address, std::span<const uint8_t> src) noexcept
    {
        if (!in_range(address, src.size())) { return error::out_of_range; }
        if (src.empty()) { return error::ok; }

        if (const error e = wait_ready(Timeouts::value.chip_erase_ms); e != error::ok) { return e; }

        while (!src.empty())
        {
            const uint32_t room = page_size_bytes - (address % page_size_bytes);
            const auto     n    = static_cast<uint32_t>(
                std::min<std::size_t>(src.size(), room));

            if (const error e = write_enable(); e != error::ok) { return e; }

            if (!Controller::write_blocking(program_cmd(Mode), address, 0,
                                            src.first(n)))
            {
                return error::invalid_argument;
            }
            s_pending = true;

            if (const error e = wait_ready(Timeouts::value.page_program_ms); e != error::ok)
            {
                return e;
            }

            address += n;
            src = src.subspan(n);
        }

        return error::ok;
    }

    template <program_mode Mode = program_mode::single>
    [[nodiscard]] static error
    program_page_async(uint32_t address,
                       std::span<const uint8_t> src,
                       completion_cb cb) noexcept
    {
        if (src.empty()) { return error::invalid_argument; }
        if (!in_range(address, src.size())) { return error::out_of_range; }
        if ((address % page_size_bytes) + src.size() > page_size_bytes)
        {
            return error::crosses_page;
        }

        if (const error e = prepare_indirect(); e != error::ok) { return e; }
        if (const error e = ensure_idle(); e != error::ok) { return e; }
        if (const error e = write_enable(); e != error::ok) { return e; }

        // If this fails the controller was claimed in a race; WEL stays set
        // until the next program/erase, which is harmless.
        if (!Controller::write_async(program_cmd(Mode), address, 0, src, cb))
        {
            return error::controller_busy;
        }

        s_pending = true;
        return error::ok;
    }

    // ------------------------------------------------------------------
    // Erase (address must be aligned to the erase unit)
    // ------------------------------------------------------------------
    [[nodiscard]] static error
    erase_sector(uint32_t address, bool wait_complete = true) noexcept
    {
        if (!in_range(address, sector_size_bytes)) { return error::out_of_range; }
        if ((address % sector_size_bytes) != 0U)   { return error::misaligned; }

        return erase(cmd_sector_erase_4b, address,
                     Timeouts::value.sector_erase_ms, wait_complete);
    }

    [[nodiscard]] static error
    erase_block_64k(uint32_t address, bool wait_complete = true) noexcept
    {
        if (!in_range(address, block_size_bytes)) { return error::out_of_range; }
        if ((address % block_size_bytes) != 0U)   { return error::misaligned; }

        return erase(cmd_block_erase_64k_4b, address,
                     Timeouts::value.block_erase_ms, wait_complete);
    }

    [[nodiscard]] static error erase_chip(bool wait_complete = true) noexcept
    {
        return erase(cmd_chip_erase, 0,
                     Timeouts::value.chip_erase_ms, wait_complete);
    }

    // Sector-aligned range; uses 64 KiB blocks where possible.
    [[nodiscard]] static error
    erase_range(uint32_t address, uint32_t length) noexcept
    {
        if (length == 0U) { return error::ok; }
        if ((address % sector_size_bytes) != 0U ||
            (length % sector_size_bytes) != 0U)
        {
            return error::misaligned;
        }
        if (!in_range(address, length)) { return error::out_of_range; }

        while (length != 0U)
        {
            const bool use_block =
                (address % block_size_bytes) == 0U && length >= block_size_bytes;

            const uint32_t step = use_block ? block_size_bytes : sector_size_bytes;

            const error e = use_block ? erase_block_64k(address)
                                      : erase_sector(address);
            if (e != error::ok) { return e; }

            address += step;
            length  -= step;
        }

        return error::ok;
    }

    // ------------------------------------------------------------------
    // Memory-mapped read
    // ------------------------------------------------------------------
    // Read command: 0x6C (4-byte address, 8 dummy clocks, x4 data). Requires QE.
    // Memory-mapped writes are not exposed: NOR needs WEL + status polling.
    [[nodiscard]] static error enable_memory_mapped() noexcept
    {
        if (s_mapped) { return error::ok; }
        if (Controller::busy()) { return error::controller_busy; }
        if (const error e = ensure_idle(); e != error::ok) { return e; }

        if (!Controller::memory_mapped(cmd_fast_read_quad_output_4b,
                                       cmd_quad_page_program_4b))
        {
            return error::invalid_argument;
        }

        s_mapped = true;
        return error::ok;
    }

    [[nodiscard]] static bool is_memory_mapped() noexcept { return s_mapped; }

    // Empty span unless memory-mapped mode is active.
    [[nodiscard]] static std::span<const volatile uint8_t> memory_map() noexcept
    {
        if (!s_mapped) { return {}; }
        return {Controller::template memory_base<uint8_t>(), total_size_bytes};
    }

    template <class T>
        requires std::is_trivially_copyable_v<T>
    [[nodiscard]] static const volatile T* memory_ptr() noexcept
    {
        return s_mapped ? Controller::template memory_base<T>() : nullptr;
    }

    // ------------------------------------------------------------------
    // IRQ
    // ------------------------------------------------------------------
    [[nodiscard]] static bool controller_busy() noexcept
    {
        return Controller::busy();
    }

    static void irq_handler() noexcept { Controller::irq_handler(); }

private:
    static inline bool s_pending = false;
    static inline bool s_mapped  = false;

     [[nodiscard]] static uint32_t now_ms() noexcept
    {
        return static_cast<uint32_t>(stm32u3::clock::millis().count());
    }

    // Controller must be free and out of memory-mapped mode for indirect access.
    [[nodiscard]] static error prepare_indirect() noexcept
    {
        if (Controller::busy()) { return error::controller_busy; }

        if (s_mapped)
        {
            if constexpr (requires { Controller::abort(); })
            {
                Controller::abort();
                s_mapped = false;
            }
            else
            {
                return error::memory_mapped_active;
            }
        }

        return error::ok;
    }

    [[nodiscard]] static error do_send(const command& cmd,
                                       uint32_t address = 0) noexcept
    {
        return Controller::send(cmd, address, 0) ? error::ok
                                                 : error::invalid_argument;
    }

    [[nodiscard]] static error read_reg(const command& cmd, uint8_t& out) noexcept
    {
        if (const error e = prepare_indirect(); e != error::ok) { return e; }

        uint8_t v = 0;
        if (!Controller::read_blocking(cmd, 0, 0, std::span<uint8_t>{&v, 1}))
        {
            return error::invalid_argument;
        }
        out = v;
        return error::ok;
    }

    // Cheap pre-check: only talks to the flash if an operation may be pending.
    [[nodiscard]] static error ensure_idle() noexcept
    {
        if (!s_pending) { return error::ok; }

        uint8_t sr = 0;
        if (const error e = read_reg(cmd_read_status1, sr); e != error::ok) { return e; }

        if ((sr & sr1_busy) != 0U) { return error::device_busy; }

        s_pending = false;
        return error::ok;
    }

    // Sends WREN and verifies that WEL is actually set.
    [[nodiscard]] static error write_enable() noexcept
    {
        if (const error e = prepare_indirect(); e != error::ok) { return e; }
        if (const error e = do_send(cmd_write_enable); e != error::ok) { return e; }

        uint8_t sr = 0;
        if (const error e = read_reg(cmd_read_status1, sr); e != error::ok) { return e; }

        return ((sr & sr1_wel) != 0U) ? error::ok : error::write_enable_failed;
    }

    [[nodiscard]] static error
    erase(const command& cmd, uint32_t address,
          uint32_t timeout_ms, bool wait_complete) noexcept
    {
        if (const error e = wait_ready(Timeouts::value.chip_erase_ms); e != error::ok) { return e; }
        if (const error e = write_enable(); e != error::ok) { return e; }

        if (const error e = do_send(cmd, address); e != error::ok) { return e; }
        s_pending = true;

        return wait_complete ? wait_ready(timeout_ms) : error::ok;
    }
};

} // namespace stm32u3::qspi::w25q512jv