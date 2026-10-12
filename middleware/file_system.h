#pragma once

/// @file file_system.h
/// @brief norfs: small log-structured, power-fail-safe file system for NOR flash.
///
/// Execution model
///  - Fully asynchronous and non-blocking: every *_start() call only begins an
///    operation; process() advances it one step each time a hardware transfer
///    has finished. No call ever waits for hardware.
///  - Single context: every member function must be called from the same
///    context (normally the main loop). irq() is the only exception - it is
///    ISR-safe and merely records that hardware finished (see irq_pending()).
///  - No heap, no exceptions, no RTTI, no virtual dispatch; the NOR and CRC
///    back ends are template parameters constrained by concepts.
///
/// Contract of the back ends (see AsyncNor / AsyncCrc32)
///  - start_*() returns false without side effects if it cannot start;
///    otherwise state() must read `busy` as soon as it returns true, and keep
///    reading done/error until the next start_*().
///  - Buffers handed to start_program() / start() must stay valid until the
///    operation has finished (the file system guarantees this for its own
///    buffers; the buffer given to write_start() must stay valid until
///    is_busy() is false).
///  - start_program() must accept any address/length inside the device and
///    split page-boundary crossings itself; a record is never programmed
///    over non-erased bytes.
///  - mapped(address) is a read-only (memory-mapped / XIP) view of the device.
///
/// Power-fail behaviour
///  - A record becomes visible only after its header (with CRCs) is programmed;
///    a torn record is ignored on mount, its magic is zeroed, and its block is
///    retired. A block becomes valid only when its commit word is programmed.
///  - GC copies the live image into an erased block, commits it, and only then
///    erases the oldest block, so a cut at any point leaves a complete image.
///  - One erased block is always kept in reserve for GC; no_space means "run
///    gc_start() and repeat". Mount reclaims blocks left uncommitted by a cut.
///
/// RAM: file table + staging buffers + one MaxDataPerRecord buffer used by GC.
///
/// On-flash format (little-endian, version 2)
///  block  : BlockHeader (32 B) | records...        (header committed last)
///  record : RecordHeader (40 B) padded to program_size | payload
///           (whole record padded to program_size)
///  Block generations and record sequence numbers only ever increase, so
///  replaying blocks in generation order replays history in order.

#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <string_view>
#include <type_traits>

namespace norfs
{

static_assert(std::endian::native == std::endian::little,
              "the on-flash format is little-endian");

enum class Error : std::int8_t
{
    ok             = 0,
    busy           = 1,    ///< result(): operation running / start: not accepted, FS busy
    retry          = 2,    ///< a new block is being prepared: repeat the call once idle
    invalid        = -1,
    not_mounted    = -2,
    not_found      = -3,
    exists         = -4,
    no_space       = -5,   ///< run gc_start() and repeat
    io             = -6,
    corrupt        = -7,
    name_too_long  = -8,
    too_many_files = -9,
    bad_geometry   = -10,
    crc            = -11,
};

enum class AsyncState : std::uint8_t { idle, busy, done, error };

struct Geometry
{
    std::uint32_t total_size{};
    std::uint32_t erase_size{};
    std::uint32_t program_size{};
};

/// Physical NOR: XSPI + GPDMA for program/erase, memory-mapped mode for reads.
template <class T>
concept AsyncNor = requires(T& nor, const T& cnor, std::uint32_t address,
                            std::span<const std::byte> data) {
    { cnor.geometry() } noexcept -> std::same_as<Geometry>;
    { cnor.mapped(address) } noexcept -> std::same_as<const std::byte*>;
    { nor.start_program(address, data) } noexcept -> std::same_as<bool>;
    { nor.start_erase(address) } noexcept -> std::same_as<bool>;
    { cnor.state() } noexcept -> std::same_as<AsyncState>;
};

/// Hardware CRC32 (e.g. GPDMA -> CRC->DR). The span must stay valid until the
/// operation has finished.
template <class T>
concept AsyncCrc32 = requires(T& crc, const T& ccrc, std::span<const std::byte> data) {
    { crc.start(data) } noexcept -> std::same_as<bool>;
    { ccrc.state() } noexcept -> std::same_as<AsyncState>;
    { ccrc.value() } noexcept -> std::same_as<std::uint32_t>;
};

namespace detail
{

inline constexpr std::uint32_t npos           = 0xFFFF'FFFFU;
inline constexpr std::uint32_t erased32       = 0xFFFF'FFFFU;
inline constexpr std::uint32_t block_magic    = 0x314B'4C42U;   // "BLK1"
inline constexpr std::uint32_t record_magic   = 0x3143'4552U;   // "REC1"
inline constexpr std::uint32_t format_version = 2U;
inline constexpr std::uint32_t block_commit   = 0xC0A1'17EDU;

inline constexpr std::uint16_t rec_create = 1U;
inline constexpr std::uint16_t rec_data   = 2U;
inline constexpr std::uint16_t rec_remove = 3U;

// Explicit reserved fields: no implicit padding ever reaches the flash, so the
// bytes covered by the CRC are exactly the bytes that are programmed.
struct BlockHeader
{
    std::uint32_t magic;
    std::uint32_t version;
    std::uint64_t generation;
    std::uint32_t erase_count;
    std::uint32_t crc;          ///< CRC32 of the bytes before this field
    std::uint32_t commit;       ///< programmed last; block is valid only if set
    std::uint32_t reserved;
};

struct RecordHeader
{
    std::uint32_t magic;
    std::uint16_t type;
    std::uint16_t flags;
    std::uint32_t file_id;
    std::uint32_t offset;
    std::uint32_t size;
    std::uint32_t reserved;
    std::uint64_t sequence;
    std::uint32_t payload_crc;
    std::uint32_t header_crc;   ///< CRC32 of the bytes before this field
};

inline constexpr std::uint32_t block_header_size   = sizeof(BlockHeader);
inline constexpr std::uint32_t record_header_size  = sizeof(RecordHeader);
inline constexpr std::uint32_t block_crc_offset    = 20U;
inline constexpr std::uint32_t block_commit_offset = 24U;
inline constexpr std::uint32_t record_crc_offset   = 36U;

static_assert(block_header_size == 32U && record_header_size == 40U);
static_assert(std::has_unique_object_representations_v<BlockHeader>);
static_assert(std::has_unique_object_representations_v<RecordHeader>);
static_assert(offsetof(BlockHeader, crc) == block_crc_offset);
static_assert(offsetof(BlockHeader, commit) == block_commit_offset);
static_assert(offsetof(RecordHeader, header_crc) == record_crc_offset);

[[nodiscard]] constexpr std::uint32_t align_up(std::uint32_t n, std::uint32_t a) noexcept
{
    return ((n + a - 1U) / a) * a;
}

template <class T>
    requires std::is_trivially_copyable_v<T>
[[nodiscard]] constexpr T load(std::span<const std::byte> bytes) noexcept
{
    std::array<std::byte, sizeof(T)> raw{};
    std::ranges::copy(bytes.first(sizeof(T)), raw.begin());
    return std::bit_cast<T>(raw);
}

template <class T>
    requires std::is_trivially_copyable_v<T>
constexpr void store(std::span<std::byte> bytes, const T& value) noexcept
{
    const auto raw = std::bit_cast<std::array<std::byte, sizeof(T)>>(value);
    std::ranges::copy(raw, bytes.begin());
}

/// Ordering key of a block: (generation, index).
[[nodiscard]] constexpr bool key_less(std::uint64_t gen_a, std::uint32_t blk_a,
                                      std::uint64_t gen_b, std::uint32_t blk_b) noexcept
{
    return gen_a != gen_b ? gen_a < gen_b : blk_a < blk_b;
}

inline constexpr std::array<std::byte, 4> commit_bytes =
    std::bit_cast<std::array<std::byte, 4>>(block_commit);

/// Programmed over the magic of a torn record: clearing bits is always legal
/// on NOR, and a zero magic ends every scan at that record.
inline constexpr std::array<std::byte, 4> zero_word{};

} // namespace detail

template <AsyncNor Nor, AsyncCrc32 Crc,
          std::size_t MaxFiles        = 32,
          std::size_t MaxName         = 48,
          std::size_t MaxFileSize     = 1024U * 1024U,
          std::size_t MaxDataPerRecord = 2048,
          std::size_t MaxProgramSize  = 256>
class FileSystem
{
    static_assert(MaxFiles > 0U && MaxFiles < detail::npos);
    static_assert(MaxName >= 2U && MaxName <= MaxDataPerRecord);
    static_assert(MaxDataPerRecord > 0U);
    static_assert(MaxFileSize > 0U && MaxFileSize <= std::numeric_limits<std::uint32_t>::max());
    static_assert(std::has_single_bit(MaxProgramSize));
    static_assert(std::atomic<bool>::is_always_lock_free);

    enum class Op : std::uint8_t
    {
        idle,
        format_erase,
        block_crc, block_program, block_commit_program,
        rec_payload_crc, rec_header_crc, rec_header_program, rec_payload_program,
        mount_block_crc, mount_record_header_crc, mount_record_payload_crc,
        mount_reclaim_erase,
        poison_program,
        gc_commit_program, gc_erase,
    };

    enum class Waits : std::uint8_t { none, nor, crc };
    enum class Origin : std::uint8_t { user, gc };

    struct FileEntry
    {
        bool                       used{};
        bool                       deleted{};
        std::array<char, MaxName>  name{};
        std::uint32_t              size{};
        std::uint64_t              create_seq{};   ///< records older than this belong to an earlier file with the same id
    };

    struct Pending
    {
        std::uint16_t              type{};
        std::uint32_t              file_id{};
        std::uint32_t              offset{};
        std::uint32_t              size{};
        std::uint64_t              sequence{};
        std::span<const std::byte> payload{};
        std::uint32_t              payload_crc{};
        std::uint32_t              header_crc{};
        std::uint32_t              record_address{};
        Origin                     origin{Origin::user};
    };

    struct Gc
    {
        bool          active{};
        bool          created{};        ///< create record of gc.file already copied
        std::uint32_t victim{detail::npos};
        std::uint32_t destination{detail::npos};
        std::size_t   file{};
        std::uint32_t offset{};
    };

    /// Result of locating the newest copy of a logical byte range.
    struct Source
    {
        std::uint32_t address{detail::npos};    ///< flash address of the first byte
        std::uint32_t size{};                   ///< bytes available from one record
        std::uint32_t next_start{detail::npos}; ///< start of the next covered range (if address == npos)
    };

    static constexpr std::size_t stage_size =
        std::bit_ceil(std::max<std::size_t>(MaxProgramSize, detail::record_header_size));

    Nor&           nor_;
    Crc&           crc_;
    Geometry       g_{};
    std::uint32_t  blocks_{};

    std::array<FileEntry, MaxFiles> files_{};
    Pending        pending_{};
    Gc             gc_{};
    std::optional<std::size_t> pending_new_file_{};

    Op             op_{Op::idle};
    Error          result_{Error::ok};
    bool           mounted_{};
    std::atomic<bool> irq_flag_{false};

    std::uint32_t  active_block_{detail::npos};
    std::uint32_t  write_pos_{};
    std::uint64_t  next_generation_{1};
    std::uint64_t  next_sequence_{1};

    // a block being initialised (rollover / GC destination): restore on failure
    bool           init_pending_{};
    bool           init_for_gc_{};
    std::uint32_t  prev_active_{detail::npos};
    std::uint32_t  prev_write_pos_{};

    std::uint32_t  format_block_{};

    // neutralising a torn record (see poison_program)
    bool           poison_requested_{};
    bool           poison_mount_{};
    std::uint32_t  poison_address_{};

    // mount: blocks are replayed in (generation, index) order
    bool                mount_have_prev_{};
    std::uint64_t       mount_prev_gen_{};
    std::uint32_t       mount_prev_block_{};
    std::uint32_t       mount_block_{detail::npos};
    std::uint32_t       mount_pos_{};
    std::uint64_t       mount_last_gen_{};
    std::uint32_t       mount_last_block_{detail::npos};
    std::uint32_t       mount_last_end_{};
    std::uint32_t       reclaim_block_{};
    detail::BlockHeader  mount_block_header_{};
    detail::RecordHeader mount_record_{};
    std::uint32_t       mount_record_address_{};
    std::uint32_t       mount_record_total_{};

    // staging buffers for CRC/program operations (live until completion)
    std::array<std::byte, stage_size> header_buf_{};
    std::array<std::byte, stage_size> block_buf_{};
    std::array<std::byte, MaxDataPerRecord> gc_buf_{};   ///< GC chunk assembly (RAM cost: MaxDataPerRecord)

    // ------------------------------------------------------------------
    // Geometry / addressing
    // ------------------------------------------------------------------

    [[nodiscard]] static constexpr std::uint32_t first_record_offset() noexcept
    {
        return detail::align_up(detail::block_header_size, 4U);
    }

    [[nodiscard]] std::uint32_t addr(std::uint32_t block, std::uint32_t offset = 0U) const noexcept
    {
        return (block * g_.erase_size) + offset;
    }

    /// Offset of the payload inside a record (header padded to program size).
    [[nodiscard]] std::uint32_t payload_offset() const noexcept
    {
        return detail::align_up(detail::record_header_size, g_.program_size);
    }

    /// Flash footprint of a record with `payload` bytes.
    [[nodiscard]] std::uint32_t rec_total(std::uint32_t payload) const noexcept
    {
        return detail::align_up(payload_offset() + payload, g_.program_size);
    }

    [[nodiscard]] bool geometry_ok() const noexcept
    {
        if (g_.total_size == 0U || g_.erase_size == 0U || g_.program_size == 0U) { return false; }
        if (!std::has_single_bit(g_.program_size) || g_.program_size > MaxProgramSize) { return false; }
        if ((g_.total_size % g_.erase_size) != 0U || (g_.erase_size % g_.program_size) != 0U) { return false; }
        if (blocks_ < 3U) { return false; }
        const std::uint64_t largest = rec_total(static_cast<std::uint32_t>(MaxDataPerRecord));
        return first_record_offset() + largest <= g_.erase_size;
    }

    [[nodiscard]] bool busy() const noexcept { return op_ != Op::idle; }

    [[nodiscard]] std::optional<std::span<const std::byte>>
    window(std::uint32_t address, std::uint32_t length) const noexcept
    {
        if (std::uint64_t{address} + length > g_.total_size) { return std::nullopt; }
        const std::byte* const p = nor_.mapped(address);
        if (p == nullptr) { return std::nullopt; }
        return std::span<const std::byte>{p, length};
    }

    [[nodiscard]] std::optional<detail::BlockHeader> peek_block(std::uint32_t block) const noexcept
    {
        const auto w = window(addr(block), detail::block_header_size);
        if (!w) { return std::nullopt; }
        return detail::load<detail::BlockHeader>(*w);
    }

    [[nodiscard]] static bool is_header(const detail::BlockHeader& h) noexcept
    {
        return h.magic == detail::block_magic && h.version == detail::format_version;
    }

    [[nodiscard]] bool is_erased(std::uint32_t block) const noexcept
    {
        const auto w = window(addr(block), detail::block_header_size);
        return w && std::ranges::all_of(*w, [](std::byte b) { return b == std::byte{0xFF}; });
    }

    [[nodiscard]] std::uint32_t first_erased() const noexcept
    {
        for (std::uint32_t b = 0; b < blocks_; ++b)
        {
            if (b != active_block_ && is_erased(b)) { return b; }
        }
        return detail::npos;
    }

    [[nodiscard]] std::uint32_t erased_blocks() const noexcept
    {
        std::uint32_t n = 0;
        for (std::uint32_t b = 0; b < blocks_; ++b)
        {
            if (b != active_block_ && is_erased(b)) { ++n; }
        }
        return n;
    }

    [[nodiscard]] bool reserve(std::uint32_t bytes) const noexcept
    {
        return active_block_ != detail::npos &&
               std::uint64_t{write_pos_} + bytes <= g_.erase_size;
    }

    // ------------------------------------------------------------------
    // File table
    // ------------------------------------------------------------------

    void clear_files() noexcept { files_.fill(FileEntry{}); }

    [[nodiscard]] static std::string_view name_of(const FileEntry& e) noexcept
    {
        const auto end = std::ranges::find(e.name, '\0');
        return {e.name.data(), static_cast<std::size_t>(end - e.name.begin())};
    }

    [[nodiscard]] std::optional<std::size_t> find_file(std::string_view name) const noexcept
    {
        for (std::size_t i = 0; i < MaxFiles; ++i)
        {
            if (files_[i].used && !files_[i].deleted && name_of(files_[i]) == name) { return i; }
        }
        return std::nullopt;
    }

    /// Takes a free slot, else recycles a deleted one (older records of the
    /// previous file are hidden by create_seq and replayed in order on mount).
    [[nodiscard]] std::optional<std::size_t> alloc_file(std::string_view name) noexcept
    {
        auto it = std::ranges::find_if(files_, [](const FileEntry& e) { return !e.used; });
        if (it == files_.end())
        {
            it = std::ranges::find_if(files_, [](const FileEntry& e) { return e.deleted; });
        }
        if (it == files_.end()) { return std::nullopt; }

        *it = FileEntry{};
        it->used = true;
        std::ranges::copy(name, it->name.begin());   // name.size() < MaxName, rest stays 0
        return static_cast<std::size_t>(it - files_.begin());
    }

    // ------------------------------------------------------------------
    // Completion helpers
    // ------------------------------------------------------------------

    void finish(Error e) noexcept
    {
        result_ = e;
        op_     = Op::idle;
    }

    void fail(Error e) noexcept
    {
        // A record write interrupted mid-way leaves a valid-looking header
        // without (all of) its payload: retire the block and neutralise it.
        if ((op_ == Op::rec_header_program || op_ == Op::rec_payload_program) &&
            pending_.origin == Origin::user)
        {
            poison_requested_ = true;
            poison_address_   = pending_.record_address;
            write_pos_        = g_.erase_size;
        }
        if (pending_new_file_)
        {
            files_[*pending_new_file_] = FileEntry{};
            pending_new_file_.reset();
        }
        if (init_pending_)
        {
            active_block_  = prev_active_;
            write_pos_     = prev_write_pos_;
            init_pending_  = false;
        }
        gc_      = Gc{};
        pending_ = Pending{};
        finish(e);
    }

    void begin_crc(Op next, std::span<const std::byte> data) noexcept
    {
        if (!crc_.start(data)) { fail(Error::crc); return; }
        op_ = next;
    }

    void begin_program(Op next, std::uint32_t address, std::span<const std::byte> data) noexcept
    {
        if (!nor_.start_program(address, data)) { fail(Error::io); return; }
        op_ = next;
    }

    void begin_erase(Op next, std::uint32_t block) noexcept
    {
        if (!nor_.start_erase(addr(block))) { fail(Error::io); return; }
        op_ = next;
    }

    [[nodiscard]] static constexpr Waits waits_on(Op op) noexcept
    {
        switch (op)
        {
        case Op::format_erase:
        case Op::block_program:
        case Op::block_commit_program:
        case Op::rec_header_program:
        case Op::rec_payload_program:
        case Op::mount_reclaim_erase:
        case Op::poison_program:
        case Op::gc_commit_program:
        case Op::gc_erase:
            return Waits::nor;
        case Op::block_crc:
        case Op::rec_payload_crc:
        case Op::rec_header_crc:
        case Op::mount_block_crc:
        case Op::mount_record_header_crc:
        case Op::mount_record_payload_crc:
            return Waits::crc;
        case Op::idle:
            break;
        }
        return Waits::none;
    }

    // ------------------------------------------------------------------
    // Block initialisation (format / rollover / GC destination)
    // ------------------------------------------------------------------

    void start_block_init(std::uint32_t block, bool for_gc) noexcept
    {
        init_pending_   = true;
        init_for_gc_    = for_gc;
        prev_active_    = active_block_;
        prev_write_pos_ = write_pos_;
        active_block_   = block;
        write_pos_      = first_record_offset();

        block_buf_.fill(std::byte{0xFF});
        detail::store(std::span{block_buf_},
                      detail::BlockHeader{.magic      = detail::block_magic,
                                          .version    = detail::format_version,
                                          .generation = next_generation_++,
                                          .erase_count = 0U,
                                          .crc        = 0U,
                                          .commit     = detail::erased32,
                                          .reserved   = detail::erased32});
        begin_crc(Op::block_crc,
                  std::span<const std::byte>{block_buf_}.first(detail::block_crc_offset));
    }

    /// Keeps one erased block in reserve so GC always has a destination.
    [[nodiscard]] Error rollover_start() noexcept
    {
        if (erased_blocks() < 2U) { return Error::no_space; }
        result_ = Error::busy;
        start_block_init(first_erased(), false);
        return op_ == Op::idle ? result_ : Error::retry;
    }

    // ------------------------------------------------------------------
    // Record pipeline: payload CRC -> header CRC -> header -> payload
    // ------------------------------------------------------------------

    void stage_record_header() noexcept
    {
        header_buf_.fill(std::byte{0xFF});
        detail::store(std::span{header_buf_},
                      detail::RecordHeader{.magic       = detail::record_magic,
                                           .type        = pending_.type,
                                           .flags       = 0U,
                                           .file_id     = pending_.file_id,
                                           .offset      = pending_.offset,
                                           .size        = pending_.size,
                                           .reserved    = 0U,
                                           .sequence    = pending_.sequence,
                                           .payload_crc = pending_.payload_crc,
                                           .header_crc  = pending_.header_crc});
    }

    void begin_pending(std::uint16_t type, std::uint32_t file, std::uint32_t offset,
                       std::span<const std::byte> payload, Origin origin) noexcept
    {
        pending_                = Pending{};
        pending_.type           = type;
        pending_.file_id        = file;
        pending_.offset         = offset;
        pending_.size           = static_cast<std::uint32_t>(payload.size());
        pending_.sequence       = next_sequence_++;
        pending_.payload        = payload;
        pending_.origin         = origin;
        pending_.record_address = addr(active_block_, write_pos_);
        // Zero-length payloads still go through the CRC engine so every
        // record follows the same hardware contract.
        begin_crc(Op::rec_payload_crc, payload);
    }

    void complete_pending() noexcept
    {
        write_pos_ += rec_total(pending_.size);
        const Pending done = pending_;
        pending_           = Pending{};

        if (done.origin == Origin::gc)
        {
            // GC only re-writes history that is already in RAM.
            if (done.type == detail::rec_create) { gc_.created = true; }
            else { gc_.offset += done.size; }
            continue_gc();
            return;
        }

        FileEntry& f = files_[done.file_id];
        if (done.type == detail::rec_create)
        {
            pending_new_file_.reset();
            f.create_seq = done.sequence;
        }
        else if (done.type == detail::rec_data)
        {
            f.size = std::max(f.size, done.offset + done.size);
        }
        else
        {
            f.deleted = true;
        }
        finish(Error::ok);
    }

    // ------------------------------------------------------------------
    // Mount: replay blocks in (generation, index) order
    // ------------------------------------------------------------------

    void mount_next_block() noexcept
    {
        std::uint32_t       best = detail::npos;
        detail::BlockHeader best_header{};

        for (std::uint32_t b = 0; b < blocks_; ++b)
        {
            const auto h = peek_block(b);
            if (!h) { fail(Error::io); return; }
            if (!is_header(*h) || h->commit != detail::block_commit) { continue; }
            if (mount_have_prev_ &&
                !detail::key_less(mount_prev_gen_, mount_prev_block_, h->generation, b)) { continue; }
            if (best == detail::npos ||
                detail::key_less(h->generation, b, best_header.generation, best))
            {
                best        = b;
                best_header = *h;
            }
        }

        if (best == detail::npos) { mount_reclaim_next(); return; }

        mount_have_prev_    = true;
        mount_prev_gen_     = best_header.generation;
        mount_prev_block_   = best;
        mount_block_        = best;
        mount_block_header_ = best_header;

        detail::store(std::span{block_buf_}, best_header);
        begin_crc(Op::mount_block_crc,
                  std::span<const std::byte>{block_buf_}.first(detail::block_crc_offset));
    }

    void mount_block_end() noexcept
    {
        mount_last_end_ = mount_pos_;
        mount_next_block();
    }

    void mount_begin_record() noexcept
    {
        if (std::uint64_t{mount_pos_} + detail::record_header_size > g_.erase_size)
        {
            mount_block_end();
            return;
        }
        const auto w = window(addr(mount_block_, mount_pos_), detail::record_header_size);
        if (!w) { fail(Error::io); return; }

        if (std::ranges::all_of(*w, [](std::byte b) { return b == std::byte{0xFF}; }))
        {
            mount_block_end();      // end of written data
            return;
        }

        const auto h = detail::load<detail::RecordHeader>(*w);
        if (h.magic != detail::record_magic ||
            h.type < detail::rec_create || h.type > detail::rec_remove ||
            h.size > MaxDataPerRecord ||
            std::uint64_t{h.offset} + h.size > MaxFileSize)
        {
            mount_block_end();      // torn tail ends this block, not the file system
            return;
        }

        const std::uint32_t total = rec_total(h.size);
        if (std::uint64_t{mount_pos_} + total > g_.erase_size)
        {
            mount_block_end();
            return;
        }

        mount_record_         = h;
        mount_record_address_ = addr(mount_block_, mount_pos_);
        mount_record_total_   = total;
        std::ranges::copy(*w, header_buf_.begin());
        begin_crc(Op::mount_record_header_crc,
                  std::span<const std::byte>{header_buf_}.first(detail::record_crc_offset));
    }

    void apply_mount_record() noexcept
    {
        const detail::RecordHeader& h = mount_record_;
        next_sequence_ = std::max(next_sequence_, h.sequence + 1U);

        if (h.file_id < MaxFiles)
        {
            FileEntry& f = files_[h.file_id];
            if (h.type == detail::rec_create)
            {
                if (h.size != 0U && h.size < MaxName)
                {
                    const auto w = window(mount_record_address_ + payload_offset(), h.size);
                    if (!w) { fail(Error::io); return; }
                    f = FileEntry{};
                    f.used       = true;
                    f.create_seq = h.sequence;
                    std::ranges::transform(*w, f.name.begin(),
                                           [](std::byte b) { return static_cast<char>(b); });
                }
            }
            else if (h.type == detail::rec_data)
            {
                // Stale records of a file that no longer exists are skipped.
                if (f.used && !f.deleted && h.sequence >= f.create_seq)
                {
                    f.size = std::max(f.size, h.offset + h.size);
                }
            }
            else if (f.used)
            {
                f.deleted = true;
            }
        }

        mount_pos_ += mount_record_total_;
        mount_begin_record();
    }

    /// Erases blocks whose initialisation never committed (power loss during
    /// rollover or GC): they hold no live data and would otherwise leak.
    void mount_reclaim_next() noexcept
    {
        while (reclaim_block_ < blocks_)
        {
            const std::uint32_t b = reclaim_block_++;
            const auto h = peek_block(b);
            if (!h) { fail(Error::io); return; }
            if (is_header(*h) && h->commit != detail::block_commit)
            {
                begin_erase(Op::mount_reclaim_erase, b);
                return;
            }
        }
        mount_complete();
    }

    void mount_complete() noexcept
    {
        if (mount_last_block_ == detail::npos)
        {
            // Blank device: bring up the first block.
            active_block_    = detail::npos;
            write_pos_       = 0U;
            next_generation_ = 1U;
            const std::uint32_t b = first_erased();
            if (b == detail::npos) { fail(Error::no_space); return; }
            start_block_init(b, false);
            return;
        }
        active_block_    = mount_last_block_;
        write_pos_       = mount_last_end_;
        next_generation_ = mount_last_gen_ + 1U;

        // A torn record (power loss mid-write) leaves programmed bytes behind
        // the last valid record. Never program over them: retire the block
        // (the next write rolls over, GC reclaims it) and zero the torn
        // record's magic so that no scan ever mistakes it for data.
        const std::uint32_t tail_address = addr(mount_last_block_, mount_last_end_);
        const auto tail = window(tail_address, g_.erase_size - mount_last_end_);
        if (!tail) { fail(Error::io); return; }
        if (std::ranges::all_of(*tail, [](std::byte b) { return b == std::byte{0xFF}; }))
        {
            mounted_ = true;
            finish(Error::ok);
            return;
        }

        write_pos_ = g_.erase_size;
        const auto magic = tail->first(std::min<std::size_t>(tail->size(), detail::zero_word.size()));
        const bool erased_magic = std::ranges::all_of(magic, [](std::byte b) { return b == std::byte{0xFF}; });
        const bool zero_magic   = std::ranges::all_of(magic, [](std::byte b) { return b == std::byte{0x00}; });
        if (magic.size() < detail::zero_word.size() || erased_magic || zero_magic)
        {
            mounted_ = true;
            finish(Error::ok);
            return;
        }
        poison_mount_ = true;
        begin_program(Op::poison_program, tail_address, std::span<const std::byte>{detail::zero_word});
    }

    // ------------------------------------------------------------------
    // Reading: newest copy of a logical byte range
    // ------------------------------------------------------------------

    /// Calls visit(block, position, header) for every record of every block,
    /// in physical order. Returns false if the flash could not be mapped.
    template <class Visitor>
    [[nodiscard]] bool scan_records(Visitor&& visit) const noexcept
    {
        for (std::uint32_t b = 0; b < blocks_; ++b)
        {
            const auto h = peek_block(b);
            if (!h) { return false; }
            if (!is_header(*h) || h->commit != detail::block_commit) { continue; }

            const auto w = window(addr(b), g_.erase_size);
            if (!w) { return false; }

            std::uint32_t p = first_record_offset();
            while (std::uint64_t{p} + detail::record_header_size <= g_.erase_size)
            {
                const auto rh = detail::load<detail::RecordHeader>(w->subspan(p));
                if (rh.magic != detail::record_magic || rh.size > MaxDataPerRecord) { break; }
                const std::uint32_t total = rec_total(rh.size);
                if (std::uint64_t{p} + total > g_.erase_size) { break; }
                visit(b, p, rh);
                p += total;
            }
        }
        return true;
    }

    /// Finds the record holding the newest copy of byte `logical` and how many
    /// bytes (<= wanted) can be served from it before a newer record takes over.
    /// corrupt = byte never written (out.next_start hints at the next written range).
    [[nodiscard]] Error locate(std::uint32_t file, std::uint32_t logical,
                               std::uint32_t wanted, Source& out) const noexcept
    {
        out = Source{};
        const std::uint64_t min_seq = files_[file].create_seq;

        bool          found     = false;
        std::uint64_t best_seq  = 0;
        std::uint64_t best_end  = 0;
        std::uint32_t best_addr = 0;
        std::uint32_t next      = detail::npos;

        const bool scanned = scan_records(
            [&](std::uint32_t b, std::uint32_t p, const detail::RecordHeader& h) {
                if (h.type != detail::rec_data || h.file_id != file || h.sequence < min_seq) { return; }
                const std::uint64_t end = std::uint64_t{h.offset} + h.size;
                if (logical >= h.offset && logical < end)
                {
                    if (!found || h.sequence >= best_seq)
                    {
                        found     = true;
                        best_seq  = h.sequence;
                        best_end  = end;
                        best_addr = addr(b, p) + payload_offset() + (logical - h.offset);
                    }
                }
                else if (h.offset > logical && h.offset < next)
                {
                    next = h.offset;
                }
            });
        if (!scanned) { return Error::io; }
        if (!found)
        {
            out.next_start = next;
            return Error::corrupt;
        }

        // A newer record that starts inside the range overrides the tail of it.
        std::uint64_t limit = std::min<std::uint64_t>(best_end, std::uint64_t{logical} + wanted);
        const bool rescanned = scan_records(
            [&](std::uint32_t, std::uint32_t, const detail::RecordHeader& h) {
                if (h.type == detail::rec_data && h.file_id == file && h.sequence >= min_seq &&
                    h.sequence > best_seq && h.offset > logical && h.offset < limit)
                {
                    limit = h.offset;
                }
            });
        if (!rescanned) { return Error::io; }

        out.address = best_addr;
        out.size    = static_cast<std::uint32_t>(limit - logical);
        return Error::ok;
    }

    // ------------------------------------------------------------------
    // Garbage collection: compact live data into an erased block, commit it,
    // then erase the oldest block.
    // ------------------------------------------------------------------

    [[nodiscard]] bool select_gc() noexcept
    {
        const std::uint32_t destination = first_erased();
        if (destination == detail::npos) { return false; }

        std::uint32_t       victim = detail::npos;
        detail::BlockHeader victim_header{};
        for (std::uint32_t b = 0; b < blocks_; ++b)
        {
            if (b == active_block_ || b == destination) { continue; }
            const auto h = peek_block(b);
            if (!h || !is_header(*h) || h->commit != detail::block_commit) { continue; }
            if (victim == detail::npos ||
                detail::key_less(h->generation, b, victim_header.generation, victim))
            {
                victim        = b;
                victim_header = *h;
            }
        }
        if (victim == detail::npos) { return false; }

        gc_ = Gc{.active = true, .victim = victim, .destination = destination};
        return true;
    }

    void next_gc_file() noexcept
    {
        ++gc_.file;
        gc_.offset  = 0U;
        gc_.created = false;
    }

    void continue_gc() noexcept
    {
        while (gc_.file < MaxFiles)
        {
            const FileEntry& f = files_[gc_.file];
            if (!f.used || f.deleted) { next_gc_file(); continue; }
            const auto file = static_cast<std::uint32_t>(gc_.file);

            // The create record carries the name; without it a remount could
            // not attach the copied data to a file.
            if (!gc_.created)
            {
                const std::string_view name = name_of(f);
                if (!reserve(rec_total(static_cast<std::uint32_t>(name.size())))) { fail(Error::no_space); return; }
                begin_pending(detail::rec_create, file, 0U,
                              std::as_bytes(std::span{name.data(), name.size()}), Origin::gc);
                return;
            }

            if (gc_.offset >= f.size) { next_gc_file(); continue; }

            const auto wanted = std::min<std::uint32_t>(f.size - gc_.offset,
                                                        static_cast<std::uint32_t>(MaxDataPerRecord));
            Source src{};
            Error e = locate(file, gc_.offset, wanted, src);
            if (e == Error::corrupt && src.next_start != detail::npos && src.next_start > gc_.offset)
            {
                gc_.offset = src.next_start;      // hole in a sparse file
                continue;
            }
            if (e != Error::ok) { fail(e == Error::io ? Error::io : Error::corrupt); return; }

            // Assemble the newest bytes of the next chunk into one contiguous
            // buffer so the copy is fully compacted, however fragmented the
            // history of overlapping writes was.
            std::uint32_t n = 0;
            for (;;)
            {
                const auto w = window(src.address, src.size);
                if (!w) { fail(Error::io); return; }
                std::ranges::copy(*w, gc_buf_.begin() + static_cast<std::ptrdiff_t>(n));
                n += src.size;
                if (n >= wanted) { break; }
                e = locate(file, gc_.offset + n, wanted - n, src);
                if (e == Error::io) { fail(Error::io); return; }
                if (e != Error::ok) { break; }    // hole: the chunk ends here
            }

            if (!reserve(rec_total(n))) { fail(Error::no_space); return; }
            begin_pending(detail::rec_data, file, gc_.offset,
                          std::span<const std::byte>{gc_buf_}.first(n), Origin::gc);
            return;
        }

        // Everything is in the destination: commit it, then erase the victim.
        // Power loss before the commit leaves the old image, after it the new.
        begin_program(Op::gc_commit_program, addr(active_block_, detail::block_commit_offset),
                      std::span<const std::byte>{detail::commit_bytes});
    }

    // ------------------------------------------------------------------
    // Hardware completion handlers
    // ------------------------------------------------------------------

    void on_nor_done() noexcept
    {
        switch (op_)
        {
        case Op::format_erase:
            if (++format_block_ < blocks_) { begin_erase(Op::format_erase, format_block_); }
            else
            {
                active_block_    = detail::npos;
                write_pos_       = 0U;
                next_generation_ = 1U;
                start_block_init(0U, false);
            }
            break;

        case Op::block_program:
            if (init_for_gc_) { continue_gc(); }
            else
            {
                begin_program(Op::block_commit_program,
                              addr(active_block_, detail::block_commit_offset),
                              std::span<const std::byte>{detail::commit_bytes});
            }
            break;

        case Op::block_commit_program:
            init_pending_ = false;
            mounted_      = true;
            finish(Error::ok);
            break;

        case Op::rec_header_program:
            if (pending_.size == 0U) { complete_pending(); }
            else
            {
                begin_program(Op::rec_payload_program,
                              pending_.record_address + payload_offset(), pending_.payload);
            }
            break;

        case Op::rec_payload_program:
            complete_pending();
            break;

        case Op::mount_reclaim_erase:
            mount_reclaim_next();
            break;

        case Op::poison_program:
            if (poison_mount_)
            {
                poison_mount_ = false;
                mounted_      = true;
                finish(Error::ok);
            }
            else { op_ = Op::idle; }        // housekeeping: result of the last operation stays
            break;

        case Op::gc_commit_program:
            init_pending_ = false;
            begin_erase(Op::gc_erase, gc_.victim);
            break;

        case Op::gc_erase:
            gc_ = Gc{};
            finish(Error::ok);
            break;

        default:
            break;
        }
    }

    void on_crc_done() noexcept
    {
        const std::uint32_t value = crc_.value();
        switch (op_)
        {
        case Op::block_crc:
        {
            auto h = detail::load<detail::BlockHeader>(std::span<const std::byte>{block_buf_});
            h.crc  = value;
            detail::store(std::span{block_buf_}, h);
            begin_program(Op::block_program, addr(active_block_),
                          std::span<const std::byte>{block_buf_}.first(detail::block_header_size));
            break;
        }

        case Op::rec_payload_crc:
            pending_.payload_crc = value;
            stage_record_header();
            begin_crc(Op::rec_header_crc,
                      std::span<const std::byte>{header_buf_}.first(detail::record_crc_offset));
            break;

        case Op::rec_header_crc:
            pending_.header_crc = value;
            stage_record_header();
            begin_program(Op::rec_header_program, pending_.record_address,
                          std::span<const std::byte>{header_buf_}.first(payload_offset()));
            break;

        case Op::mount_block_crc:
            if (value != mount_block_header_.crc) { mount_next_block(); break; }
            mount_last_block_ = mount_block_;
            mount_last_gen_   = mount_block_header_.generation;
            mount_pos_        = first_record_offset();
            mount_last_end_   = mount_pos_;
            mount_begin_record();
            break;

        case Op::mount_record_header_crc:
        {
            if (value != mount_record_.header_crc) { mount_block_end(); break; }
            const auto w = window(mount_record_address_ + payload_offset(), mount_record_.size);
            if (!w) { fail(Error::io); break; }
            begin_crc(Op::mount_record_payload_crc, *w);
            break;
        }

        case Op::mount_record_payload_crc:
            if (value != mount_record_.payload_crc) { mount_block_end(); }
            else { apply_mount_record(); }
            break;

        default:
            break;
        }
    }

public:
    explicit FileSystem(Nor& nor, Crc& crc) noexcept
        : nor_{nor}, crc_{crc}, g_{nor.geometry()}
    {
        blocks_ = g_.erase_size != 0U ? g_.total_size / g_.erase_size : 0U;
    }

    FileSystem(const FileSystem&)            = delete;
    FileSystem& operator=(const FileSystem&) = delete;

    /// Status of the last operation; Error::busy while one is running.
    [[nodiscard]] Error result() const noexcept     { return result_; }
    [[nodiscard]] bool  is_busy() const noexcept    { return busy(); }
    [[nodiscard]] bool  is_mounted() const noexcept { return mounted_; }

    /// ISR-safe: call from the NOR/CRC completion interrupts. Only records that
    /// hardware finished; the state machine runs in process().
    void irq() noexcept { irq_flag_.store(true, std::memory_order_release); }

    /// True if irq() was called since the last process() (e.g. to skip a WFI).
    [[nodiscard]] bool irq_pending() const noexcept
    {
        return irq_flag_.load(std::memory_order_acquire);
    }

    /// Advances the running operation by one step if the hardware it is
    /// waiting for has finished. Never blocks; call it from the main loop.
    void process() noexcept
    {
        irq_flag_.store(false, std::memory_order_relaxed);
        if (op_ == Op::idle && poison_requested_)
        {
            poison_requested_ = false;
            if (nor_.start_program(poison_address_, std::span<const std::byte>{detail::zero_word}))
            {
                op_ = Op::poison_program;
            }
            return;
        }
        switch (waits_on(op_))
        {
        case Waits::nor:
        {
            const AsyncState s = nor_.state();
            if (s == AsyncState::error)
            {
                if (op_ == Op::poison_program && !poison_mount_) { op_ = Op::idle; }
                else { fail(Error::io); }
            }
            else if (s == AsyncState::done) { on_nor_done(); }
            break;
        }
        case Waits::crc:
        {
            const AsyncState s = crc_.state();
            if (s == AsyncState::error) { fail(Error::crc); }
            else if (s == AsyncState::done) { on_crc_done(); }
            break;
        }
        case Waits::none:
            break;
        }
    }

    // ------------------------------------------------------------------
    // Operations. Each returns Error::ok when started (completion: is_busy()
    // false, outcome in result()), Error::busy if another operation is
    // running, Error::retry if a new block is being prepared first (repeat the
    // call once idle), or an error. Error::no_space: run gc_start(), repeat.
    // ------------------------------------------------------------------

    [[nodiscard]] Error format_start() noexcept
    {
        if (!geometry_ok()) { return Error::bad_geometry; }
        if (busy()) { return Error::busy; }
        mounted_         = false;
        result_          = Error::busy;
        clear_files();
        gc_              = Gc{};
        pending_         = Pending{};
        pending_new_file_.reset();
        init_pending_    = false;
        active_block_    = detail::npos;
        write_pos_       = 0U;
        next_generation_ = 1U;
        next_sequence_   = 1U;
        format_block_    = 0U;
        poison_requested_ = false;
        poison_mount_     = false;
        begin_erase(Op::format_erase, 0U);
        return op_ == Op::idle ? result_ : Error::ok;
    }

    [[nodiscard]] Error mount_start() noexcept
    {
        if (!geometry_ok()) { return Error::bad_geometry; }
        if (busy()) { return Error::busy; }
        mounted_          = false;
        result_           = Error::busy;
        clear_files();
        gc_               = Gc{};
        pending_          = Pending{};
        pending_new_file_.reset();
        init_pending_     = false;
        active_block_     = detail::npos;
        write_pos_        = 0U;
        next_sequence_    = 1U;
        mount_have_prev_  = false;
        mount_prev_gen_   = 0U;
        mount_prev_block_ = 0U;
        mount_last_block_ = detail::npos;
        mount_last_gen_   = 0U;
        mount_last_end_   = first_record_offset();
        reclaim_block_    = 0U;
        poison_requested_ = false;
        poison_mount_     = false;
        mount_next_block();
        return op_ == Op::idle ? result_ : Error::ok;
    }

    [[nodiscard]] Error create_start(std::string_view name) noexcept
    {
        if (!mounted_) { return Error::not_mounted; }
        if (busy()) { return Error::busy; }
        if (name.empty() || name.size() >= MaxName) { return Error::name_too_long; }
        if (std::ranges::find(name, '\0') != name.end()) { return Error::invalid; }
        if (find_file(name)) { return Error::exists; }

        if (!reserve(rec_total(static_cast<std::uint32_t>(name.size())))) { return rollover_start(); }

        const auto id = alloc_file(name);
        if (!id) { return Error::too_many_files; }
        pending_new_file_ = id;
        result_           = Error::busy;
        // The payload is the file table's own copy of the name, so the caller's
        // string need not outlive this call.
        begin_pending(detail::rec_create, static_cast<std::uint32_t>(*id), 0U,
                      std::as_bytes(std::span{files_[*id].name.data(), name.size()}), Origin::user);
        return op_ == Op::idle ? result_ : Error::ok;
    }

    /// Appends one record. `data` must stay valid until is_busy() is false.
    /// For more than MaxDataPerRecord bytes call repeatedly with consecutive offsets.
    [[nodiscard]] Error write_start(std::string_view name, std::uint32_t offset,
                                    std::span<const std::byte> data) noexcept
    {
        if (!mounted_) { return Error::not_mounted; }
        if (busy()) { return Error::busy; }
        const auto id = find_file(name);
        if (!id) { return Error::not_found; }
        if (data.empty()) { result_ = Error::ok; return Error::ok; }
        if (data.size() > MaxDataPerRecord || offset > MaxFileSize ||
            data.size() > MaxFileSize - offset) { return Error::invalid; }

        if (!reserve(rec_total(static_cast<std::uint32_t>(data.size())))) { return rollover_start(); }

        result_ = Error::busy;
        begin_pending(detail::rec_data, static_cast<std::uint32_t>(*id), offset, data, Origin::user);
        return op_ == Op::idle ? result_ : Error::ok;
    }

    [[nodiscard]] Error remove_start(std::string_view name) noexcept
    {
        if (!mounted_) { return Error::not_mounted; }
        if (busy()) { return Error::busy; }
        const auto id = find_file(name);
        if (!id) { return Error::not_found; }

        if (!reserve(rec_total(0U))) { return rollover_start(); }

        result_ = Error::busy;
        begin_pending(detail::rec_remove, static_cast<std::uint32_t>(*id), 0U, {}, Origin::user);
        return op_ == Op::idle ? result_ : Error::ok;
    }

    [[nodiscard]] Error gc_start() noexcept
    {
        if (!mounted_) { return Error::not_mounted; }
        if (busy()) { return Error::busy; }
        if (!select_gc()) { return Error::no_space; }
        result_ = Error::busy;
        start_block_init(gc_.destination, true);
        return op_ == Op::idle ? result_ : Error::ok;
    }

    // ------------------------------------------------------------------
    // Queries (synchronous, read the memory-mapped flash)
    // ------------------------------------------------------------------

    [[nodiscard]] Error read(std::string_view name, std::uint32_t offset,
                             std::span<std::byte> out) const noexcept
    {
        if (!mounted_) { return Error::not_mounted; }
        const auto id = find_file(name);
        if (!id) { return Error::not_found; }
        const FileEntry& f = files_[*id];
        if (offset > f.size || out.size() > f.size - offset) { return Error::invalid; }

        std::size_t done = 0;
        while (done < out.size())
        {
            Source src{};
            const Error e = locate(static_cast<std::uint32_t>(*id),
                                   offset + static_cast<std::uint32_t>(done),
                                   static_cast<std::uint32_t>(out.size() - done), src);
            if (e != Error::ok) { return e; }
            const auto w = window(src.address, src.size);
            if (!w) { return Error::io; }
            std::ranges::copy(*w, out.begin() + static_cast<std::ptrdiff_t>(done));
            done += src.size;
        }
        return Error::ok;
    }

    /// Zero-copy view of the flash. Empty unless the whole range is served by
    /// one record (nothing newer overlaps it). Valid until the next gc_start().
    [[nodiscard]] std::span<const std::byte> view(std::string_view name, std::uint32_t offset,
                                                  std::uint32_t size) const noexcept
    {
        if (!mounted_ || size == 0U) { return {}; }
        const auto id = find_file(name);
        if (!id) { return {}; }
        const FileEntry& f = files_[*id];
        if (offset > f.size || size > f.size - offset) { return {}; }

        Source src{};
        if (locate(static_cast<std::uint32_t>(*id), offset, size, src) != Error::ok || src.size != size)
        {
            return {};
        }
        const auto w = window(src.address, src.size);
        return w ? *w : std::span<const std::byte>{};
    }

    [[nodiscard]] std::optional<std::uint32_t> size(std::string_view name) const noexcept
    {
        const auto id = find_file(name);
        if (!id) { return std::nullopt; }
        return files_[*id].size;
    }

    template <class F>
        requires std::invocable<F&, std::string_view, std::uint32_t>
    void for_each(F&& f) const
    {
        for (const FileEntry& e : files_)
        {
            if (e.used && !e.deleted) { f(name_of(e), e.size); }
        }
    }
};

} // namespace norfs