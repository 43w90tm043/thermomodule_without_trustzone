#pragma once

/// @file fdcan_protocol.h
/// @brief hcan: lightweight application protocol on top of CAN FD (29-bit IDs).
///
/// Design notes
///  - Header-only, no heap, no exceptions, no RTTI, no virtual dispatch.
///  - Every function is noexcept; errors are reported through Result.
///  - All buffers are fixed-size and owned by the caller / the objects below.
///  - Views (std::span) handed out by the receive path are NON-OWNING; their
///    lifetime is documented on each type.
///  - Wire format is little-endian and independent of the host byte order.

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <type_traits>

namespace hcan
{

/// Millisecond tick, wraps at 2^32 (all comparisons are wrap-safe).
using Timestamp = std::uint32_t;

// ============================================================================
// Configuration
// ============================================================================

struct Config
{
    static constexpr std::size_t MaxCanData          = 64;
    static constexpr std::size_t TransportHeaderSize = 4;
    static constexpr std::size_t ServiceHeaderSize   = 2;

    static constexpr std::size_t MaxTransportPayload =
        MaxCanData - TransportHeaderSize;

    /// Sequence 0 is reserved for the first frame, so a transfer can use at
    /// most 255 frames before the 8-bit sequence counter would wrap.
    static constexpr std::size_t MaxFrames = 256;
    static constexpr std::size_t MaxTransferSize =
        (MaxFrames - 1U) * MaxTransportPayload;

    static constexpr std::size_t MaxReassemblySlots = 8;

    static constexpr std::uint8_t  BroadcastNode  = 127;
    static constexpr std::uint32_t ExtendedIdMask = 0x1FFF'FFFFU;
};

static_assert(Config::MaxCanData == 64U);
static_assert(Config::MaxFrames <= 256U, "sequence number is 8 bits wide");
static_assert(Config::MaxTransportPayload > 0U);


// ============================================================================
// CAN FD frame abstraction
// ============================================================================

struct CanFdFrame
{
    std::uint32_t id     = 0;
    std::uint8_t  length = 0;
    std::array<std::uint8_t, Config::MaxCanData> data{};

    /// Valid payload bytes (length is clamped to the buffer size).
    [[nodiscard]] constexpr std::span<const std::uint8_t> payload() const noexcept
    {
        return std::span<const std::uint8_t>{data}.first(
            std::min<std::size_t>(length, Config::MaxCanData));
    }
};

static_assert(std::is_trivially_copyable_v<CanFdFrame>);


// ============================================================================
// Byte helpers
// ============================================================================

namespace detail
{

[[nodiscard]] constexpr std::uint16_t
get_u16_le(std::span<const std::uint8_t, 2> in) noexcept
{
    return static_cast<std::uint16_t>(
        static_cast<unsigned>(in[0]) |
        (static_cast<unsigned>(in[1]) << 8U));
}

constexpr void
put_u16_le(std::span<std::uint8_t, 2> out, std::uint16_t value) noexcept
{
    out[0] = static_cast<std::uint8_t>(value & 0xFFU);
    out[1] = static_cast<std::uint8_t>(value >> 8U);
}

} // namespace detail

/// Object representation of a trivially copyable value (host byte order).
/// Convenience for building payloads without pointer casts.
template <class T>
    requires std::is_trivially_copyable_v<T>
[[nodiscard]] constexpr std::array<std::uint8_t, sizeof(T)>
to_bytes(const T& value) noexcept
{
    return std::bit_cast<std::array<std::uint8_t, sizeof(T)>>(value);
}

/// Inverse of to_bytes(); empty optional if the size does not match exactly.
template <class T>
    requires std::is_trivially_copyable_v<T>
[[nodiscard]] constexpr std::optional<T>
from_bytes(std::span<const std::uint8_t> bytes) noexcept
{
    if (bytes.size() != sizeof(T))
    {
        return std::nullopt;
    }

    std::array<std::uint8_t, sizeof(T)> raw{};
    std::ranges::copy(bytes, raw.begin());
    return std::bit_cast<T>(raw);
}


// ============================================================================
// Message kinds
// ============================================================================

enum class Kind : std::uint8_t
{
    Publish   = 0,
    Request   = 1,
    Response  = 2,
    Transport = 3
};


// ============================================================================
// CAN protocol ID
//
// 28..26 : priority       3 bits
// 25..24 : kind           2 bits
// 23..14 : message ID    10 bits
// 13..7  : source         7 bits
//  6..0  : destination    7 bits
// ============================================================================

struct MessageId
{
    static constexpr unsigned PriorityShift = 26U;
    static constexpr unsigned KindShift     = 24U;
    static constexpr unsigned MessageShift  = 14U;
    static constexpr unsigned SourceShift   = 7U;
    static constexpr unsigned DestShift     = 0U;

    static constexpr std::uint32_t PriorityMask = 0x07U;
    static constexpr std::uint32_t KindMask     = 0x03U;
    static constexpr std::uint32_t MessageMask  = 0x3FFU;
    static constexpr std::uint32_t NodeMask     = 0x7FU;

    std::uint8_t  priority    = 0;
    Kind          kind        = Kind::Publish;
    std::uint16_t message_id  = 0;
    std::uint8_t  source      = 0;
    std::uint8_t  destination = 0;

    [[nodiscard]] friend constexpr bool
    operator==(const MessageId&, const MessageId&) noexcept = default;

    [[nodiscard]] constexpr std::uint32_t encode() const noexcept
    {
        return ((static_cast<std::uint32_t>(priority)   & PriorityMask) << PriorityShift) |
               ((static_cast<std::uint32_t>(kind)       & KindMask)     << KindShift)     |
               ((static_cast<std::uint32_t>(message_id) & MessageMask)  << MessageShift)  |
               ((static_cast<std::uint32_t>(source)     & NodeMask)     << SourceShift)   |
               ((static_cast<std::uint32_t>(destination) & NodeMask)    << DestShift);
    }

    /// Total: every 29-bit pattern maps to a MessageId (all four Kind values
    /// are defined). Bits above 28 are ignored.
    [[nodiscard]] static constexpr MessageId decode(std::uint32_t id) noexcept
    {
        return MessageId{
            .priority    = static_cast<std::uint8_t>((id >> PriorityShift) & PriorityMask),
            .kind        = static_cast<Kind>((id >> KindShift) & KindMask),
            .message_id  = static_cast<std::uint16_t>((id >> MessageShift) & MessageMask),
            .source      = static_cast<std::uint8_t>((id >> SourceShift) & NodeMask),
            .destination = static_cast<std::uint8_t>((id >> DestShift) & NodeMask)};
    }

    /// True if every field fits its bit width (i.e. encode() is lossless).
    [[nodiscard]] constexpr bool valid() const noexcept
    {
        return priority <= PriorityMask &&
               static_cast<std::uint32_t>(kind) <= KindMask &&
               message_id <= MessageMask &&
               source <= NodeMask &&
               destination <= NodeMask;
    }
};

static_assert(MessageId::decode(MessageId{.priority = 5, .kind = Kind::Response,
                                          .message_id = 0x2AB, .source = 0x12,
                                          .destination = 0x7F}.encode()) ==
              MessageId{.priority = 5, .kind = Kind::Response,
                        .message_id = 0x2AB, .source = 0x12, .destination = 0x7F});
static_assert(MessageId{.priority = 7, .kind = Kind::Transport, .message_id = 0x3FF,
                        .source = 0x7F, .destination = 0x7F}.encode() ==
              Config::ExtendedIdMask);


// ============================================================================
// Transport header (little-endian on the wire)
//
// byte 0 : flags  (bit7 = start, bit6 = end, bits5..0 reserved, must be 0)
// byte 1 : sequence
// byte 2..3 : transfer ID
// ============================================================================

struct TransportHeader
{
    static constexpr std::size_t Size = Config::TransportHeaderSize;

    static constexpr std::uint8_t StartFlag    = 0x80U;
    static constexpr std::uint8_t EndFlag      = 0x40U;
    static constexpr std::uint8_t ReservedMask = 0x3FU;

    std::uint8_t  flags       = 0;
    std::uint8_t  sequence    = 0;
    std::uint16_t transfer_id = 0;

    [[nodiscard]] constexpr bool is_start() const noexcept
    {
        return (flags & StartFlag) != 0U;
    }

    [[nodiscard]] constexpr bool is_end() const noexcept
    {
        return (flags & EndFlag) != 0U;
    }

    [[nodiscard]] constexpr bool has_reserved_bits() const noexcept
    {
        return (flags & ReservedMask) != 0U;
    }

    constexpr void encode(std::span<std::uint8_t, Size> out) const noexcept
    {
        out[0] = flags;
        out[1] = sequence;
        detail::put_u16_le(out.subspan<2, 2>(), transfer_id);
    }

    [[nodiscard]] static constexpr TransportHeader
    decode(std::span<const std::uint8_t, Size> in) noexcept
    {
        return TransportHeader{
            .flags       = in[0],
            .sequence    = in[1],
            .transfer_id = detail::get_u16_le(in.subspan<2, 2>())};
    }
};


// ============================================================================
// Result
// ============================================================================

enum class Result : std::uint8_t
{
    Ok,

    InvalidArgument,
    InvalidFrame,
    InvalidId,

    NotForUs,

    UnexpectedFrame,
    WrongSequence,
    Duplicate,

    NoReassemblySlot,
    TransferTooLarge,

    Timeout
};


// ============================================================================
// TX helpers (stateless, allocation-free)
//
// On any non-Ok result the output frame is left untouched.
// Unused bytes of the frame are zero.
// ============================================================================

class Tx
{
public:
    Tx() = delete;

    [[nodiscard]] static constexpr Result make_publish(
        std::uint8_t                      priority,
        std::uint16_t                     message_id,
        std::uint8_t                      source,
        std::uint8_t                      destination,
        std::span<const std::uint8_t>     payload,
        CanFdFrame&                       frame) noexcept
    {
        if (payload.size() > Config::MaxCanData)
        {
            return Result::InvalidArgument;
        }

        const MessageId id{.priority    = priority,
                           .kind        = Kind::Publish,
                           .message_id  = message_id,
                           .source      = source,
                           .destination = destination};

        if (!id.valid())
        {
            return Result::InvalidId;
        }

        CanFdFrame out{};
        out.id     = id.encode();
        out.length = static_cast<std::uint8_t>(payload.size());
        std::ranges::copy(payload, out.data.begin());

        frame = out;
        return Result::Ok;
    }

    /// Request: [txn id (2, LE)] [payload...]
    [[nodiscard]] static constexpr Result make_request(
        std::uint8_t                  priority,
        std::uint16_t                 service_id,
        std::uint8_t                  source,
        std::uint8_t                  destination,
        std::uint16_t                 transaction_id,
        std::span<const std::uint8_t> payload,
        CanFdFrame&                   frame) noexcept
    {
        return make_service(Kind::Request, priority, service_id, source,
                            destination, transaction_id, payload, frame);
    }

    /// Response: [txn id (2, LE)] [payload...]
    [[nodiscard]] static constexpr Result make_response(
        std::uint8_t                  priority,
        std::uint16_t                 service_id,
        std::uint8_t                  source,
        std::uint8_t                  destination,
        std::uint16_t                 transaction_id,
        std::span<const std::uint8_t> payload,
        CanFdFrame&                   frame) noexcept
    {
        return make_service(Kind::Response, priority, service_id, source,
                            destination, transaction_id, payload, frame);
    }

    /// One frame of a segmented transfer. `id.kind` must be Kind::Transport.
    /// The first segment must have sequence 0; later segments must not.
    [[nodiscard]] static constexpr Result make_segment(
        MessageId                     id,
        std::uint16_t                 transfer_id,
        std::uint8_t                  sequence,
        bool                          first,
        bool                          last,
        std::span<const std::uint8_t> payload,
        CanFdFrame&                   frame) noexcept
    {
        if (id.kind != Kind::Transport || !id.valid())
        {
            return Result::InvalidId;
        }

        if (payload.size() > Config::MaxTransportPayload)
        {
            return Result::InvalidArgument;
        }

        if (first != (sequence == 0U))
        {
            return Result::InvalidArgument;
        }

        TransportHeader header{};
        header.flags = static_cast<std::uint8_t>(
            (first ? TransportHeader::StartFlag : 0U) |
            (last  ? TransportHeader::EndFlag   : 0U));
        header.sequence    = sequence;
        header.transfer_id = transfer_id;

        CanFdFrame out{};
        out.id = id.encode();
        header.encode(std::span{out.data}.first<TransportHeader::Size>());
        std::ranges::copy(payload, out.data.begin() + TransportHeader::Size);
        out.length = static_cast<std::uint8_t>(TransportHeader::Size + payload.size());

        frame = out;
        return Result::Ok;
    }

private:
    [[nodiscard]] static constexpr Result make_service(
        Kind                          kind,
        std::uint8_t                  priority,
        std::uint16_t                 service_id,
        std::uint8_t                  source,
        std::uint8_t                  destination,
        std::uint16_t                 transaction_id,
        std::span<const std::uint8_t> payload,
        CanFdFrame&                   frame) noexcept
    {
        if (payload.size() > Config::MaxCanData - Config::ServiceHeaderSize)
        {
            return Result::InvalidArgument;
        }

        const MessageId id{.priority    = priority,
                           .kind        = kind,
                           .message_id  = service_id,
                           .source      = source,
                           .destination = destination};

        if (!id.valid())
        {
            return Result::InvalidId;
        }

        CanFdFrame out{};
        out.id = id.encode();
        detail::put_u16_le(std::span{out.data}.first<Config::ServiceHeaderSize>(),
                           transaction_id);
        std::ranges::copy(payload, out.data.begin() + Config::ServiceHeaderSize);
        out.length = static_cast<std::uint8_t>(Config::ServiceHeaderSize + payload.size());

        frame = out;
        return Result::Ok;
    }
};


// ============================================================================
// Reassembly
// ============================================================================

/// A finished segmented transfer. `payload` points into the reassembler's
/// storage and is valid only until the next call to process() / expire().
struct CompleteTransfer
{
    MessageId                     id{};
    std::uint16_t                 transfer_id = 0;
    std::span<const std::uint8_t> payload{};
};

template <std::size_t MaxPayload = Config::MaxTransferSize,
          std::size_t Slots      = Config::MaxReassemblySlots>
class Reassembler
{
    static_assert(MaxPayload > 0U, "MaxPayload must be non-zero");
    static_assert(MaxPayload <= Config::MaxTransferSize,
                  "MaxPayload exceeds what the sequence counter can address");
    static_assert(Slots > 0U, "at least one reassembly slot is required");

public:
    /// Feed one received frame.
    ///  - Ok with `out` empty : frame accepted, transfer still in progress
    ///  - Ok with `out` set   : transfer complete
    ///  - anything else       : frame rejected (and the transfer dropped if
    ///                          it was a sequence/size violation)
    [[nodiscard]] Result process(const CanFdFrame&               frame,
                                 Timestamp                       now,
                                 std::optional<CompleteTransfer>& out) noexcept
    {
        out.reset();

        if (frame.length > Config::MaxCanData ||
            frame.length < TransportHeader::Size)
        {
            return Result::InvalidFrame;
        }

        const MessageId id = MessageId::decode(frame.id);
        if (id.kind != Kind::Transport)
        {
            return Result::InvalidFrame;
        }

        const std::span<const std::uint8_t> bytes = frame.payload();
        const TransportHeader header =
            TransportHeader::decode(bytes.first<TransportHeader::Size>());

        if (header.has_reserved_bits())
        {
            return Result::InvalidFrame;
        }

        const auto chunk = bytes.subspan(TransportHeader::Size);

        return header.is_start() ? begin_transfer(id, header, chunk, now, out)
                                 : continue_transfer(id, header, chunk, now, out);
    }

    /// Drop transfers that have been idle for more than `timeout_ms`.
    void expire(Timestamp now, std::uint32_t timeout_ms) noexcept
    {
        for (Slot& slot : slots_)
        {
            if (slot.active &&
                static_cast<std::uint32_t>(now - slot.last_timestamp) > timeout_ms)
            {
                release(slot);
            }
        }
    }

private:
    struct Slot
    {
        bool          active            = false;
        MessageId     id{};
        std::uint16_t transfer_id       = 0;
        std::uint8_t  expected_sequence = 0;
        std::size_t   size              = 0;
        Timestamp     last_timestamp    = 0;
        std::array<std::uint8_t, MaxPayload> buffer{};
    };

    std::array<Slot, Slots> slots_{};

    [[nodiscard]] Slot* find_slot(const MessageId& id, std::uint16_t transfer_id) noexcept
    {
        const auto it = std::ranges::find_if(slots_, [&](const Slot& s) {
            return s.active &&
                   s.id.source      == id.source &&
                   s.id.destination == id.destination &&
                   s.id.message_id  == id.message_id &&
                   s.transfer_id    == transfer_id;
        });
        return it != slots_.end() ? &*it : nullptr;
    }

    [[nodiscard]] Slot* allocate_slot(const MessageId& id,
                                      std::uint16_t    transfer_id,
                                      Timestamp        now) noexcept
    {
        const auto it = std::ranges::find_if(
            slots_, [](const Slot& s) { return !s.active; });
        if (it == slots_.end())
        {
            return nullptr;
        }

        it->active            = true;
        it->id                = id;
        it->transfer_id       = transfer_id;
        it->expected_sequence = 0;
        it->size              = 0;
        it->last_timestamp    = now;
        return &*it;
    }

    static void release(Slot& slot) noexcept
    {
        slot.active = false;
        slot.size   = 0;
    }

    static Result finish(Slot& slot, std::optional<CompleteTransfer>& out) noexcept
    {
        out = CompleteTransfer{
            .id          = slot.id,
            .transfer_id = slot.transfer_id,
            .payload     = std::span<const std::uint8_t>{slot.buffer}.first(slot.size)};
        release(slot);   // buffer contents stay intact until the slot is reused
        return Result::Ok;
    }

    [[nodiscard]] Result begin_transfer(const MessageId&                 id,
                                        const TransportHeader&           header,
                                        std::span<const std::uint8_t>    chunk,
                                        Timestamp                        now,
                                        std::optional<CompleteTransfer>& out) noexcept
    {
        if (header.sequence != 0U)
        {
            return Result::InvalidFrame;
        }

        if (find_slot(id, header.transfer_id) != nullptr)
        {
            return Result::Duplicate;   // never silently overwrite a live transfer
        }

        if (chunk.size() > MaxPayload)
        {
            return Result::TransferTooLarge;
        }

        Slot* const slot = allocate_slot(id, header.transfer_id, now);
        if (slot == nullptr)
        {
            return Result::NoReassemblySlot;
        }

        std::ranges::copy(chunk, slot->buffer.begin());
        slot->size              = chunk.size();
        slot->expected_sequence = 1;

        return header.is_end() ? finish(*slot, out) : Result::Ok;
    }

    [[nodiscard]] Result continue_transfer(const MessageId&                 id,
                                           const TransportHeader&           header,
                                           std::span<const std::uint8_t>    chunk,
                                           Timestamp                        now,
                                           std::optional<CompleteTransfer>& out) noexcept
    {
        Slot* const slot = find_slot(id, header.transfer_id);
        if (slot == nullptr)
        {
            return Result::UnexpectedFrame;
        }

        if (header.sequence != slot->expected_sequence)
        {
            release(*slot);
            return Result::WrongSequence;
        }

        if (chunk.size() > MaxPayload - slot->size)
        {
            release(*slot);
            return Result::TransferTooLarge;
        }

        std::ranges::copy(chunk, slot->buffer.begin() + static_cast<std::ptrdiff_t>(slot->size));
        slot->size += chunk.size();
        ++slot->expected_sequence;   // cannot wrap: MaxPayload bounds the frame count
        slot->last_timestamp = now;

        return header.is_end() ? finish(*slot, out) : Result::Ok;
    }
};


// ============================================================================
// Main protocol receiver
// ============================================================================

template <std::size_t MaxTransfer     = Config::MaxTransferSize,
          std::size_t ReassemblySlots = Config::MaxReassemblySlots>
class Protocol
{
public:
    using Rx = Reassembler<MaxTransfer, ReassemblySlots>;

    /// Result of receive(). `payload` is a non-owning view:
    ///  - Publish/Request/Response : points into the frame passed to receive()
    ///  - TransferComplete         : points into the reassembler; valid until
    ///                               the next receive() or tick()
    /// For TransferComplete, `transaction_id` carries the transfer ID.
    struct Event
    {
        enum class Type : std::uint8_t
        {
            None,
            Publish,
            Request,
            Response,
            TransferComplete,
            Error
        };

        Type                          type           = Type::None;
        MessageId                     id{};
        std::uint16_t                 transaction_id = 0;
        std::span<const std::uint8_t> payload{};
        Result                        error          = Result::Ok;
    };

    explicit constexpr Protocol(std::uint8_t local_node) noexcept
        : local_node_{local_node}
    {
    }

    [[nodiscard]] Event receive(const CanFdFrame& frame, Timestamp now) noexcept
    {
        if (frame.id > Config::ExtendedIdMask)
        {
            return make_error(Result::InvalidId);
        }

        if (frame.length > Config::MaxCanData)
        {
            return make_error(Result::InvalidFrame);
        }

        const MessageId id = MessageId::decode(frame.id);

        // Destination filtering; broadcast is accepted by everyone.
        if (id.destination != local_node_ &&
            id.destination != Config::BroadcastNode)
        {
            return Event{};
        }

        const std::span<const std::uint8_t> bytes = frame.payload();

        switch (id.kind)
        {
        case Kind::Publish:
            return Event{.type = Event::Type::Publish, .id = id, .payload = bytes};

        case Kind::Request:
            return parse_service(Event::Type::Request, id, bytes);

        case Kind::Response:
            return parse_service(Event::Type::Response, id, bytes);

        case Kind::Transport:
            return receive_transport(frame, now);
        }

        return make_error(Result::InvalidFrame);   // unreachable for valid Kind
    }

    void tick(Timestamp now, std::uint32_t timeout_ms) noexcept
    {
        reassembler_.expire(now, timeout_ms);
    }

private:
    [[nodiscard]] static constexpr Event make_error(Result error) noexcept
    {
        return Event{.type = Event::Type::Error, .error = error};
    }

    [[nodiscard]] static Event parse_service(typename Event::Type type,
                                             const MessageId&     id,
                                             std::span<const std::uint8_t> bytes) noexcept
    {
        if (bytes.size() < Config::ServiceHeaderSize)
        {
            return make_error(Result::InvalidFrame);
        }

        return Event{
            .type           = type,
            .id             = id,
            .transaction_id = detail::get_u16_le(bytes.first<Config::ServiceHeaderSize>()),
            .payload        = bytes.subspan(Config::ServiceHeaderSize)};
    }

    [[nodiscard]] Event receive_transport(const CanFdFrame& frame, Timestamp now) noexcept
    {
        std::optional<CompleteTransfer> transfer{};

        const Result r = reassembler_.process(frame, now, transfer);
        if (r != Result::Ok)
        {
            return make_error(r);
        }

        // Ok with no transfer: intermediate frame accepted.
        if (!transfer)
        {
            return Event{};
        }

        return Event{.type           = Event::Type::TransferComplete,
                     .id             = transfer->id,
                     .transaction_id = transfer->transfer_id,
                     .payload        = transfer->payload};
    }

    std::uint8_t local_node_;
    Rx           reassembler_{};
};

} // namespace hcan