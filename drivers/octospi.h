#pragma once

#include "stm32u3xx.h"
#include "gpdma.h"
#include "gpio.h"
#include <atomic>
#include <concepts>
#include <cstdint>
#include <cstddef>
#include <span>

namespace stm32u3::qspi {
namespace pins {

inline constexpr gpio::pin_config af10_cfg{
    .mode = gpio::mode::af, .speed = gpio::speed::very_high, .af = 10};
inline constexpr gpio::pin_config af3_cfg{
    .mode = gpio::mode::af, .speed = gpio::speed::very_high, .af = 3};
inline constexpr gpio::pin_config out_cfg{
    .mode = gpio::mode::output, .speed = gpio::speed::very_high};

inline void init_all() noexcept
{
    gpio::configure_pin<gpio::port_b>(1, af10_cfg); // DQ0
    gpio::configure_pin<gpio::port_b>(0, af10_cfg); // DQ1
    gpio::configure_pin<gpio::port_a>(7, af10_cfg); // DQ2
    gpio::configure_pin<gpio::port_a>(6, af10_cfg); // DQ3
    gpio::configure_pin<gpio::port_a>(3, af10_cfg); // CLK
    gpio::configure_pin<gpio::port_a>(4, af3_cfg);  // NCS
    gpio::configure_pin<gpio::port_a>(5, out_cfg);  // RESET
    gpio::pin<gpio::port_a, 5>::set();
      __DSB();
      __ISB();
}

} // namespace pins

enum class lines : uint32_t {
    none = 0,
    x1   = 1,
    x2   = 2,
    x4   = 3,
    x8   = 4,
};

enum class field_size : uint32_t {
    _8bit  = 0,
    _16bit = 1,
    _24bit = 2,
    _32bit = 3,
};

enum class dtr_mode : bool { disable = false, enable = true };

enum class functional_mode : uint32_t {
    indirect_write = 0,
    indirect_read  = 1,
    auto_polling   = 2,
    memory_mapped  = 3,
};

struct command {
    uint32_t   instruction;
    lines      inst_mode    = lines::x1;
    field_size inst_size    = field_size::_8bit;
    lines      addr_mode    = lines::none;
    field_size addr_size    = field_size::_24bit;
    lines      alt_mode     = lines::none;
    field_size alt_size     = field_size::_8bit;
    uint32_t   dummy_cycles = 0;
    lines      data_mode    = lines::none;
    dtr_mode   dtr          = dtr_mode::disable;
    bool       dqs          = false;
    bool       sioo         = false;

    // TCR_DCYC is a 5-bit field (0..31)
    [[nodiscard]] constexpr bool valid() const noexcept { return dummy_cycles <= 31U; }

    [[nodiscard]] constexpr bool has_address() const noexcept { return addr_mode != lines::none; }
    [[nodiscard]] constexpr bool has_data()    const noexcept { return data_mode != lines::none; }
    [[nodiscard]] constexpr bool has_alt()     const noexcept { return alt_mode  != lines::none; }

    [[nodiscard]] constexpr uint32_t ccr() const noexcept {
        return
            (static_cast<uint32_t>(inst_mode) << XSPI_CCR_IMODE_Pos) |
            (static_cast<uint32_t>(inst_size) << XSPI_CCR_ISIZE_Pos) |
            (static_cast<uint32_t>(addr_mode) << XSPI_CCR_ADMODE_Pos) |
            (static_cast<uint32_t>(addr_size) << XSPI_CCR_ADSIZE_Pos) |
            (static_cast<uint32_t>(alt_mode)  << XSPI_CCR_ABMODE_Pos) |
            (static_cast<uint32_t>(alt_size)  << XSPI_CCR_ABSIZE_Pos) |
            (static_cast<uint32_t>(data_mode) << XSPI_CCR_DMODE_Pos) |
            (dtr == dtr_mode::enable ? (XSPI_CCR_IDTR | XSPI_CCR_ADDTR |
                                        XSPI_CCR_ABDTR | XSPI_CCR_DDTR) : 0u) |
            (dqs  ? XSPI_CCR_DQSE : 0u) |
            (sioo ? XSPI_CCR_SIOO : 0u);
    }

    [[nodiscard]] constexpr uint32_t tcr() const noexcept {
        return (dummy_cycles << XSPI_TCR_DCYC_Pos) | XSPI_TCR_SSHIFT;
    }
};

struct device_config {
    uint32_t device_size_log2;
    uint32_t cs_high_time_cycles = 2;
    uint32_t clock_prescaler     = 0;
    bool     clock_mode3         = false;   // CKMODE: false=mode0, true=mode3

    [[nodiscard]] constexpr uint32_t dcr1() const noexcept {
        return (clock_mode3 ? XSPI_DCR1_CKMODE : 0u) |
               (cs_high_time_cycles << XSPI_DCR1_CSHT_Pos) |
               (device_size_log2 << XSPI_DCR1_DEVSIZE_Pos);
    }

    [[nodiscard]] constexpr uint32_t dcr2() const noexcept {
        return clock_prescaler << XSPI_DCR2_PRESCALER_Pos;
    }
};

// Device is passed as a *type* with `static constexpr device_config value`,
// so no class-type value is used as a template argument (IAR EDG bug).
template <class D>
concept device_traits = requires {
    { D::value } -> std::convertible_to<device_config>;
};

template <
    uintptr_t BaseAddr,
    IRQn_Type Irqn,
    device_traits Dev,
    gpdma::dma_channel RxChannel,
    gpdma::dma_channel TxChannel,
    uint32_t RxRequest,
    uint32_t TxRequest,
    uint32_t FifoThreshold = 3
>
requires (FifoThreshold <= 63U)   // XSPI_CR_FTHRES is a 6-bit field
class controller {

public:
    using completion_cb = void (*)(bool success) noexcept;

private:
    static constinit inline std::atomic<bool>          s_busy{false};
    static constinit inline std::atomic<completion_cb> s_callback{nullptr};

    static constexpr uint32_t base_cr =
        (FifoThreshold << XSPI_CR_FTHRES_Pos) |
        XSPI_CR_TEIE |
        XSPI_CR_TCIE;

    static constexpr uint32_t memory_mapped_cr =
        (FifoThreshold << XSPI_CR_FTHRES_Pos);

    static constexpr uint32_t dev_dcr1 = Dev::value.dcr1();
    static constexpr uint32_t dev_dcr2 = Dev::value.dcr2();

    static constexpr gpdma::config rx_cfg{
        .request   = RxRequest,
        .dir       = gpdma::direction::periph_to_mem,
        .src_width = gpdma::width::byte,
        .dst_width = gpdma::width::byte,
        .src_inc   = gpdma::increment::disable,
        .dst_inc   = gpdma::increment::enable,
        .prio      = gpdma::priority::high
    };

    static constexpr gpdma::config tx_cfg{
        .request   = TxRequest,
        .dir       = gpdma::direction::mem_to_periph,
        .src_width = gpdma::width::byte,
        .dst_width = gpdma::width::byte,
        .src_inc   = gpdma::increment::enable,
        .dst_inc   = gpdma::increment::disable,
        .prio      = gpdma::priority::high
    };

    [[gnu::always_inline]]
    static inline void wait_not_busy() noexcept {
        while (OCTOSPI1->SR & XSPI_SR_BUSY) { /* wait */ }
    }

    [[gnu::always_inline]]
    static inline void load_command(const command& cmd,
                                    uint32_t address,
                                    uint32_t alt_bytes,
                                    uint32_t data_length) noexcept
    {
        OCTOSPI1->CCR = cmd.ccr();
        OCTOSPI1->TCR = cmd.tcr();

        if (cmd.has_alt()) {
            OCTOSPI1->ABR = alt_bytes;
        }
        if (cmd.has_data()) {
            OCTOSPI1->DLR = data_length - 1;
        }
        if (cmd.has_address()) {
            OCTOSPI1->AR = address;
        }
        OCTOSPI1->IR = cmd.instruction;
    }

    [[gnu::always_inline]]
    static inline void set_functional_mode(functional_mode mode,
                                           bool dma) noexcept
    {
        OCTOSPI1->CR = base_cr
                 | (static_cast<uint32_t>(mode) << XSPI_CR_FMODE_Pos)
                 | (dma ? XSPI_CR_DMAEN : 0u)
                 | XSPI_CR_EN;
    }

    [[nodiscard]]
    static inline bool try_begin(completion_cb callback) noexcept {
        if (s_busy.exchange(true, std::memory_order_acq_rel))
            return false;
        s_callback.store(callback, std::memory_order_release);
        std::atomic_signal_fence(std::memory_order_seq_cst);
        return true;
    }

public:
    static inline void enable_clock() noexcept {
        RCC->AHB2ENR2 |= RCC_AHB2ENR2_OCTOSPI1EN;
        __DSB();
        __ISB();
    }

    static inline void init_delay_block(DLYB_TypeDef* dlyb) noexcept {
        OCTOSPI1->DCR1 |= XSPI_DCR1_FRCK;

        dlyb->CR |= DLYB_CR_DEN;
        dlyb->CFGR &= ~DLYB_CFGR_SEL;
        dlyb->CFGR &= ~DLYB_CFGR_UNIT;

        OCTOSPI1->CR |= XSPI_CR_ABORT;
        while (OCTOSPI1->CR & XSPI_CR_ABORT) { /* wait */ }

        OCTOSPI1->DCR1 &= ~XSPI_DCR1_FRCK;
    }

    static inline void init(DLYB_TypeDef* dlyb) noexcept {
        pins::init_all();
        enable_clock();
        OCTOSPI1->DCR1 = dev_dcr1;
        OCTOSPI1->DCR2 = dev_dcr2;
        OCTOSPI1->DCR3 = 0;
        OCTOSPI1->DCR4 = 0;
        __DSB();
        __ISB();
        OCTOSPI1->TCR = XSPI_TCR_SSHIFT;
        __DSB();
        __ISB();
        OCTOSPI1->CR = base_cr | XSPI_CR_EN;
        __DSB();
        __ISB();
        init_delay_block(dlyb);
        __DSB();
        __ISB();
        RxChannel::enable_clock();
        RxChannel::reset();
        TxChannel::enable_clock();
        TxChannel::reset();

        NVIC_SetPriority(Irqn, 5);
        NVIC_EnableIRQ(Irqn);
    }

    [[nodiscard]] static inline bool busy() noexcept {
        return s_busy.load(std::memory_order_acquire);
    }

    // All entry points return false if the command is invalid for the
    // operation (bad dummy cycles, data phase mismatch, ...).

    // Command with no data phase (write-enable, reset, erase, ...).
    [[nodiscard]]
    static inline bool send(const command& cmd,
                            uint32_t address = 0,
                            uint32_t alt_bytes = 0) noexcept
    {
        if (!cmd.valid() || cmd.has_data()) return false;

        wait_not_busy();
        set_functional_mode(functional_mode::indirect_write, /*dma=*/false);
        load_command(cmd, address, alt_bytes, 0);

        wait_not_busy();
        OCTOSPI1->FCR = XSPI_FCR_CTCF;
        return true;
    }

    [[nodiscard]]
    static inline bool read_blocking(const command& cmd,
                                     uint32_t address, uint32_t alt_bytes,
                                     std::span<uint8_t> dst) noexcept
    {
        if (!cmd.valid() || !cmd.has_data()) return false;
        if (dst.empty()) return true;   // DLR = length-1 would underflow

        wait_not_busy();
        set_functional_mode(functional_mode::indirect_read, /*dma=*/false);
        load_command(cmd, address, alt_bytes, static_cast<uint32_t>(dst.size()));

        auto* dr = reinterpret_cast<volatile uint8_t*>(&OCTOSPI1->DR);
        for (uint8_t& b : dst) {
            while (!(OCTOSPI1->SR & XSPI_SR_TCF) && (OCTOSPI1->SR & XSPI_SR_FLEVEL) == 0) {
                /* wait */
            }
            b = *dr;
        }
        wait_not_busy();
        OCTOSPI1->FCR = XSPI_FCR_CTCF;
        return true;
    }

    [[nodiscard]]
    static inline bool write_blocking(const command& cmd,
                                      uint32_t address, uint32_t alt_bytes,
                                      std::span<const uint8_t> src) noexcept
    {
        if (!cmd.valid() || !cmd.has_data()) return false;
        if (src.empty()) return true;   // DLR = length-1 would underflow

        wait_not_busy();
        set_functional_mode(functional_mode::indirect_write, false);
        load_command(cmd, address, alt_bytes, static_cast<uint32_t>(src.size()));

        auto* dr = reinterpret_cast<volatile uint8_t*>(&OCTOSPI1->DR);
        for (const uint8_t b : src) {
            while ((OCTOSPI1->SR & XSPI_SR_FLEVEL) >= 16) { /* wait */ }
            *dr = b;
        }
        wait_not_busy();
        OCTOSPI1->FCR = XSPI_FCR_CTCF;
        return true;
    }

    [[nodiscard]]
    static inline bool memory_mapped(const command& read,
                                     const command& write,
                                     uint32_t read_alt_bytes = 0,
                                     uint32_t write_alt_bytes = 0) noexcept
    {
        if (!read.valid()  || !read.has_data()  || !read.has_address() ||
            !write.valid() || !write.has_data() || !write.has_address())
            return false;

        wait_not_busy();

        /* Read configuration */
        OCTOSPI1->TCR = read.tcr();
        OCTOSPI1->CCR = read.ccr();

        if (read.has_alt()) {
            OCTOSPI1->ABR = read_alt_bytes;
        }

        OCTOSPI1->IR = read.instruction;

        /* Write configuration */
        OCTOSPI1->WTCR = write.tcr();
        OCTOSPI1->WCCR = write.ccr();

        if (write.has_alt()) {
            OCTOSPI1->WABR = write_alt_bytes;
        }

        OCTOSPI1->WIR = write.instruction;

        /* FMODE = 3 -> memory-mapped */
        OCTOSPI1->CR =
            memory_mapped_cr |
            (static_cast<uint32_t>(functional_mode::memory_mapped)
             << XSPI_CR_FMODE_Pos) |
            XSPI_CR_EN;

        __DSB();
        __ISB();
        return true;
    }

    template<typename T = uint8_t>
    [[nodiscard]]
    static inline volatile T* memory_base() noexcept
    {
        return reinterpret_cast<volatile T*>(BaseAddr);
    }

    // -------------------------------------------------------------------
    //  GPDMA + interrupt driven read/write.
    // -------------------------------------------------------------------
    [[nodiscard]]
    static inline bool read_async(const command& cmd,
                                  uint32_t address, uint32_t alt_bytes,
                                  std::span<uint8_t> dst,
                                  completion_cb callback) noexcept
    {
        if (!cmd.valid() || !cmd.has_data()) return false;
        if (dst.empty() || !try_begin(callback)) return false;

        RxChannel::template configure<rx_cfg>();

        wait_not_busy();

        RxChannel::start_raw(
            reinterpret_cast<uintptr_t>(&OCTOSPI1->DR),
            reinterpret_cast<uintptr_t>(dst.data()),
            static_cast<uint32_t>(dst.size()));

        set_functional_mode(functional_mode::indirect_read, /*dma=*/true);
        load_command(cmd, address, alt_bytes, static_cast<uint32_t>(dst.size()));

        return true;
    }

    [[nodiscard]]
    static inline bool write_async(const command& cmd,
                                   uint32_t address, uint32_t alt_bytes,
                                   std::span<const uint8_t> src,
                                   completion_cb callback) noexcept
    {
        if (!cmd.valid() || !cmd.has_data()) return false;
        if (src.empty() || !try_begin(callback)) return false;

        TxChannel::template configure<tx_cfg>();

        wait_not_busy();

        TxChannel::start_raw(
            reinterpret_cast<uintptr_t>(src.data()),
            reinterpret_cast<uintptr_t>(&OCTOSPI1->DR),
            static_cast<uint32_t>(src.size()));

        set_functional_mode(functional_mode::indirect_write, /*dma=*/true);
        load_command(cmd, address, alt_bytes, static_cast<uint32_t>(src.size()));

        return true;
    }

    static inline void irq_handler() noexcept {
        const uint32_t sr = OCTOSPI1->SR;
        const bool ok  = (sr & XSPI_SR_TCF) != 0;
        const bool err = (sr & XSPI_SR_TEF) != 0;

        if (!ok && !err) return; // spurious/other-flag IRQ, ignore

        OCTOSPI1->FCR = XSPI_FCR_CTCF | XSPI_FCR_CTEF;
        OCTOSPI1->CR &= ~(XSPI_CR_DMAEN);

        // Take the callback *before* releasing busy, so a new transfer
        // started from a higher-priority IRQ can't have its callback
        // stolen by this completion.
        const completion_cb cb = s_callback.exchange(nullptr, std::memory_order_acq_rel);
        s_busy.store(false, std::memory_order_release);
        if (cb) {
            cb(ok && !err);
        }
    }
};

} // namespace stm32u3::qspi