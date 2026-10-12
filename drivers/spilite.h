#pragma once

#include <atomic>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>
#include <type_traits>
#include "stm32u3xx.h"
#include "gpdma.h"

namespace stm32u3::spi
{

inline constexpr uint32_t gpdma_req_spi1_rx = 6;
inline constexpr uint32_t gpdma_req_spi1_tx = 7;

inline constexpr gpdma::config spi1_tx_dma_cfg{
    .request   = gpdma_req_spi1_tx,
    .dir       = gpdma::direction::mem_to_periph,
    .src_width = gpdma::width::byte,
    .dst_width = gpdma::width::byte,
    .src_inc   = gpdma::increment::enable,
    .dst_inc   = gpdma::increment::disable,
    .prio      = gpdma::priority::medium
};

inline constexpr gpdma::config spi1_tx_dma_noinc_cfg{
    .request   = gpdma_req_spi1_tx,
    .dir       = gpdma::direction::mem_to_periph,
    .src_width = gpdma::width::byte,
    .dst_width = gpdma::width::byte,
    .src_inc   = gpdma::increment::disable,
    .dst_inc   = gpdma::increment::disable,
    .prio      = gpdma::priority::medium
};

inline constexpr gpdma::config spi1_rx_dma_cfg{
    .request   = gpdma_req_spi1_rx,
    .dir       = gpdma::direction::periph_to_mem,
    .src_width = gpdma::width::byte,
    .dst_width = gpdma::width::byte,
    .src_inc   = gpdma::increment::disable,
    .dst_inc   = gpdma::increment::enable,
    .prio      = gpdma::priority::medium
};

inline constexpr gpdma::config spi1_rx_dma_noinc_cfg{
    .request   = gpdma_req_spi1_rx,
    .dir       = gpdma::direction::periph_to_mem,
    .src_width = gpdma::width::byte,
    .dst_width = gpdma::width::byte,
    .src_inc   = gpdma::increment::disable,
    .dst_inc   = gpdma::increment::disable,
    .prio      = gpdma::priority::medium
};

using spi1_tx_channel = gpdma::channel<0>;
using spi1_rx_channel = gpdma::channel<1>;

struct spi1
{
    [[nodiscard]] static SPI_TypeDef* regs() noexcept { return SPI1; }
    static constexpr IRQn_Type irqn = SPI1_IRQn;

    using tx_channel = spi1_tx_channel;
    using rx_channel = spi1_rx_channel;

    static constexpr gpdma::config tx_cfg        = spi1_tx_dma_cfg;
    static constexpr gpdma::config tx_noinc_cfg  = spi1_tx_dma_noinc_cfg;
    static constexpr gpdma::config rx_cfg        = spi1_rx_dma_cfg;
    static constexpr gpdma::config rx_noinc_cfg  = spi1_rx_dma_noinc_cfg;

    static inline void enable_clock() noexcept
    {
        RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
        __DSB();
        __ISB();
    }
};

enum class prescaler : uint32_t
{
    div2   = 0,
    div4   = SPI_CFG1_MBR_0,
    div8   = SPI_CFG1_MBR_1,
    div16  = SPI_CFG1_MBR_1 | SPI_CFG1_MBR_0,
    div32  = SPI_CFG1_MBR_2,
    div64  = SPI_CFG1_MBR_2 | SPI_CFG1_MBR_0,
    div128 = SPI_CFG1_MBR_2 | SPI_CFG1_MBR_1,
    div256 = SPI_CFG1_MBR
};

enum class phase : uint32_t
{
    first  = 0,
    second = SPI_CFG2_CPHA
};

enum class polarity : uint32_t
{
    low  = 0,
    high = SPI_CFG2_CPOL
};

enum class datasize : uint32_t
{
    bits8  = 7,
    bits16 = 15
};

enum class nss_mode : uint32_t
{
    software = SPI_CFG2_SSM,
    hardware = SPI_CFG2_SSOE
};

enum class drdy_level
{
    active_low,
    active_high
};

struct master_config
{
    spi::prescaler  prescaler = spi::prescaler::div2;
    spi::phase      phase     = spi::phase::first;
    spi::polarity   polarity  = spi::polarity::low;
    spi::datasize   datasize  = spi::datasize::bits8;
    spi::nss_mode   nss       = spi::nss_mode::software;
    spi::drdy_level drdy      = spi::drdy_level::active_low;
};

// ------------------------- concepts -------------------------

template<class I>
concept spi_instance = requires
{
    { I::regs() } -> std::convertible_to<SPI_TypeDef*>;
    { I::irqn }   -> std::convertible_to<IRQn_Type>;
    I::enable_clock();
    requires gpdma::dma_channel<typename I::tx_channel>;
    requires gpdma::dma_channel<typename I::rx_channel>;
    { I::tx_cfg }       -> std::convertible_to<gpdma::config>;
    { I::tx_noinc_cfg } -> std::convertible_to<gpdma::config>;
    { I::rx_cfg }       -> std::convertible_to<gpdma::config>;
    { I::rx_noinc_cfg } -> std::convertible_to<gpdma::config>;
};

template<class T>
concept af_pin = requires { T::init(); };

template<class T>
concept output_pin = requires { T::init(); T::set(); T::reset(); };

template<class T>
concept drdy_pin = requires
{
    T::init();
    { T::read() }         -> std::same_as<bool>;
    { T::exti_pending() } -> std::same_as<bool>;
    T::exti_clear();
};

template<
    spi_instance Instance,
    af_pin Sck,
    af_pin Miso,
    af_pin Mosi,
    class Nss  = void,
    class Drdy = void,
    master_config Cfg = master_config{}>
requires (std::is_void_v<Nss>  || output_pin<Nss>) &&
         (std::is_void_v<Drdy> || drdy_pin<Drdy>)
class master
{
    static constexpr bool has_nss  = !std::is_void_v<Nss>;
    static constexpr bool has_drdy = !std::is_void_v<Drdy>;

    using tx_channel = typename Instance::tx_channel;
    using rx_channel = typename Instance::rx_channel;

    // CR2.TSIZE is 16 bits wide
    static constexpr std::size_t tsize_max = 0xFFFFu;

public:
    using callback_t = void (*)(bool success);

    static inline void init() noexcept
    {
        Instance::enable_clock();
        
        Sck::init();
        Miso::init();
        Mosi::init();

        if constexpr (has_nss)
        {
            Nss::init();
        }

        if constexpr (has_drdy)
        {
            Drdy::init();
            Drdy::exti_clear();
        }

        auto* const spi = Instance::regs();
        spi->CR1 &= ~SPI_CR1_SPE;

        uint32_t cfg2 = SPI_CFG2_MASTER |
                        static_cast<uint32_t>(Cfg.phase) |
                        static_cast<uint32_t>(Cfg.polarity);

        if constexpr (Cfg.nss == nss_mode::hardware)
        {
            cfg2 |= SPI_CFG2_SSOE;
        }
        else
        {
            cfg2 |= SPI_CFG2_SSM;
            spi->CR1 |= SPI_CR1_SSI;
        }

        constexpr uint32_t cfg1 =
            static_cast<uint32_t>(Cfg.prescaler) |
            (static_cast<uint32_t>(Cfg.datasize) << SPI_CFG1_DSIZE_Pos);

        spi->CFG1 = cfg1;
        spi->CFG2 = cfg2;
        spi->AUTOCR = 0;

        NVIC_SetPriority(Instance::irqn, 5);
        NVIC_EnableIRQ(Instance::irqn);

        spi->CR1 |= SPI_CR1_SPE;
    }

    [[nodiscard]] static inline bool is_drdy_asserted() noexcept
    {
        if constexpr (!has_drdy)
        {
            return true;
        }
        else if constexpr (Cfg.drdy == drdy_level::active_low)
        {
            return !Drdy::read();
        }
        else
        {
            return Drdy::read();
        }
    }

    static inline void drdy_exti_handler() noexcept
    {
        if constexpr (has_drdy)
        {
            if (Drdy::exti_pending())
            {
                Drdy::exti_clear();
                if (is_drdy_asserted() &&
                    waiting_drdy_.exchange(false, std::memory_order_acq_rel))
                {
                    start_dma_fn_();
                }
            }
        }
    }

    // ---- Blocking API ----

    [[nodiscard]]
    static bool transmit(std::span<const uint8_t> data,
                         uint32_t timeout_ticks = 100000) noexcept
    {
        if (data.empty() || data.size() > tsize_max)
            return false;

        auto* const spi  = Instance::regs();
        auto* const txdr = reinterpret_cast<volatile uint8_t*>(&spi->TXDR);
        auto* const rxdr = reinterpret_cast<volatile uint8_t*>(&spi->RXDR);

        begin_blocking(
            data.size(),
            SPI_IFCR_EOTC |
            SPI_IFCR_TXTFC |
            SPI_IFCR_OVRC |
            SPI_IFCR_UDRC);

        spi->CR1 |= SPI_CR1_SPE;

        const uint8_t* tx = data.data();
        std::size_t tx_cnt = data.size();
        std::size_t rx_cnt = data.size();

        *txdr = *tx++;
        --tx_cnt;

        spi->CR1 |= SPI_CR1_CSTART;

        uint32_t timeout = timeout_ticks;

        while (tx_cnt > 0 || rx_cnt > 0)
        {
            bool progress = false;

            if (tx_cnt > 0 && (spi->SR & SPI_SR_TXP))
            {
                *txdr = *tx++;
                --tx_cnt;
                progress = true;
            }

            if (rx_cnt > 0 && (spi->SR & SPI_SR_RXP))
            {
                (void)*rxdr;
                --rx_cnt;
                progress = true;
            }

            if (progress)
            {
                timeout = timeout_ticks;
            }
            else if (--timeout == 0)
            {
                abort_blocking();
                return false;
            }
        }

        if (!wait_for(SPI_SR_EOT, timeout_ticks))
        {
            abort_blocking();
            return false;
        }
        spi->IFCR = SPI_IFCR_EOTC | SPI_IFCR_TXTFC;
        spi->CR1 &= ~SPI_CR1_SPE;

        return true;
    }

    [[nodiscard]]
    static bool transmit(uint8_t byte, uint32_t timeout_ticks = 100000) noexcept
    {
        return transmit(std::span<const uint8_t>{&byte, 1}, timeout_ticks);
    }

    [[nodiscard]]
    static bool transmit_receive(std::span<const uint8_t> tx_data,
                                 std::span<uint8_t> rx_data,
                                 uint32_t timeout_ticks = 100000) noexcept
    {
        const std::size_t size = tx_data.size();
        if (size == 0 || size != rx_data.size() || size > tsize_max) return false;

        auto* const spi  = Instance::regs();
        auto* const txdr = reinterpret_cast<volatile uint8_t*>(&spi->TXDR);
        auto* const rxdr = reinterpret_cast<volatile uint8_t*>(&spi->RXDR);

        begin_blocking(size, SPI_IFCR_EOTC | SPI_IFCR_TXTFC | SPI_IFCR_OVRC | SPI_IFCR_UDRC);

        spi->CR1 |= SPI_CR1_SPE;

        const uint8_t* tx = tx_data.data();
        uint8_t*       rx = rx_data.data();

        *txdr = *tx++;
        std::size_t tx_cnt = size - 1;
        std::size_t rx_cnt = size;

        spi->CR1 |= SPI_CR1_CSTART;

        // The timeout counts consecutive spins without progress; any byte
        // moved re-arms it.
        uint32_t timeout = timeout_ticks;
        while (tx_cnt > 0 || rx_cnt > 0)
        {
            bool progress = false;

            if ((tx_cnt > 0) && (spi->SR & SPI_SR_TXP)) {
                *txdr = *tx++;
                --tx_cnt;
                progress = true;
            }

          if ((rx_cnt > 0) && (spi->SR & SPI_SR_RXP) &&
                (rx_cnt > 1 || (spi->SR & SPI_SR_EOT)))
            {
                *rx++ = *rxdr;
                --rx_cnt;
                progress = true;
            }

            if (progress) {
                timeout = timeout_ticks;
            } else if (--timeout == 0) {
                abort_blocking();
                return false;
            }
        }

        if (!wait_for(SPI_SR_EOT, timeout_ticks)) { abort_blocking(); return false; }
        spi->IFCR = SPI_IFCR_EOTC;
        spi->CR1 &= ~SPI_CR1_SPE;
        return true;
    }

    // ---- Non-blocking DMA API ----

    [[nodiscard]]
    static inline bool transmit_dma(std::span<const uint8_t> data,
                                    callback_t on_complete = nullptr) noexcept
    {
        return dma_transfer<Instance::tx_cfg, Instance::rx_noinc_cfg>(
            data.data(), nullptr, data.size(), on_complete);
    }

    [[nodiscard]]
    static inline bool receive_dma(std::span<uint8_t> data,
                                   callback_t on_complete = nullptr) noexcept
    {
        return dma_transfer<Instance::tx_noinc_cfg, Instance::rx_cfg>(
            nullptr, data.data(), data.size(), on_complete);
    }

    [[nodiscard]]
    static inline bool transmit_receive_dma(std::span<const uint8_t> tx_data,
                                            std::span<uint8_t> rx_data,
                                            callback_t on_complete = nullptr) noexcept
    {
        if (tx_data.size() != rx_data.size()) return false;
        return dma_transfer<Instance::tx_cfg, Instance::rx_cfg>(
            tx_data.data(), rx_data.data(), tx_data.size(), on_complete);
    }

    [[nodiscard]] static inline bool busy() noexcept
    {
        return busy_.load(std::memory_order_acquire);
    }

    static inline void irq_handler() noexcept
    {
        auto* const spi = Instance::regs();
        const uint32_t sr  = spi->SR;
        const uint32_t ier = spi->IER;
        const uint32_t irq = sr & ier;
        if (irq & (SPI_SR_OVR | SPI_SR_MODF | SPI_SR_UDR | SPI_SR_TIFRE))
        {
            spi->IER = 0;
            spi->IFCR = SPI_IFCR_OVRC | SPI_IFCR_MODFC | SPI_IFCR_UDRC | SPI_IFCR_TIFREC;
            finish_dma(false);
            return;
        }

        if (irq & SPI_SR_EOT)
        {
            spi->IER = 0;
            spi->IFCR = SPI_IFCR_EOTC | SPI_IFCR_TXTFC;
            finish_dma(true);
        }
    }

private:

    // ---- blocking helpers ----

    static inline void begin_blocking(std::size_t size, uint32_t ifcr) noexcept
    {
        auto* const spi = Instance::regs();
        spi->CR1 &= ~SPI_CR1_SPE;
        spi->IFCR = ifcr;
        spi->CR2 = (spi->CR2 & ~SPI_CR2_TSIZE) |
                   ((static_cast<uint32_t>(size) << SPI_CR2_TSIZE_Pos) & SPI_CR2_TSIZE);
    }

    [[nodiscard]]
    static inline bool wait_for(uint32_t sr_flag, uint32_t timeout) noexcept
    {
        auto* const spi = Instance::regs();
        while (!(spi->SR & sr_flag))
        {
            if (--timeout == 0) return false;
        }
        return true;
    }

    static inline void abort_blocking() noexcept
    {
        Instance::regs()->CR1 &= ~SPI_CR1_SPE;
    }

    // ---- DMA helpers ----

    using start_dma_fn_t = void (*)() noexcept;

    template<gpdma::config TxCfg, gpdma::config RxCfg>
    [[nodiscard]]
    static inline bool dma_transfer(const uint8_t* tx_data,
                                    uint8_t* rx_data,
                                    std::size_t size,
                                    callback_t on_complete) noexcept
    {
        if (size == 0 || size > tsize_max) return false;

        if (busy_.exchange(true, std::memory_order_acq_rel)) return false;

        callback_ = on_complete;
        pending_tx_data_ = tx_data;
        pending_rx_data_ = rx_data;
        pending_size_    = static_cast<uint16_t>(size);
        start_dma_fn_    = &start_dma_impl<TxCfg, RxCfg>;
        std::atomic_signal_fence(std::memory_order_seq_cst);

        if (is_drdy_asserted())
        {
            start_dma_fn_();
            return true;
        }

        if constexpr (has_drdy)
        {
            Drdy::exti_clear();
            waiting_drdy_.store(true, std::memory_order_release);

            if (is_drdy_asserted() &&
                waiting_drdy_.exchange(false, std::memory_order_acq_rel))
            {
                start_dma_fn_();
            }
        }

        return true;
    }

    template<gpdma::config TxCfg, gpdma::config RxCfg>
    static inline void start_dma_impl() noexcept
    {
        auto* const spi = Instance::regs();

        if constexpr (Cfg.nss == nss_mode::software && has_nss)
        {
            Nss::reset();
        }
        spi->CR1 &= ~SPI_CR1_SPE;
        spi->IFCR = 0xFFFFFFFF;
        spi->CR2  = pending_size_;
        RCC->AHB1ENR1 |= RCC_AHB1ENR1_GPDMA1EN; 
        __DSB(); __ISB();
        tx_channel::enable_clock();
        rx_channel::enable_clock();

        tx_channel::template configure<TxCfg>();
        rx_channel::template configure<RxCfg>();

        const uintptr_t rx_dst = pending_rx_data_
            ? reinterpret_cast<uintptr_t>(pending_rx_data_)
            : reinterpret_cast<uintptr_t>(&dummy_rx_byte_);
        const uintptr_t tx_src = pending_tx_data_
            ? reinterpret_cast<uintptr_t>(pending_tx_data_)
            : reinterpret_cast<uintptr_t>(&dummy_tx_byte_);

        rx_channel::start_raw(
            reinterpret_cast<uintptr_t>(&spi->RXDR),
            rx_dst,
            pending_size_);

        tx_channel::start_raw(
            tx_src,
            reinterpret_cast<uintptr_t>(&spi->TXDR),
            pending_size_);

        spi->IER = SPI_IER_EOTIE | SPI_IER_OVRIE | SPI_IER_MODFIE;
        spi->CFG1 |= SPI_CFG1_RXDMAEN | SPI_CFG1_TXDMAEN;

        spi->CR1 |= SPI_CR1_SPE;
        spi->CR1 |= SPI_CR1_CSTART;
    }

    static inline void finish_dma(bool ok) noexcept
    {
        auto* const spi = Instance::regs();

        tx_channel::stop();
        rx_channel::stop();
        spi->CFG1 &= ~(SPI_CFG1_RXDMAEN | SPI_CFG1_TXDMAEN);

        spi->CR1 &= ~SPI_CR1_SPE;

        if constexpr (Cfg.nss == nss_mode::software && has_nss)
        {
            Nss::set();
        }

        waiting_drdy_.store(false, std::memory_order_relaxed);

        const callback_t cb = callback_;
        callback_ = nullptr;
        busy_.store(false, std::memory_order_release);

        if (cb)
            cb(ok);
    }

    static constinit inline std::atomic<bool> busy_{false};
    static constinit inline std::atomic<bool> waiting_drdy_{false};
    static inline callback_t callback_ = nullptr;

    static inline const uint8_t *pending_tx_data_ = nullptr;
    static inline uint8_t *pending_rx_data_ = nullptr;
    static inline uint16_t pending_size_ = 0;
    static inline start_dma_fn_t start_dma_fn_ = nullptr;

    static inline const uint8_t dummy_tx_byte_ = 0xFF;
    static inline volatile uint8_t dummy_rx_byte_ = 0x00;
};

} // namespace stm32u3::spi