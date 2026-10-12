#pragma once

#include "stm32u3xx.h"
#include <concepts>
#include <cstdint>
#include <cstddef>
#include <span>
#include <type_traits>

namespace stm32u3::gpdma {

enum class width : uint32_t {
    byte     = 0,
    halfword = 1,
    word     = 2
};

enum class increment : bool {
    disable = false,
    enable  = true
};

enum class direction {
    mem_to_mem,
    mem_to_periph,
    periph_to_mem
};

enum class priority : uint32_t {
    low       = 0,
    medium    = 1,
    high      = 2,
    very_high = 3
};

struct config {
    uint32_t          request   = 0;
    gpdma::direction  dir       = gpdma::direction::mem_to_mem;
    gpdma::width      src_width = gpdma::width::byte;
    gpdma::width      dst_width = gpdma::width::byte;
    gpdma::increment  src_inc   = gpdma::increment::disable;
    gpdma::increment  dst_inc   = gpdma::increment::disable;
    gpdma::priority   prio      = gpdma::priority::low;

    [[nodiscard]] constexpr uint32_t ctr1() const noexcept {
        return (static_cast<uint32_t>(src_width) << DMA_CTR1_SDW_LOG2_Pos) |
               (static_cast<uint32_t>(dst_width) << DMA_CTR1_DDW_LOG2_Pos) |
               (src_inc == increment::enable ? DMA_CTR1_SINC : 0u) |
               (dst_inc == increment::enable ? DMA_CTR1_DINC : 0u);
    }

    [[nodiscard]] constexpr uint32_t ctr2() const noexcept {
        return (dir == direction::mem_to_mem)
            ? DMA_CTR2_SWREQ
            : ((request << DMA_CTR2_REQSEL_Pos) |
               (dir == direction::mem_to_periph ? DMA_CTR2_DREQ : 0u));
    }

    [[nodiscard]] constexpr uint32_t ccr() const noexcept {
        return DMA_CCR_TCIE |
               DMA_CCR_DTEIE |
               DMA_CCR_USEIE |
               DMA_CCR_ULEIE |
               (static_cast<uint32_t>(prio) << DMA_CCR_PRIO_Pos);
    }
};

template<uint32_t Channel>
requires (Channel < 12U)   // GPDMA1 supports channels 0..11
class channel {

    static constexpr uintptr_t address =
        GPDMA1_BASE + 0x50u + Channel * 0x80u;

public:
    [[nodiscard]]
    [[gnu::always_inline]]
    static inline DMA_Channel_TypeDef* regs() noexcept {
        return reinterpret_cast<DMA_Channel_TypeDef*>(address);
    }

    [[gnu::always_inline]]
    static inline void enable_clock() noexcept {
        RCC->AHB1ENR1 |= RCC_AHB1ENR1_GPDMA1EN;
        __DSB();
    }

    [[gnu::always_inline]]
    static inline void reset() noexcept {
        regs()->CCR |= DMA_CCR_RESET;
    }

    [[gnu::always_inline]]
    static inline void clear_flags() noexcept {
        regs()->CFCR =
            DMA_CFCR_TCF   |
            DMA_CFCR_HTF   |
            DMA_CFCR_DTEF  |
            DMA_CFCR_ULEF  |
            DMA_CFCR_USEF  |
            DMA_CFCR_SUSPF |
            DMA_CFCR_TOF;
    }

    [[gnu::always_inline]]
    static inline void stop() noexcept {
        regs()->CCR &= ~DMA_CCR_EN;
    }

    template<config Cfg>
    [[gnu::always_inline]]
    static inline void configure() noexcept {
        constexpr uint32_t ctr1 = Cfg.ctr1();
        constexpr uint32_t ctr2 = Cfg.ctr2();
        constexpr uint32_t ccr  = Cfg.ccr();

        stop();
        clear_flags();

        regs()->CTR1  = ctr1;
        regs()->CTR2  = ctr2;
        regs()->CTR3  = 0;
        regs()->CBR2  = 0;
        regs()->CLBAR = 0;
        regs()->CLLR  = 0;
        regs()->CCR   = ccr;
    }

    template<typename T>
    requires std::is_trivially_copyable_v<T>
    [[gnu::always_inline]]
    static inline void start(
        std::span<const T> src,
        volatile T* dst) noexcept
    {
        stop();
        clear_flags();

        regs()->CSAR = reinterpret_cast<uintptr_t>(src.data());
        regs()->CDAR = reinterpret_cast<uintptr_t>(dst);
        regs()->CBR1 = static_cast<uint32_t>(src.size_bytes());

        __DSB();
        regs()->CCR |= DMA_CCR_EN;
    }

    [[gnu::always_inline]]
    static inline void start_raw(
        uintptr_t src,
        uintptr_t dst,
        uint32_t bytes) noexcept
    {
        stop();
        clear_flags();

        regs()->CSAR = src;
        regs()->CDAR = dst;
        regs()->CBR1 = bytes;

        __DSB();

        regs()->CCR |= DMA_CCR_EN;
    }

    [[nodiscard]]
    [[gnu::always_inline]]
    static inline bool complete() noexcept {
        return (regs()->CSR & DMA_CSR_TCF) != 0U;
    }

    [[nodiscard]]
    [[gnu::always_inline]]
    static inline bool error() noexcept {
        return (regs()->CSR &
            (DMA_CSR_DTEF |
             DMA_CSR_USEF |
             DMA_CSR_ULEF |
             DMA_CSR_TOF)) != 0U;
    }

    [[gnu::always_inline]]
    static inline void clear_complete() noexcept {
        regs()->CFCR = DMA_CFCR_TCF;
    }

    [[gnu::always_inline]]
    static inline void clear_errors() noexcept {
        regs()->CFCR =
            DMA_CFCR_DTEF |
            DMA_CFCR_USEF |
            DMA_CFCR_ULEF |
            DMA_CFCR_TOF;
    }

    [[nodiscard]]
    [[gnu::always_inline]]
    static inline bool busy() noexcept {
        return (regs()->CSR & DMA_CSR_IDLEF) == 0U;
    }
};

template<class C>
concept dma_channel = requires(uintptr_t a, uint32_t n)
{
    C::enable_clock();
    C::reset();
    C::stop();
    C::start_raw(a, a, n);
    C::template configure<config{}>();
};

} // namespace stm32u3::gpdma