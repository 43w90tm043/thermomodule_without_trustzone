#include "stm32u3xx.h"
#include "clock.h"
#include "gpio.h"
#include "swo.h"
#include "rng.h"
#include "rtc.h"
#include "wdg.h"
#include "crc.h"
#include "spilite.h"
#include "ads1262.h"
#include "thermo.h"   
#include "octospi.h"
#include "w25q512jvfim.h"
#include "file_system.h"
#include "fdcan.h"
#include "fdcan_protocol.h"
#include "fdcan_control.h"
#include "arm_math.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

using namespace stm32u3;

namespace cfg
{
inline constexpr gpio::pin_config out_high{
    .mode = gpio::mode::output, .speed = gpio::speed::high};

inline constexpr gpio::pin_config out_very_high{
    .mode = gpio::mode::output, .speed = gpio::speed::very_high};

inline constexpr gpio::pin_config fdcan_af{
    .mode = gpio::mode::af, .speed = gpio::speed::high, .af = 9};

inline constexpr gpio::pin_config spi_af{
    .mode = gpio::mode::af, .speed = gpio::speed::very_high, .af = 5};

inline constexpr gpio::pin_config spi_af_pullup{
    .mode = gpio::mode::af, .speed = gpio::speed::very_high,
    .pull = gpio::pull::up, .af = 5};
} // namespace cfg

namespace
{
constexpr uint8_t  k_local_node          = 1;
constexpr uint32_t k_adc_setup_attempts  = 3;
constexpr uint32_t k_hcan_tick_period_ms = 1000;

template <class T, std::size_t N>
class spsc_queue
{
    static_assert(std::has_single_bit(N), "N must be a power of two");
    static_assert(std::atomic<uint32_t>::is_always_lock_free);

public:
    [[nodiscard]] bool push(const T& v) noexcept // producer (ISR)
    {
        const uint32_t h = head_.load(std::memory_order_relaxed);
        const uint32_t t = tail_.load(std::memory_order_acquire);
        if (h - t >= N) { return false; }

        buf_[h & mask] = v;
        head_.store(h + 1U, std::memory_order_release);
        return true;
    }

    [[nodiscard]] std::optional<T> pop() noexcept // consumer (main loop)
    {
        const uint32_t t = tail_.load(std::memory_order_relaxed);
        const uint32_t h = head_.load(std::memory_order_acquire);
        if (t == h) { return std::nullopt; }

        const T v = buf_[t & mask];
        tail_.store(t + 1U, std::memory_order_release);
        return v;
    }

private:
    static constexpr uint32_t mask = static_cast<uint32_t>(N - 1U);

    std::array<T, N>      buf_{};
    std::atomic<uint32_t> head_{0};
    std::atomic<uint32_t> tail_{0};
};

} // namespace

// ============================================================================
//  Flash W25Q512JV (OCTOSPI1 + GPDMA)
// ============================================================================
namespace w25q = qspi::w25q512jv;

using flash_rx_dma = gpdma::channel<9>;
using flash_tx_dma = gpdma::channel<10>;

constexpr uint32_t octospi1_gpdma_req = 29U;

using flash_controller = qspi::controller<
    OCTOSPI1_BASE,
    OCTOSPI1_IRQn,
    w25q::device_traits,
    flash_rx_dma,
    flash_tx_dma,
    octospi1_gpdma_req,
    octospi1_gpdma_req>;

using flash = w25q::driver<flash_controller>;

static void test_memory_mapped_addresses()
{
    if (const auto e = flash::enable_memory_mapped(); e != w25q::error::ok)
    {
        swo::out << "flash memory-mapped failed, err="
                 << static_cast<uint32_t>(e) << "\r\n";
        return;
    }

    const auto map = flash::memory_map();

    constexpr std::array<uint32_t, 5> addresses{
        0x00000000UL, 0x00100000UL, 0x01000000UL, 0x02000000UL, 0x03FFFFFCUL};

    for (const uint32_t address : addresses)
    {
        if (address >= map.size()) { continue; }

        const uint8_t value = map[address];

        swo::out << "MM[" << swo::hex(address) << "] = "
                 << swo::hex(static_cast<uint32_t>(value)) << "\r\n";
    }
}

// ============================================================================
//  CAN FD
// ============================================================================
using fdcan_tx   = gpio::pin<gpio::port_a, 12, cfg::fdcan_af>;
using fdcan_rx   = gpio::pin<gpio::port_a, 11, cfg::fdcan_af>;
using fdcan_stb  = gpio::pin<gpio::port_a, 10, cfg::out_high>;
using fdcan_gpio = gpio::pin<gpio::port_a,  9, cfg::out_high>;

static void fdcan_pins_init()
{
    fdcan_tx::init();
    fdcan_rx::init();
    fdcan_stb::init();
    fdcan_stb::reset();   // transceiver out of standby
    fdcan_gpio::init();
}

[[nodiscard]] static hcan::CanFdFrame to_hcan(const fdcan::Frame& msg)
{
    hcan::CanFdFrame frame{};
    frame.id     = msg.id;
    frame.length = msg.length;
    std::copy_n(msg.data.data(), msg.length, &frame.data[0]);
    return frame;
}

[[nodiscard]] static std::optional<fdcan::Frame> to_fdcan(const hcan::CanFdFrame& frame)
{
    return fdcan::make_frame(
        frame.id,
        std::span<const uint8_t>{&frame.data[0], static_cast<std::size_t>(frame.length)},
        fdcan::BitRateSwitch::Off);
}

using HcanProtocol = hcan::Protocol<4096, 1>;

static HcanProtocol g_hcan{k_local_node};

[[nodiscard]] static bool fdcan_send_temperature(float temperature_celsius)
{
    hcan::CanFdFrame frame{};

    const auto result = hcan::Tx::make_publish(
        4,
        0x001,
        k_local_node,
        hcan::Config::BroadcastNode,
        hcan::to_bytes(temperature_celsius),
        frame);

    if (result != hcan::Result::Ok) { return false; }

    const auto tx = to_fdcan(frame);
    if (!tx) { return false; }

    if (const auto e = fdcan::Fdcan1::send(*tx); e != fdcan::Error::Ok)
    {
        swo::out << "can tx error=" << static_cast<uint32_t>(e) << "\r\n";
        return false;
    }
    return true;
}

static void print_can_frame(const fdcan::Frame& msg)
{
    swo::out << "CAN RX: ID=0x" << swo::hex(msg.id)
             << " DLC=0x" << swo::hex(static_cast<uint32_t>(msg.length))
             << " DATA=";

    for (const uint8_t b : msg.payload())
    {
        swo::out << swo::hex(static_cast<uint32_t>(b)) << " ";
    }
    swo::out << "\r\n";
}

static void handle_hcan_event(const HcanProtocol::Event& event)
{
    using Type = HcanProtocol::Event::Type;

    switch (event.type)
    {
    case Type::Publish:
        swo::out << "hcan publish, id=" << swo::hex(event.id.message_id) << "\r\n";
        break;
    case Type::Request:
        swo::out << "hcan request\r\n";
        break;
    case Type::Response:
        swo::out << "hcan response\r\n";
        break;
    case Type::TransferComplete:
        swo::out << "hcan transfer complete, size="
                 << static_cast<uint32_t>(event.payload.size()) << "\r\n";
        break;
    case Type::Error:
        swo::out << "hcan error: " << static_cast<uint32_t>(event.error) << "\r\n";
        break;
    case Type::None:
    default:
        break;
    }
}

static void service_can()
{
      static uint32_t last_ev = 0;
     const auto ev = fdcan::Fdcan1::take_events();
     if (ev != fdcan::BusEvent::None && fdcan::to_underlying(ev) != last_ev) {
        swo::out << "can events=" << swo::hex(fdcan::to_underlying(ev)) << "\r\n";

        if (fdcan::has(ev, fdcan::BusEvent::BusOff))
        {
            fdcan::Fdcan1::restart_after_bus_off();
        } }
 

    // Received frames (bounded by the queue depth).
    while (const auto rx = fdcan::Fdcan1::receive())
    {
        print_can_frame(*rx);
        handle_hcan_event(g_hcan.receive(to_hcan(*rx), clock::millis().count()));
    }
}

// ============================================================================
//  ADS1262 + PT1000
// ============================================================================

using adc_start = gpio::pin<gpio::port_b,  7, cfg::out_high>;
using adc_reset = gpio::pin<gpio::port_b,  6, cfg::out_high>;
using adc_mosi  = gpio::pin<gpio::port_b,  5, cfg::spi_af>;
using adc_miso  = gpio::pin<gpio::port_b,  4, cfg::spi_af_pullup>;
using adc_nss   = gpio::pin<gpio::port_a, 15, cfg::out_very_high>;
using adc_sck   = gpio::pin<gpio::port_a,  1, cfg::spi_af>;

namespace cfg
{
inline constexpr gpio::pin_config adc_drdy_in{
    .mode    = gpio::mode::input,
    .exti    = gpio::exti_mode::interrupt,
    .trigger = gpio::exti_trigger::falling};
} 
using adc_drdy = gpio::pin<gpio::port_a, 2, cfg::adc_drdy_in>;

inline constexpr spi::master_config adc_spi_cfg{
    .prescaler = spi::prescaler::div32,
    .phase     = spi::phase::second,
    .polarity  = spi::polarity::low,
    .datasize  = spi::datasize::bits8,
    .nss       = spi::nss_mode::software,
    .drdy      = spi::drdy_level::active_low};

using my_spi = spi::master<
    spi::spi1,
    adc_sck, adc_miso, adc_mosi, adc_nss, adc_drdy,
    adc_spi_cfg>;

inline constexpr drivers::hw_options adc_hw_opts{
    .spi_drdy_gated = true, .cs_setup_us = 1};

using adc = drivers::ads1262<
    my_spi,
    adc_nss,
    adc_drdy,
    adc_start,
    adc_reset,
    adc_hw_opts>;

using pt1000 = sensors::pt1000_channel<adc>;

namespace
{
constinit spsc_queue<sensors::reading, 8> g_readings;
constinit std::atomic<uint32_t>           g_dropped_readings{0};
} // namespace

static void on_reading(const sensors::reading& r) noexcept
{
    if (!g_readings.push(r))
    {
        g_dropped_readings.fetch_add(1U, std::memory_order_relaxed);
    }
}

[[nodiscard]] static bool start_temperature_channel()
{
    for (uint32_t attempt = 1; attempt <= k_adc_setup_attempts; ++attempt)
    {
        auto e = pt1000::setup();                       // default channel_config
        if (e == sensors::pt1000_error::ok) { e = pt1000::start_streaming(&on_reading); }
        if (e == sensors::pt1000_error::ok)
        {
            swo::out << "pt1000 ok, rref_ohms="
                     << static_cast<uint32_t>(pt1000::rref_ohms()) << "\r\n";
            return true;
        }
        const auto m = adc::last_mismatch();
        swo::out << "pt1000 setup failed, attempt=" << attempt
                 << " err=" << static_cast<uint32_t>(e)
                 << " adc_err=" << static_cast<uint32_t>(pt1000::last_adc_error())
                   << "mismatch reg=" << swo::hex(uint32_t{m.reg})
                   << " exp=" << swo::hex(uint32_t{m.expected})
                   << " got=" << swo::hex(uint32_t{m.actual}) 
                   << " step=" << static_cast<uint32_t>(pt1000::last_setup_step())
                   << " rref_x10=" << static_cast<uint32_t>(static_cast<int32_t>(pt1000::rref_ohms() * 10.0F))<< "\r\n";
    }
    return false;
}

static void service_temperature()
{
    while (const auto r = g_readings.pop())
    {
        using sensors::sample_status;

        switch (r->status)
        {
    case sample_status::ok:
    if (!fdcan_send_temperature(r->celsius))
    {
        swo::out << "temperature publish failed\r\n";
    }
    break;

        case sample_status::settling:
            break; // expected after a mux change, not an error

        case sample_status::sensor_fault:
            swo::out << "pt1000 sensor fault (open/short/out of range)\r\n";
            break;

        case sample_status::crc_error:
        case sample_status::transfer_error:
            swo::out << "pt1000 sample error=" << static_cast<uint32_t>(r->status) << "\r\n";
            break;
        }
    }
}
// ============================================================================
// Main
// ============================================================================

int main()
{
    clock::init();
    clock::dwt_init();
    clock::systick_init<1000>();

    if (const auto e = flash::init(); e == w25q::error::ok)
    {
        test_memory_mapped_addresses();
    }
    else
    {
        swo::out << "flash init failed, err=" << static_cast<uint32_t>(e) << "\r\n";
    }

    my_spi::init();
    NVIC_SetPriority(EXTI2_IRQn, 3);
    NVIC_EnableIRQ(EXTI2_IRQn);
    if (!start_temperature_channel())
    {
        swo::out << "running without temperature channel\r\n";
    }

    fdcan_pins_init();
    const fdcan::Config can_cfg{
        .timing = {
            .nominal = {.prescaler = 4, .sjw = 4, .seg1 = 17, .seg2 = 6},
            .data    = {.prescaler = 1, .sjw = 6, .seg1 = 17, .seg2 = 6}},
        .mode = fdcan::Mode::Normal};

    if (const auto e = fdcan::Fdcan1::init(can_cfg); e != fdcan::Error::Ok)
    {
        swo::out << "fdcan init failed, err=" << static_cast<uint32_t>(e) << "\r\n";
    }
    
    static uint32_t g_busy_cycles = 0;
    static uint32_t g_window_start = clock::cycles();
    clock::systick_irq_disable();

    wdg::Iwdg::init<
            wdg::IwdgPrescaler::div256,
            4095,
            4095,
            100,
            wdg::Ewi::enable>();

    while (true)
    {
        const uint32_t t0 = clock::cycles();

        service_temperature();
        service_can();
        g_hcan.tick(clock::millis().count(), k_hcan_tick_period_ms);
        wdg::Iwdg::refresh();

        g_busy_cycles += clock::cycles_elapsed(t0);
        if (clock::cycles_elapsed(g_window_start) >= SystemCoreClock)
        {
            const uint64_t busy_percent_x100 =
                (static_cast<uint64_t>(g_busy_cycles) * 10000ULL) /
                static_cast<uint64_t>(SystemCoreClock);

            swo::out << "cpu busy in while(true)="
                     << (busy_percent_x100 / 100U)
                     << "."
                     << (busy_percent_x100 % 100U < 10 ? "0" : "")
                     << (busy_percent_x100 % 100U)
                     << "%\r\n";

            g_busy_cycles = 0;
            g_window_start = clock::cycles();
        }
        
        //__WFI();
    }
}

// ============================================================================
// Interrupts
// ============================================================================
extern "C" void EXTI2_IRQHandler()      { my_spi::drdy_exti_handler(); }
extern "C" void SPI1_IRQHandler()       { my_spi::irq_handler(); }
extern "C" void OCTOSPI1_IRQHandler()   { flash::irq_handler(); }
extern "C" void FDCAN1_IT0_IRQHandler() { fdcan::Fdcan1::irq_handler(); }