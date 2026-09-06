#include <Arduino.h>

#include "config.h"
#include "power_manager.h"

#if defined(BOARD_RP2040) || defined(BOARD_RP2350)
#include <hardware/clocks.h>
#include <hardware/regs/clocks.h>
#include <hardware/structs/clocks.h>
#endif

namespace PowerManager {

#if defined(BOARD_RP2040) || defined(BOARD_RP2350)
namespace {

#ifdef BOARD_RP2040
// XIAO RP2040 onboard LED GPIOs from the board schematic. They are not
// Arduino D-pin aliases and are separate from the application's external LEDs.
constexpr uint8_t ONBOARD_NEOPIXEL_POWER_PIN = 11u; // Active high
constexpr uint8_t ONBOARD_NEOPIXEL_DATA_PIN = 12u;
constexpr uint8_t ONBOARD_LED_GREEN_PIN = 16u; // Status RGB LED is active low
constexpr uint8_t ONBOARD_LED_RED_PIN = 17u;
constexpr uint8_t ONBOARD_LED_BLUE_PIN = 25u;
#else
// XIAO RP2350 onboard loads. The addressable RGB LED has no power-enable
// GPIO, so hold its data input low. The yellow user LED is active low.
constexpr uint8_t ONBOARD_RGB_DATA_PIN = 22u;
constexpr uint8_t ONBOARD_USER_LED_PIN = 25u;
constexpr uint8_t BATTERY_SENSE_ENABLE_PIN = 19u; // Active high
#endif

bool g_sleep_clocks_active = false;
uint32_t g_awake_sys_clock_hz = 0;
uint32_t g_awake_peri_clock_hz = 0;
uint32_t g_awake_sleep_en0 = 0;
uint32_t g_awake_sleep_en1 = 0;

#ifdef BOARD_RP2040
// The sleep clock masks take effect only while both cores execute WFI/WFE.
// Keep USB device operation, the timer used by delay(), I2C1 used by Wire,
// GPIO, and every memory bank that can hold application state.
constexpr uint32_t SLEEP_EN0 =
  CLOCKS_SLEEP_EN0_CLK_SYS_SRAM3_BITS |
  CLOCKS_SLEEP_EN0_CLK_SYS_SRAM2_BITS |
  CLOCKS_SLEEP_EN0_CLK_SYS_SRAM1_BITS |
  CLOCKS_SLEEP_EN0_CLK_SYS_SRAM0_BITS |
  CLOCKS_SLEEP_EN0_CLK_SYS_SIO_BITS |
  CLOCKS_SLEEP_EN0_CLK_SYS_PLL_USB_BITS |
  CLOCKS_SLEEP_EN0_CLK_SYS_PLL_SYS_BITS |
  CLOCKS_SLEEP_EN0_CLK_SYS_PADS_BITS |
  CLOCKS_SLEEP_EN0_CLK_SYS_VREG_AND_CHIP_RESET_BITS |
  CLOCKS_SLEEP_EN0_CLK_SYS_IO_BITS |
  CLOCKS_SLEEP_EN0_CLK_SYS_I2C1_BITS |
  CLOCKS_SLEEP_EN0_CLK_SYS_BUSFABRIC_BITS |
  CLOCKS_SLEEP_EN0_CLK_SYS_BUSCTRL_BITS |
  CLOCKS_SLEEP_EN0_CLK_SYS_CLOCKS_BITS;

constexpr uint32_t SLEEP_EN1 =
  CLOCKS_SLEEP_EN1_CLK_SYS_XOSC_BITS |
  CLOCKS_SLEEP_EN1_CLK_SYS_XIP_BITS |
  CLOCKS_SLEEP_EN1_CLK_SYS_TIMER_BITS |
  CLOCKS_SLEEP_EN1_CLK_SYS_SYSCFG_BITS |
  CLOCKS_SLEEP_EN1_CLK_SYS_SRAM5_BITS |
  CLOCKS_SLEEP_EN1_CLK_SYS_SRAM4_BITS |
  CLOCKS_SLEEP_EN1_CLK_USB_USBCTRL_BITS |
  CLOCKS_SLEEP_EN1_CLK_SYS_USBCTRL_BITS;

static_assert(RP2040_SLEEP_SYS_CLOCK_KHZ > 48000, "USB requires clk_sys above 48 MHz");
#else
// RP2350 has 63 clock destinations split over two sleep-enable words. Keep
// both USB clocks, both timer blocks and their tick sources, I2C1 used by
// Wire1, GPIO, clock/power control, XIP, and every SRAM bank. Compute masks
// from SDK destination numbers to avoid RP2040/RP2350 register-name overlap.
constexpr uint32_t sleepEn0Bit(clock_dest_num_t destination)
{
    return 1u << static_cast<uint32_t>(destination);
}

constexpr uint32_t sleepEn1Bit(clock_dest_num_t destination)
{
    return 1u << (static_cast<uint32_t>(destination) - 32u);
}

constexpr uint32_t SLEEP_EN0 =
  sleepEn0Bit(CLK_DEST_SYS_CLOCKS) |
  sleepEn0Bit(CLK_DEST_SYS_ACCESSCTRL) |
  sleepEn0Bit(CLK_DEST_SYS_BUSCTRL) |
  sleepEn0Bit(CLK_DEST_SYS_BUSFABRIC) |
  sleepEn0Bit(CLK_DEST_SYS_I2C1) |
  sleepEn0Bit(CLK_DEST_SYS_IO) |
  sleepEn0Bit(CLK_DEST_SYS_PADS) |
  sleepEn0Bit(CLK_DEST_SYS_PLL_USB) |
  sleepEn0Bit(CLK_DEST_REF_POWMAN) |
  sleepEn0Bit(CLK_DEST_SYS_POWMAN) |
  sleepEn0Bit(CLK_DEST_SYS_RESETS) |
  sleepEn0Bit(CLK_DEST_SYS_SIO);

constexpr uint32_t SLEEP_EN1 =
  sleepEn1Bit(CLK_DEST_SYS_SRAM0) |
  sleepEn1Bit(CLK_DEST_SYS_SRAM1) |
  sleepEn1Bit(CLK_DEST_SYS_SRAM2) |
  sleepEn1Bit(CLK_DEST_SYS_SRAM3) |
  sleepEn1Bit(CLK_DEST_SYS_SRAM4) |
  sleepEn1Bit(CLK_DEST_SYS_SRAM5) |
  sleepEn1Bit(CLK_DEST_SYS_SRAM6) |
  sleepEn1Bit(CLK_DEST_SYS_SRAM7) |
  sleepEn1Bit(CLK_DEST_SYS_SRAM8) |
  sleepEn1Bit(CLK_DEST_SYS_SRAM9) |
  sleepEn1Bit(CLK_DEST_SYS_SYSCFG) |
  sleepEn1Bit(CLK_DEST_REF_TICKS) |
  sleepEn1Bit(CLK_DEST_SYS_TICKS) |
  sleepEn1Bit(CLK_DEST_SYS_TIMER0) |
  sleepEn1Bit(CLK_DEST_SYS_TIMER1) |
  sleepEn1Bit(CLK_DEST_SYS_USBCTRL) |
  sleepEn1Bit(CLK_DEST_USB) |
  sleepEn1Bit(CLK_DEST_SYS_WATCHDOG) |
  sleepEn1Bit(CLK_DEST_SYS_XIP) |
  sleepEn1Bit(CLK_DEST_SYS_XOSC);
#endif

void setOutputLevel(uint8_t pin, uint8_t level)
{
    pinMode(pin, OUTPUT);
    digitalWrite(pin, level);
}

} // namespace
#endif

void begin()
{
#ifdef BOARD_RP2040
    setOutputLevel(ONBOARD_NEOPIXEL_POWER_PIN, LOW);
    setOutputLevel(ONBOARD_NEOPIXEL_DATA_PIN, LOW);
    setOutputLevel(ONBOARD_LED_GREEN_PIN, HIGH);
    setOutputLevel(ONBOARD_LED_RED_PIN, HIGH);
    setOutputLevel(ONBOARD_LED_BLUE_PIN, HIGH);
#elif defined(BOARD_RP2350)
    setOutputLevel(ONBOARD_RGB_DATA_PIN, LOW);
    setOutputLevel(ONBOARD_USER_LED_PIN, HIGH);
    setOutputLevel(BATTERY_SENSE_ENABLE_PIN, LOW);
#endif
}

void enterSleep()
{
#if defined(BOARD_RP2040) || defined(BOARD_RP2350)
    if (g_sleep_clocks_active) {
        return;
    }

    g_awake_sys_clock_hz = clock_get_hz(clk_sys);
    g_awake_peri_clock_hz = clock_get_hz(clk_peri);
#ifdef BOARD_RP2040
    g_awake_sleep_en0 = clocks_hw->sleep_en0;
    g_awake_sleep_en1 = clocks_hw->sleep_en1;
#else
    g_awake_sleep_en0 = clocks_hw->sleep_en[0];
    g_awake_sleep_en1 = clocks_hw->sleep_en[1];
#endif

#ifdef BOARD_RP2040
    // Keep clk_sys above the RP2040's 48 MHz USB requirement. If the requested
    // frequency cannot be generated, leave all sleep settings untouched.
    if (!set_sys_clock_khz(RP2040_SLEEP_SYS_CLOCK_KHZ, false)) {
        return;
    }
#else
    // RP2350 can run clk_sys directly from the 48 MHz USB PLL. The SDK helper
    // also shuts down PLL_SYS, saving its quiescent current while USB remains
    // enumerated and available as a wake source.
    set_sys_clock_48mhz();
#endif

#ifdef BOARD_RP2040
    clocks_hw->sleep_en0 = SLEEP_EN0;
    clocks_hw->sleep_en1 = SLEEP_EN1;
#else
    clocks_hw->sleep_en[0] = SLEEP_EN0;
    clocks_hw->sleep_en[1] = SLEEP_EN1;
#endif
    g_sleep_clocks_active = true;
#endif
}

void exitSleep()
{
#if defined(BOARD_RP2040) || defined(BOARD_RP2350)
    if (!g_sleep_clocks_active) {
        return;
    }

#ifdef BOARD_RP2040
    clocks_hw->sleep_en0 = g_awake_sleep_en0;
    clocks_hw->sleep_en1 = g_awake_sleep_en1;
#else
    clocks_hw->sleep_en[0] = g_awake_sleep_en0;
    clocks_hw->sleep_en[1] = g_awake_sleep_en1;
#endif
    set_sys_clock_khz((g_awake_sys_clock_hz + 999u) / 1000u, true);

    // set_sys_clock_khz() deliberately moves clk_peri to a safe 48 MHz source.
    // Put it back on clk_sys so I2C and the other peripherals regain their
    // original active-state rate after wake.
    clock_configure(
      clk_peri,
      0,
      CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
      clock_get_hz(clk_sys),
      g_awake_peri_clock_hz);
    g_sleep_clocks_active = false;
#endif
}

bool sleepActive()
{
    return g_sleep_clocks_active;
}

} // namespace PowerManager
