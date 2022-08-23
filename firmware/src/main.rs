#![no_std]
#![no_main]

use adxl343::{accelerometer::Accelerometer, Adxl343};
use core::fmt::Write;
use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::fixed_point::FixedPoint;
use embedded_time::rate::Extensions;
use panic_probe as _;

use rp2040_hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio::FunctionUart,
    i2c::I2C,
    pac,
    sio::Sio,
    uart::{self, UartPeripheral},
    watchdog::Watchdog,
};

#[link_section = ".boot_loader"]
#[used]
pub static BOOT_LOADER: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

//

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut periphs = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(periphs.WATCHDOG);
    let sio = Sio::new(periphs.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        periphs.XOSC,
        periphs.CLOCKS,
        periphs.PLL_SYS,
        periphs.PLL_USB,
        &mut periphs.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let pins = rp2040_hal::gpio::Pins::new(
        periphs.IO_BANK0,
        periphs.PADS_BANK0,
        sio.gpio_bank0,
        &mut periphs.RESETS,
    );

    // let mut led_pin = pins.gpio0.into_push_pull_output();
    // let mut led_pin2 = pins.gpio25.into_push_pull_output();
    let mut led_pin = pins.gpio9.into_push_pull_output();
    let mut led_pin2 = pins.gpio10.into_push_pull_output();

    // main controller side UART pins
    let c_uart_pins = (
        pins.gpio4.into_mode::<FunctionUart>(),
        pins.gpio5.into_mode::<FunctionUart>(),
    );

    // upstream controller side UART pins
    let s_uart_pins = (
        pins.gpio16.into_mode::<FunctionUart>(),
        pins.gpio17.into_mode::<FunctionUart>(),
    );

    let uart_clocks = clocks.peripheral_clock.into();

    // main controller UART peripheral
    let mut uart_c = UartPeripheral::new(periphs.UART1, c_uart_pins, &mut periphs.RESETS)
        .enable(uart::common_configs::_115200_8_N_1, uart_clocks)
        .unwrap();

    // downstream sensor pod UART peripheral
    let uart_s = UartPeripheral::new(periphs.UART0, s_uart_pins, &mut periphs.RESETS)
        .enable(uart::common_configs::_115200_8_N_1, uart_clocks)
        .unwrap();

    let i2c = I2C::i2c0(
        periphs.I2C0,
        pins.gpio20.into_mode(), // sda
        pins.gpio21.into_mode(), // scl
        400.kHz(),
        &mut periphs.RESETS,
        125_000_000.Hz(),
    );

    let mut adx = Adxl343::new(i2c).unwrap();

    loop {
        info!("on!");
        led_pin.set_high().unwrap();
        led_pin2.set_low().unwrap();
        delay.delay_ms(500);
        info!("off!");
        led_pin.set_low().unwrap();
        led_pin2.set_high().unwrap();
        delay.delay_ms(500);
        let acc_data = adx.accel_norm().unwrap();
        writeln!(uart_c, "Hello World!\r").unwrap();
        writeln!(
            uart_c,
            "{{id: 1, x: {:02}, y: {:02}, z: {:02}}}\r",
            acc_data.x, acc_data.y, acc_data.z
        )
        .unwrap();
        let mut buffer = [0u8; 3];
        let mut _bytes_read = uart_s.read_raw(&mut buffer);
        writeln!(uart_c, "{:?}", buffer).unwrap();
    }
}

// End of file
