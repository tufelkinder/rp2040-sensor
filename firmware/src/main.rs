#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

extern crate alloc;

use alloc::string::{String, ToString};
use alloc_cortex_m::CortexMHeap;
use alloc::vec::Vec;
use alloc::format;
use core::alloc::Layout;
// use core::borrow::BorrowMut;
use core::fmt::Write;
// use core::ops::DerefMut;

use adxl343::{Adxl343, accelerometer::Accelerometer};  // , register::Register
use core::cell::RefCell;
// use core::fmt::Write;
// use core::time::Duration;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
// use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::{InputPin, OutputPin};
//use embedded_time::rate::Extensions;
//use embedded_time::{fixed_point::FixedPoint};
use fugit::RateExtU32;
use panic_probe as _;
// use heapless::Vec;
// use irq::{handler, scope, scoped_interrupts};

use rp2040_hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio,
    gpio::{bank0::{Gpio4, Gpio5, Gpio16, Gpio17}, Function, FunctionUart, Pin, Uart}, // , Interrupt, Pin, rp2040_hal::gpio::bank0::Gpio5;
    i2c::I2C,
    pac,
    pac::{interrupt, UART0, UART1},
    sio::Sio,
    uart::{self, Enabled, DataBits, StopBits, UartConfig, UartPeripheral},
    watchdog::Watchdog,
};

// use crate::pac::UART1;

#[link_section = ".boot_loader"]
#[used]
pub static BOOT_LOADER: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

type TUartC = UartPeripheral<Enabled, UART1, (Pin<Gpio4, Function<Uart>>, Pin<Gpio5, Function<Uart>>)>;
type TUartS = UartPeripheral<Enabled, UART0, (Pin<Gpio16, Function<Uart>>, Pin<Gpio17, Function<Uart>>)>;
type LedPin1 = gpio::Pin<gpio::bank0::Gpio9, gpio::PushPullOutput>;
type LedPin2 = gpio::Pin<gpio::bank0::Gpio10, gpio::PushPullOutput>;
type AdxlIntPin1 = gpio::Pin<gpio::bank0::Gpio19, gpio::PullDownInput>;
type AdxlIntPin2 = gpio::Pin<gpio::bank0::Gpio18, gpio::PullDownInput>;

static MSG_Q: Mutex<RefCell<Vec<String>>> = Mutex::new(RefCell::new(Vec::new()));
static LED1: Mutex<RefCell<Option<LedPin1>>> = Mutex::new(RefCell::new(None));
static LED2: Mutex<RefCell<Option<LedPin2>>> = Mutex::new(RefCell::new(None));
static ADXLI1: Mutex<RefCell<Option<AdxlIntPin1>>> = Mutex::new(RefCell::new(None));
static ADXLI2: Mutex<RefCell<Option<AdxlIntPin2>>> = Mutex::new(RefCell::new(None));
static UART_C: Mutex<RefCell<Option<TUartC>>> = Mutex::new(RefCell::new(None));
static UART_S: Mutex<RefCell<Option<TUartS>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 1024;
        static mut HEAP: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { ALLOCATOR.init(HEAP.as_ptr() as usize, HEAP_SIZE) }
    }

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

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = rp2040_hal::gpio::Pins::new(
        periphs.IO_BANK0,
        periphs.PADS_BANK0,
        sio.gpio_bank0,
        &mut periphs.RESETS,
    );

    // let mut led_pin = pins.gpio0.into_push_pull_output();
    // let mut led_pin2 = pins.gpio25.into_push_pull_output();
    // let mut led_pin = pins.gpio9.into_push_pull_output();
    // let mut led_pin2 = pins.gpio10.into_push_pull_output();

    let mut led1 = pins.gpio9.into_push_pull_output();
    let mut led2 = pins.gpio10.into_push_pull_output();

    // convert to globals...
    let adxl_int_pin1 = pins.gpio19.into_mode();
    let adxl_int_pin2 = pins.gpio18.into_mode();

    adxl_int_pin1.set_interrupt_enabled(gpio::Interrupt::LevelHigh, true);
    adxl_int_pin2.set_interrupt_enabled(gpio::Interrupt::LevelHigh, true);

    // main controller UART peripheral
    let c_uart_pins = (
        pins.gpio4.into_mode::<FunctionUart>(),
        pins.gpio5.into_mode::<FunctionUart>(),
    );
 
    // downstream sensor pod UART peripheral
    let s_uart_pins = (
        pins.gpio16.into_mode::<FunctionUart>(),
        pins.gpio17.into_mode::<FunctionUart>(),
    );

    let uart_clocks = clocks.peripheral_clock.freq();
    
    // main controller UART peripheral
    let mut uart_c = UartPeripheral::new(periphs.UART1, c_uart_pins, &mut periphs.RESETS)
//        .enable(UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One), uart_clocks)
        .enable(uart::common_configs::_115200_8_N_1, uart_clocks)
        .unwrap();

    // downstream sensor pod UART peripheral
    let mut uart_s = UartPeripheral::new(periphs.UART0, s_uart_pins, &mut periphs.RESETS)
//        .enable(UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One), uart_clocks)
        .enable(uart::common_configs::_115200_8_N_1, uart_clocks)
        .unwrap();

    uart_c.enable_rx_interrupt();
    uart_s.enable_rx_interrupt();

    unsafe {
        // cortex_m::interrupt::free(|cs| UART_C.borrow(cs).replace(Some(uart_c)));
        cortex_m::interrupt::free(|cs| UART_S.borrow(cs).replace(Some(uart_s)));
        // cortex_m::interrupt::free(|cs| LED1.borrow(cs).replace(Some(led1)));
        cortex_m::interrupt::free(|cs| LED2.borrow(cs).replace(Some(led2)));
        cortex_m::interrupt::free(|cs| ADXLI1.borrow(cs).replace(Some(adxl_int_pin1)));
        cortex_m::interrupt::free(|cs| ADXLI2.borrow(cs).replace(Some(adxl_int_pin2)));
    }

    let i2c = I2C::i2c0(
        periphs.I2C0,
        pins.gpio20.into_mode(), // sda
        pins.gpio21.into_mode(), // scl
        400.kHz(),
        &mut periphs.RESETS,
        125_000_000.Hz(),
    );

    let mut adx = Adxl343::new(i2c).unwrap();
    adx.write_register(adxl343::Register::INT_ENABLE, 1).unwrap();
    adx.write_register(adxl343::Register::THRESH_ACT, 1).unwrap();
    adx.write_register(adxl343::Register::ACT_INACT_CTL, 119).unwrap();

    // unsafe {
        // pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
        // pac::NVIC::unmask(pac::Interrupt::UART0_IRQ);
        // pac::NVIC::unmask(pac::Interrupt::UART1_IRQ);
    // }

    loop {
        led1.set_high().unwrap();
        delay.delay_ms(500);
        let acc_data = adx.accel_norm().unwrap();
        let cur_data = format!("{{id: 1, x: {:02}, y: {:02}, z: {:02}}}\r\n", acc_data.x, acc_data.y, acc_data.z);
        uart_c.write_str(cur_data.as_str()).unwrap();

        cortex_m::interrupt::free(|cs| {
            let mut messages = MSG_Q.borrow(cs).borrow_mut();

            while let Some(msg) = messages.pop() {
                uart_c.write_str(msg.as_str()).unwrap();
            }
        });

        led1.set_low().unwrap();
        delay.delay_ms(500);
    }

}

#[interrupt]
fn UART0_IRQ() {  // upstream sensor comms
    let mut buffer = [0u8; 64];

    let _bytes_read = cortex_m::interrupt::free(|cs| {
        let u_s = UART_S.borrow(cs).borrow();
        u_s.as_ref().unwrap().read_full_blocking(&mut buffer)
    });

    if _bytes_read.is_ok() {
        // let s: &str = core::str::from_utf8(&buffer).unwrap();
            if let Ok(s) = core::str::from_utf8(&buffer) {
                cortex_m::interrupt::free(|cs| {
                let mut msgs = MSG_Q.borrow(cs).borrow_mut();
                msgs.push(s.to_string());
            });
        }
    }
}

// #[interrupt]
// fn UART1_IRQ() {
//     let mut buffer = [0u8; 64];

//     let _bytes_read = cortex_m::interrupt::free(|cs| {
//         let u_s = UART_S.borrow(cs).borrow();
//         u_s.as_ref().unwrap().read_raw(&mut buffer)
//     });

//     if _bytes_read.is_ok() {
//         let s: &str = core::str::from_utf8(&buffer).unwrap();
//         cortex_m::interrupt::free(|cs| {
//             UART_C.borrow(cs).take().expect("Error taking UART1.").write_str(s).unwrap();
//         });
//     }
// }

// #[interrupt]
// fn IO_IRQ_BANK0 () {
//     cortex_m::interrupt::free(|cs| {
//         let s: &str = "{id:1, alert:\"Motion Detected.\"}\r\n";
//         UART_C.borrow(cs).take().expect("Error taking UART1.").write_str(s).unwrap();
//     });

//     let mut adi1 = cortex_m::interrupt::free(|cs| {
//         ADXLI1.borrow(cs).take().expect("Unable to take ADXL Sensor Pin 1.")
//     });
//     adi1.clear_interrupt(gpio::Interrupt::LevelHigh);
// }

#[alloc_error_handler]
fn oom(_: Layout) -> ! {
    loop {}
}

// End of file
