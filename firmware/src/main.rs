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
use core::ops::DerefMut;

use adxl343::{accelerometer::Accelerometer, Adxl343};  // , register::Register
use core::cell::RefCell;
// use core::fmt::Write;
// use core::time::Duration;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
// use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::rate::Extensions;
use embedded_time::{fixed_point::FixedPoint};
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
    uart::{self, Enabled, UartPeripheral},
    watchdog::Watchdog,
};

// use crate::pac::UART1;

#[link_section = ".boot_loader"]
#[used]
pub static BOOT_LOADER: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

// scoped_interrupts! {
//     enum Interrupt {
//         UART0_IRQ,
//         IO_IRQ_BANK0
//     }

//     use #[interrupt];
// }

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

type TUartC = UartPeripheral<Enabled, UART1, (Pin<Gpio4, Function<Uart>>, Pin<Gpio5, Function<Uart>>)>;
type TUartS = UartPeripheral<Enabled, UART0, (Pin<Gpio16, Function<Uart>>, Pin<Gpio17, Function<Uart>>)>;

static mut MSG_Q: Mutex<RefCell<Vec<String>>> = Mutex::new(RefCell::new(Vec::new()));

static mut UART_C: Mutex<RefCell<Option<TUartC>>> = Mutex::new(RefCell::new(None));
static mut UART_S: Mutex<RefCell<Option<TUartS>>> = Mutex::new(RefCell::new(None));

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
    // let mut led_pin2 = pins.gpio10.into_push_pull_output();

    let adxl_int_pin1 = pins.gpio19.into_pull_down_input();
    let adxl_int_pin2 = pins.gpio18.into_pull_down_input();

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
    
    let uart_clocks = clocks.peripheral_clock.into();
    
    // main controller UART peripheral
    let mut uart_c = UartPeripheral::new(periphs.UART1, c_uart_pins, &mut periphs.RESETS)
        .enable(uart::common_configs::_115200_8_N_1, uart_clocks)
        .unwrap();

    // downstream sensor pod UART peripheral
    let mut uart_s = UartPeripheral::new(periphs.UART0, s_uart_pins, &mut periphs.RESETS)
        .enable(uart::common_configs::_115200_8_N_1, uart_clocks)
        .unwrap();

    //    cortex_m::interrupt::free(|cs| UART_C.borrow(cs).replace(Some(uart_c)));
 
    unsafe {
        // cortex_m::interrupt::free(|cs| UART_C.borrow(cs).replace(Some(uart_c)));
        cortex_m::interrupt::free(|cs| UART_S.borrow(cs).replace(Some(uart_s)));
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

    #[interrupt]
    fn UART0_IRQ() {
        let mut buffer = [0u8; 64];

        unsafe {
            let _bytes_read = cortex_m::interrupt::free(|cs| {
                let u_s = UART_S.borrow(cs).borrow();
                u_s.as_ref().unwrap().read_raw(&mut buffer)
            });

            if _bytes_read.is_ok() {
                let s: &str = core::str::from_utf8(&buffer).unwrap();
                cortex_m::interrupt::free(|cs| {
                    MSG_Q.borrow(cs).borrow_mut().push(s.to_string());

                    // if let Some(ref mut u_c) =  UART_C.borrow(cs).borrow_mut().deref_mut() {
                    //     let s: &str = core::str::from_utf8(&buffer).unwrap();
                    //     u_c.write_str(format!("{}{}", s, "\n").as_str()).unwrap();
                    // }
                });
            }
        }
    }

    #[interrupt]
    fn IO_IRQ_BANK0 () {
        unsafe {
            cortex_m::interrupt::free(|cs| {
                let s: &str = "{id:1, alert:\"Motion Detected.\"}\n";
                MSG_Q.borrow(cs).borrow_mut().push(s.to_string());

                // if let Some(ref mut u_c) =  UART_C.borrow(cs).borrow_mut().deref_mut() {
                //     let s: &str = "{id:1, alert:\"Motion Detected.\"}\n";
                //     u_c.write_str(format!("{}{}", s, "\n").as_str()).unwrap();
                // }
            });
        }
    }

    loop {
        unsafe {
            led_pin.set_high().unwrap();
            delay.delay_ms(500);
            led_pin.set_low().unwrap();
            delay.delay_ms(500);
            let acc_data = adx.accel_norm().unwrap();

            let cur_data = format!("{{id: 1, x: {:02}, y: {:02}, z: {:02}}}\r\n", acc_data.x, acc_data.y, acc_data.z);
            uart_c.write_str(cur_data.as_str()).unwrap();

            cortex_m::interrupt::free(|cs| {
                let mut messages: Vec<String> = MSG_Q.borrow(cs).borrow_mut().to_vec();
                uart_c.write_str(messages.len().to_string().as_str()).unwrap();

                while let Some(msg) = messages.pop() {
                    let cur_data = format!("{{id: 1, x: {:02}, y: {:02}, z: {:02}}}\r\n", acc_data.x, acc_data.y, acc_data.z);
                    uart_c.write_str(msg.as_str()).unwrap();
                }

                // if let Some(ref mut u_c) =  UART_C.borrow(cs).borrow_mut().deref_mut() {
                //     let msg = format!("{{id: 1, x: {:02}, y: {:02}, z: {:02}}}\r\n", acc_data.x, acc_data.y, acc_data.z);
                //     u_c.write_str(&msg.as_str()).unwrap();
                // }
            });
        }
    }
    // })
}

#[alloc_error_handler]
fn oom(_: Layout) -> ! {
    loop {}
}

// End of file
