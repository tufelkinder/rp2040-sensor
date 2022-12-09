#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(alloc_error_handler)]

extern crate alloc;

use alloc::string::{String, ToString};
use alloc_cortex_m::CortexMHeap;
use alloc::vec::Vec;
use alloc::format;
use core::alloc::Layout;
use core::cell::RefCell;
use cortex_m::interrupt::Mutex;

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::i2c::{self, Config};
use embassy_rp::{interrupt, uart};
use embassy_rp::uart::Blocking;
use embassy_rp::peripherals::{UART0, PIN_16, PIN_17};
use embassy_rp::Peripherals;
use embassy_time::{Duration, Timer};
//use embedded_hal_async::i2c::I2c;
use embassy_rp::gpio;
use gpio::{Input, Level, Output, Pull};
use adxl343::{accelerometer::Accelerometer, Adxl343};
use {defmt_rtt as _, panic_probe as _};

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

type TUart0 = uart::Uart<'static, UART0, Blocking>;
// static mut UART_S: Mutex<RefCell<Option<TUart0>>> = Mutex::new(RefCell::new(None));

static mut MSG_Q: Mutex<RefCell<Vec<String>>> = Mutex::new(RefCell::new(Vec::new()));


#[embassy_executor::task]
async fn uart0_listen(config: uart::Config, u0: UART0, p_tx: PIN_16, p_rx: PIN_17) {
    let mut buffer = [0u8; 64];

    let mut s_uart = uart::Uart::new_blocking(u0, p_tx, p_rx, config);

    loop {
        unsafe {
            let _bytes_read = s_uart.blocking_read(&mut buffer);

            if _bytes_read.is_ok() {
                let s: &str = core::str::from_utf8(&buffer).unwrap();
                cortex_m::interrupt::free(|cs| {
                    MSG_Q.borrow(cs).borrow_mut().push(s.to_string());
                });
            }
        }
        Timer::after(Duration::from_millis(150)).await;
    }
}


#[embassy_executor::main]
async fn main(spawner: Spawner) {

    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 1024;
        static mut HEAP: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { ALLOCATOR.init(HEAP.as_ptr() as usize, HEAP_SIZE) }
    }

    let p = embassy_rp::init(Default::default());

    let mut led = Output::new(p.PIN_9, Level::Low);
    let mut led2 = Output::new(p.PIN_10, Level::Low);

    let config = uart::Config::default();
    let mut c_uart = uart::Uart::new_blocking(p.UART1, p.PIN_4, p.PIN_5, config);
    c_uart.blocking_write("{ 'id': 1, 'status': \"Initializing...\"}\r\n".as_bytes()).unwrap();

    // unwrap!(spawner.spawn(uart0_listen(config, p.UART0, p.PIN_16, p.PIN_17)));

    let sda = p.PIN_20;
    let scl = p.PIN_21;
    let irq = interrupt::take!(I2C0_IRQ);

    let adxl_int_pin1 = Input::new(p.PIN_19, Pull::None);
    let adxl_int_pin2 = Input::new(p.PIN_18, Pull::None);

    let i2c = i2c::I2c::new_async(p.I2C0, scl, sda, irq, Config::default());

    let mut adx = Adxl343::new(i2c).unwrap();
    adx.write_register(adxl343::Register::INT_ENABLE, 1).unwrap();
    adx.write_register(adxl343::Register::THRESH_ACT, 1).unwrap();
    adx.write_register(adxl343::Register::ACT_INACT_CTL, 119).unwrap();

    loop {
        let acc_data = adx.accel_norm().unwrap();
        let adxl_i1 = adxl_int_pin1.get_level();
        let adxl_i2 = adxl_int_pin2.get_level();
        let cur_data = format!("{{id: 1, x: {:02}, y: {:02}, z: {:02}, int1: {:?}, int2: {:?}}}\r\n", acc_data.x, acc_data.y, acc_data.z, adxl_i1, adxl_i2);

        led.set_high();
        c_uart.blocking_write(cur_data.as_bytes()).unwrap();
        cortex_m::asm::delay(1_000_000);
        led.set_low();
        Timer::after(Duration::from_millis(500)).await;
    }
}


#[alloc_error_handler]
fn oom(_: Layout) -> ! {
    loop {}
}