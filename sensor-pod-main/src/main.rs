#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

mod adxl343;
mod config;
mod icd;
mod json;
use core::str::FromStr;
use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering};

use adxl343::register::Register;
use adxl343::{AccelData, Adxl343};
use config::Config;
use defmt::*;
use embassy_boot_rp::*;
use embassy_executor::Spawner;
use embassy_executor::_export::StaticCell;
use embassy_rp::flash::Flash;
use embassy_rp::gpio::{AnyPin, Input, Level, Output, Pin, Pull};
use embassy_rp::i2c::{self, Async, I2c};
use embassy_rp::interrupt;
use embassy_rp::peripherals::{FLASH, I2C0, UART0, UART1};
use embassy_rp::uart::{self, BufferedUart, BufferedUartRx, BufferedUartTx};
use embassy_rp::watchdog::*;
use embassy_rp::adc::{self, Adc};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::{Channel, Receiver};
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Ticker, Timer};
use futures::StreamExt;
use heapless::String;
use icd::*;
use json::*;
use {defmt_rtt as _, panic_probe as _};

const SELF_ID: u32 = 0;
const FLASH_SIZE: usize = 2 * 1024 * 1024;
const VERSION: &str = env!("CARGO_PKG_VERSION");
const AVG_NUM: u8 = 10;

static APP_B: &[u8] = include_bytes!("../next.bin");
static REBOOT_AFTER_UTX: AtomicBool = AtomicBool::new(false);
static HEARTBEAT_TX_INTERVAL: AtomicU8 = AtomicU8::new(8);
static UPTIME: AtomicU32 = AtomicU32::new(0);
static DFU_UPDATED: Signal<ThreadModeRawMutex, ()> = Signal::new();
static REQ_CH: Channel<ThreadModeRawMutex, Request, 8> = Channel::new();
static RES_CH: Channel<ThreadModeRawMutex, Response, 64> = Channel::new();
static SENSOR_CH: Channel<ThreadModeRawMutex, SensorCmd, 64> = Channel::new();
static FLASH_CH: Channel<ThreadModeRawMutex, FlashCmd, 8> = Channel::new();
static REQ_LED_CH: Channel<ThreadModeRawMutex, (), 8> = Channel::new();
static RES_LED_CH: Channel<ThreadModeRawMutex, (), 8> = Channel::new();

macro_rules! singleton {
    ($val:expr) => {{
        type T = impl Sized;
        static STATIC_CELL: StaticCell<T> = StaticCell::new();
        let (x,) = STATIC_CELL.init(($val,));
        x
    }};
}

enum SensorCmd {
    Poll,
    SelfTest,
    Event(SensorInt),
    SetConfigReg(Register, u8),
}

enum FlashCmd<'a> {
    SetHeartbeat(u8),
    ResetConfig(bool),
    SetConfigReg(Register, u8),
    DfuWrite(usize, &'a [u8], bool),
    DfuMarkBooted,
    Status,
}

#[derive(Clone, Copy)]
enum SensorInt {
    Int1,
    //Int2,
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    REBOOT_AFTER_UTX.store(false, Ordering::Release);
    UPTIME.store(0, Ordering::Release);

    Timer::after(Duration::from_millis(100)).await; // Wait for RTT
    let mut flash = Flash::<_, FLASH_SIZE>::new(p.FLASH);

    let config = unwrap!(Config::load(&mut flash));
    info!("{:?}", config);
    HEARTBEAT_TX_INTERVAL.store(config.heartbeat_interval, Ordering::Release);

    let (sda, scl) = (p.PIN_20, p.PIN_21);
    let i2c_irq = interrupt::take!(I2C0_IRQ);
    let i2c = I2c::new_async(p.I2C0, scl, sda, i2c_irq, i2c::Config::default());
    let sensor = unwrap!(Adxl343::new(i2c, config.into()).await);

    let uart_config = uart::Config::default();
    let uart1_irq = interrupt::take!(UART1_IRQ);
    let (dtx_pin, drx_pin, uart1) = (p.PIN_4, p.PIN_5, p.UART1);
    let (drx_buf, dtx_buf) = (&mut singleton!([0u8; 128])[..], &mut singleton!([0u8; 128])[..]);
    let mut uart1 = BufferedUart::new(uart1, uart1_irq, dtx_pin, drx_pin, dtx_buf, drx_buf, uart_config);
    let (drx, dtx) = uart1.split();

    let uart0_irq = interrupt::take!(UART0_IRQ);
    let (utx_pin, urx_pin, uart0) = (p.PIN_16, p.PIN_17, p.UART0);
    let (urx_buf, utx_buf) = (&mut singleton!([0u8; 128])[..], &mut singleton!([0u8; 128])[..]);
    let mut uart0 = BufferedUart::new(uart0, uart0_irq, utx_pin, urx_pin, utx_buf, urx_buf, uart_config);
    let (urx, utx) = uart0.split();

    let req_led = Output::new(p.PIN_10.degrade(), Level::Low);
    let res_led = Output::new(p.PIN_9.degrade(), Level::Low);
    let int1_pin = Input::new(p.PIN_19.degrade(), Pull::None);
    // let int2_pin = Input::new(p.PIN_18.degrade(), Pull::None);

    let adc_irq = interrupt::take!(ADC_IRQ_FIFO);
    let adc = Adc::new(p.ADC, adc_irq, adc::Config::default());

    unwrap!(spawner.spawn(led_task(REQ_LED_CH.receiver(), req_led)));
    unwrap!(spawner.spawn(led_task(RES_LED_CH.receiver(), res_led)));
    unwrap!(spawner.spawn(int_task(SensorInt::Int1, int1_pin)));
    // unwrap!(spawner.spawn(int_task(SensorInt::Int2, int2_pin)));
    unwrap!(spawner.spawn(sensor_task(sensor)));
    unwrap!(spawner.spawn(flash_task(flash, adc)));
    unwrap!(spawner.spawn(upstream_tx_task(utx)));
    unwrap!(spawner.spawn(upstream_rx_task(urx)));
    unwrap!(spawner.spawn(downstream_tx_task(dtx)));
    unwrap!(spawner.spawn(downstream_rx_task(drx)));

    let mut watchdog = Watchdog::new(p.WATCHDOG);
    watchdog.start(Duration::from_secs(8));
    let mut heartbeat = Ticker::every(Duration::from_secs(1));

    loop {
        let uptime = UPTIME.load(Ordering::Acquire);
        if uptime % HEARTBEAT_TX_INTERVAL.load(Ordering::Acquire) as u32 == 0 {
            SENSOR_CH.send(SensorCmd::Poll).await;
        }
        UPTIME.store(uptime + 1, Ordering::Release);

        watchdog.feed();
        heartbeat.next().await;
    }
}

#[embassy_executor::task(pool_size = 2)]
async fn int_task(id: SensorInt, mut pin: Input<'static, AnyPin>) {
    loop {
        pin.wait_for_high().await;
        SENSOR_CH.send(SensorCmd::Event(id)).await;
        let _ = RES_LED_CH.try_send(());
        pin.wait_for_low().await;
    }
}

#[embassy_executor::task(pool_size = 2)]
async fn led_task(receiver: Receiver<'static, ThreadModeRawMutex, (), 8>, mut pin: Output<'static, AnyPin>) {
    loop {
        receiver.recv().await;
        pin.set_high();
        Timer::after(Duration::from_millis(50)).await;
        pin.set_low();
        Timer::after(Duration::from_millis(50)).await;
    }
}

#[embassy_executor::task]
async fn sensor_task(mut sensor: Adxl343<I2c<'static, I2C0, Async>>) {
    loop {
        match SENSOR_CH.recv().await {
            SensorCmd::Event(_int) => {
                if let Ok(src) = sensor.interrupt_source().await {
                    if src.is_single_tap() {
                        info!("Tap");
                        let data = unwrap!(sensor.accel_avg_i16(AVG_NUM).await);
                        RES_CH.send(event_response(Event::Tap, data)).await;
                    } else if src.is_activity() {
                        info!("Activity");
                        let data = unwrap!(sensor.accel_avg_i16(AVG_NUM).await);
                        RES_CH.send(event_response(Event::Activity, data)).await;
                    } else if src.is_inactivity() {
                        info!("Inactivity");
                        let data = unwrap!(sensor.accel_avg_i16(AVG_NUM).await);
                        RES_CH.send(event_response(Event::Inactivity, data)).await;
                    }
                }
            }
            SensorCmd::Poll => {
                info!("Read sensor");
                let data = unwrap!(sensor.accel_avg_i16(AVG_NUM).await);
                RES_CH.send(data_response(data)).await;
            }
            SensorCmd::SelfTest => {
                info!("Self test");
                unwrap!(sensor.self_test(true).await);
                Timer::after(Duration::from_millis(100)).await;
                unwrap!(sensor.self_test(false).await);
            }
            SensorCmd::SetConfigReg(reg, val) => {
                unwrap!(sensor.write_register(reg, val).await);
                FLASH_CH.send(FlashCmd::SetConfigReg(reg, val)).await;
            }
        }
    }
}

#[embassy_executor::task]
async fn flash_task(mut flash: Flash<'static, FLASH, FLASH_SIZE>, mut adc: Adc<'static>) {
    let mut updater = FirmwareUpdater::default();
    let mut writer = None;
    loop {
        match FLASH_CH.recv().await {
            FlashCmd::SetHeartbeat(val) => {
                info!("SET HEARTBEAT TX INTERVAL");
                let mut cfg = unwrap!(Config::load(&mut flash));
                cfg.heartbeat_interval = val;
                unwrap!(cfg.save(&mut flash));
            }
            FlashCmd::ResetConfig(defer_reboot) => {
                info!("RESET CONFIG");
                let cfg: Config = Default::default();
                unwrap!(cfg.save(&mut flash));
                reboot(defer_reboot);
            }
            FlashCmd::SetConfigReg(reg, val) => {
                info!("SET CONFIG REG");
                unwrap!(Config::update_reg(&mut flash, reg, val));
            }
            FlashCmd::DfuWrite(offset, data, is_done) => {
                info!("DFU");
                let w = writer.get_or_insert_with(|| unwrap!(updater.prepare_update_blocking(&mut flash)));
                let mut buf: AlignedBuffer<4096> = AlignedBuffer([0; 4096]);
                buf.0[..data.len()].copy_from_slice(data);
                w.write_block_blocking(offset, &buf.0[..], &mut flash, 256).unwrap();
                if is_done {
                    updater.mark_updated_blocking(&mut flash, &mut buf.0[..1]).unwrap();
                    DFU_UPDATED.signal(());
                }
            }
            FlashCmd::DfuMarkBooted => {
                info!("DFU MARK AS BOOTED");
                let mut buf: AlignedBuffer<1> = AlignedBuffer([0; 1]);
                updater.mark_booted_blocking(&mut flash, &mut buf.0).unwrap();
            }
            FlashCmd::Status => {
                info!("GET STATUS");
                let config = unwrap!(Config::load(&mut flash));
                let temp = adc.read_temperature().await;
                RES_CH.send(status_response(config, temp)).await;
            }
        }
    }
}

#[embassy_executor::task]
async fn downstream_rx_task(rx: BufferedUartRx<'static, UART1>) {
    read_json_stream::<_, _, 4160>(rx, handle_downstream_request).await;
}

#[embassy_executor::task]
async fn upstream_tx_task(mut tx: BufferedUartTx<'static, UART0>) {
    loop {
        let mut req = REQ_CH.recv().await;
        if let Some(id) = req.id {
            req.id.replace(id - 1);
        }
        unwrap!(write_json::<_, 128>(&mut tx, &req).await);
        let _ = REQ_LED_CH.try_send(());
        if REBOOT_AFTER_UTX.load(Ordering::Acquire) {
            cortex_m::peripheral::SCB::sys_reset();
        }
    }
}

#[embassy_executor::task]
async fn upstream_rx_task(rx: BufferedUartRx<'static, UART0>) {
    read_json_stream::<_, _, 256>(rx, handle_upstream_response).await;
}

#[embassy_executor::task]
async fn downstream_tx_task(mut tx: BufferedUartTx<'static, UART1>) {
    loop {
        let res = RES_CH.recv().await;
        unwrap!(write_json::<_, 256>(&mut tx, &res).await);
        let _ = RES_LED_CH.try_send(());
    }
}

#[inline(always)]
async fn handle_downstream_request(req: Request) {
    let _ = REQ_LED_CH.try_send(());
    if req.id.is_none() || req.id == Some(SELF_ID) {
        info!("EXEC");
        let _ = REQ_LED_CH.try_send(());
        match req.action {
            RequestAction::DfuWrite(ref _chunk) => {
                // TODO: Use real chunk instead of mock
                run_mock_dfu(req.id.is_none()).await;
            }
            RequestAction::DfuMarkBooted => FLASH_CH.send(FlashCmd::DfuMarkBooted).await,
            RequestAction::Poll => SENSOR_CH.send(SensorCmd::Poll).await,
            RequestAction::Status => FLASH_CH.send(FlashCmd::Status).await,
            RequestAction::ResetConfig => FLASH_CH.send(FlashCmd::ResetConfig(req.id.is_none())).await,
            RequestAction::SetHeartbeat(secs) => {
                let secs = secs.max(1); // 1 second minimum
                HEARTBEAT_TX_INTERVAL.store(secs, Ordering::Release);
                FLASH_CH.send(FlashCmd::SetHeartbeat(secs)).await;
            }
            RequestAction::SetActivityThreshold(val) => {
                SENSOR_CH.send(SensorCmd::SetConfigReg(Register::THRESH_ACT, val)).await;
            }
            RequestAction::SetInactivityTime(val) => {
                SENSOR_CH.send(SensorCmd::SetConfigReg(Register::TIME_INACT, val)).await;
            }
            RequestAction::SetTapThreshold(val) => {
                SENSOR_CH.send(SensorCmd::SetConfigReg(Register::THRESH_TAP, val)).await;
            }
            RequestAction::SetTapDuration(val) => {
                SENSOR_CH.send(SensorCmd::SetConfigReg(Register::DUR, val)).await;
            }
            RequestAction::SelfTest => {
                SENSOR_CH.send(SensorCmd::SelfTest).await;
            }
            RequestAction::Reboot => {
                reboot(req.id.is_none());
            }
        }
    }
    if req.id.is_none() || req.id != Some(SELF_ID) {
        info!("Relay to UTX");
        REQ_CH.send(req).await;
    }
}

#[inline(always)]
async fn handle_upstream_response(mut res: Response) {
    res.id += 1;
    info!("Relay to DTX");
    RES_CH.send(res).await;
    let _ = RES_LED_CH.try_send(());
}

fn data_response(data: AccelData) -> Response {
    Response {
        id: 0,
        payload: ResponsePayload::Data(data),
    }
}

fn event_response(event: Event, data: AccelData) -> Response {
    Response {
        id: 0,
        payload: ResponsePayload::Event(EventData { event, data }),
    }
}

fn status_response(config: Config, temp: u16) -> Response {
    // According to chapter 4.9.5. Temperature Sensor in RP2040 datasheet
    let temperature = (27.0 - (temp as f32 * 3.3 / 4096.0 - 0.706) / 0.001721) as i8;
    Response {
        id: 0,
        payload: ResponsePayload::Status(Status {
            version: unwrap!(String::from_str(VERSION)),
            uptime: UPTIME.load(Ordering::Acquire),
            config: config.into(),
            temperature,
        }),
    }
}

fn reboot(defer: bool) {
    if defer {
        REBOOT_AFTER_UTX.store(true, Ordering::Release);
    } else {
        cortex_m::peripheral::SCB::sys_reset();
    }
}

async fn run_mock_dfu(defer_reboot: bool) {
    let mut offset = 0;
    let chunks = APP_B.chunks(4096);
    let len = chunks.len();
    for (i, chunk) in chunks.enumerate() {
        let is_done = i == len - 1;
        FLASH_CH.send(FlashCmd::DfuWrite(offset, chunk, is_done)).await;
        if is_done {
            DFU_UPDATED.wait().await;
            reboot(defer_reboot);
        } else {
            offset += chunk.len();
            let res = Response {
                id: 0,
                payload: ResponsePayload::DfuOffset(offset),
            };
            RES_CH.send(res).await;
        }
    }
}
