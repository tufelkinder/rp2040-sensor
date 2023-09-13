#![allow(non_snake_case)]
use defmt::Format;
use embassy_boot_rp::AlignedBuffer;
use embassy_rp::flash::{self, Flash, ERASE_SIZE};
use embassy_rp::peripherals::FLASH;
use heapless::String;
use postcard::{from_bytes, to_slice};
use serde::{Deserialize, Serialize};

use crate::adxl343;
use crate::adxl343::register::Register;
const FLASH_SIZE: usize = 2 * 1024 * 1024;

extern "C" {
    // Flash storage used for configuration
    static __config_start: u32;
    static __config_end: u32;
}

#[derive(Format)]
pub enum Error {
    Load,
    Save,
}

#[derive(Format, Serialize, Deserialize, Clone)]
pub struct Config {
    pub version: String<4>,
    pub heartbeat_interval: u8,
    pub THRESH_ACT: u8,
    pub TIME_INACT: u8,
    pub THRESH_TAP: u8,
    pub DUR: u8,
}

impl From<Config> for adxl343::Config {
    fn from(cfg: Config) -> adxl343::Config {
        adxl343::Config {
            THRESH_ACT: cfg.THRESH_ACT,
            TIME_INACT: cfg.TIME_INACT,
            THRESH_TAP: cfg.THRESH_TAP,
            DUR: cfg.DUR,
        }
    }
}

impl Config {
    pub fn load(flash: &mut Flash<'_, FLASH, flash::Blocking, FLASH_SIZE>) -> Result<Self, Error> {
        let offset = unsafe { &__config_start as *const u32 as u32 };
        let mut read_buf = [0u8; ERASE_SIZE];
        match flash.blocking_read(offset, &mut read_buf) {
            Ok(_) => match from_bytes::<Self>(&mut read_buf) {
                Ok(c) => Ok(c),
                _ => {
                    defmt::info!("NO CONFIG");
                    let cfg: Self = Default::default();
                    cfg.save(flash).ok();
                    Ok(cfg)
                }
            },
            Err(_) => Err(Error::Load),
        }
    }

    pub fn save(&self, flash: &mut Flash<'_, FLASH, flash::Blocking, FLASH_SIZE>) -> Result<(), Error> {
        let from = unsafe { &__config_start as *const u32 as u32 };
        let to = unsafe { &__config_end as *const u32 as u32 };
        let mut buf: AlignedBuffer<ERASE_SIZE> = AlignedBuffer([0; ERASE_SIZE]);
        match to_slice(&self, &mut buf.0) {
            Ok(bytes) => flash
                .blocking_erase(from, to)
                .and_then(|_| flash.blocking_write(from, bytes))
                .map_err(|_| Error::Save),
            Err(_) => Err(Error::Save),
        }
    }

    pub fn update_reg(
        flash: &mut Flash<'_, FLASH, flash::Blocking, FLASH_SIZE>,
        reg: Register,
        val: u8,
    ) -> Result<(), Error> {
        let mut cfg = Self::load(flash)?;
        match reg {
            Register::THRESH_ACT => cfg.THRESH_ACT = val.max(1),
            Register::TIME_INACT => cfg.TIME_INACT = val,
            Register::THRESH_TAP => cfg.THRESH_TAP = val.max(1),
            Register::DUR => cfg.DUR = val,
            _ => (),
        }
        cfg.save(flash)
    }
}

impl<'a> Default for Config {
    fn default() -> Self {
        Self {
            version: String::from("v1"),
            heartbeat_interval: 5,
            THRESH_ACT: 10,
            TIME_INACT: 2,
            THRESH_TAP: 220,
            DUR: 240,
        }
    }
}
