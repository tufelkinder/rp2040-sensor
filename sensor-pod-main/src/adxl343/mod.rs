//! Async ADXL343 accelerometer driver (I2C)
#![allow(dead_code, non_snake_case, clippy::unreadable_literal)]
pub mod register;
use defmt::Format;
use embedded_hal_async::i2c::I2c;
use register::Register;
pub use register::{DataFormatFlags, DataFormatRange};
use serde::{Deserialize, Serialize};

/// ADXL343 I2C address.
/// Assumes ALT address pin low
pub const ADDRESS: u8 = 0x53;

/// ADXL343 device ID
pub const DEVICE_ID: u8 = 0xE5;

/// Acceleration data struct
#[derive(Format, Serialize, Deserialize)]
pub struct AccelData {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

/// Interrupt source
pub struct IntSource(u8);

impl IntSource {
    pub fn is_single_tap(&self) -> bool {
        self.0 & (1 << 6) != 0
    }
    pub fn is_double_tap(&self) -> bool {
        self.0 & (1 << 5) != 0
    }
    pub fn is_activity(&self) -> bool {
        self.0 & (1 << 4) != 0
    }
    pub fn is_inactivity(&self) -> bool {
        self.0 & (1 << 3) != 0
    }
}

#[derive(Format, Serialize, Deserialize, Clone)]
pub struct Config {
    pub THRESH_ACT: u8,
    pub TIME_INACT: u8,
    pub THRESH_TAP: u8,
    pub DUR: u8,
}

/// ADXL343 driver
pub struct Adxl343<I2C> {
    /// Underlying I2C device
    i2c: I2C,

    /// Current data format
    data_format: DataFormatFlags,
}

impl<I2C, E> Adxl343<I2C>
where
    I2C: I2c<Error = E>,
    E: Format,
{
    /// Create a new ADXL343 driver from the given I2C peripheral
    pub async fn new(i2c: I2C, config: Config) -> Result<Self, E> {
        Self::new_with_data_format(i2c, DataFormatFlags::default(), config).await
    }

    /// Create a new ADXL343 driver configured with the given data format
    pub async fn new_with_data_format<F>(i2c: I2C, data_format: F, config: Config) -> Result<Self, E>
    where
        F: Into<DataFormatFlags>,
    {
        let mut adxl343 = Adxl343 {
            i2c,
            data_format: data_format.into(),
        };

        // Ensure we have the correct device ID for the ADLX343
        if adxl343.get_device_id().await? != DEVICE_ID {
            panic!("ID");
        }

        // Configure the data format
        adxl343.data_format(adxl343.data_format).await?;

        // Enable AC mode + XYZ axis for activity/inactivity
        adxl343.write_register(Register::ACT_INACT_CTL, 0b1111_1111).await?;
        // 62.5 mg/LSB
        adxl343.write_register(Register::THRESH_ACT, config.THRESH_ACT).await?;
        // 62.5 mg/LSB
        adxl343
            .write_register(Register::THRESH_INACT, config.THRESH_ACT)
            .await?;
        // Inactivity period: 1s/LSB
        adxl343.write_register(Register::TIME_INACT, config.TIME_INACT).await?;

        // Tap threshold 62.5 mg/LSB
        adxl343.write_register(Register::THRESH_TAP, config.THRESH_TAP).await?;
        // Tap duration: 625 µs/LSB
        adxl343.write_register(Register::DUR, config.DUR).await?;
        // Tap latency: 1.25 ms/LSB (0 = no double tap)
        adxl343.write_register(Register::LATENT, 0).await?;
        // Waiting period: 1.25 ms/LSB (0 = no double tap)
        adxl343.write_register(Register::WINDOW, 0).await?;
        // Enable XYZ axis for tap
        adxl343.write_register(Register::TAP_AXES, 0b0000_0111).await?;
        // Map TAP/ACT/INACT interrupts to INT1
        adxl343.write_register(Register::INT_MAP, 0b1010_0111).await?;
        // Enable TAP/ACT/INACT interrupts
        adxl343.write_register(Register::INT_ENABLE, 0b0101_1000).await?;

        // Link ACT/INACT and enable measurements
        adxl343.write_register(Register::POWER_CTL, 0b0010_1000).await?;

        // Clear any pending interrupt
        let mut buf = [0];
        adxl343.write_read_register(Register::INT_SOURCE, &mut buf).await?;

        Ok(adxl343)
    }

    /// Get interrupt source(s)
    pub async fn interrupt_source(&mut self) -> Result<IntSource, E> {
        let mut buf = [0];
        self.write_read_register(Register::INT_SOURCE, &mut buf).await?;
        Ok(IntSource(buf[0]))
    }

    /// Set the device data format
    pub async fn data_format<F>(&mut self, data_format: F) -> Result<(), E>
    where
        F: Into<DataFormatFlags>,
    {
        let f = data_format.into();
        let input = [Register::DATA_FORMAT.addr(), f.bits()];
        self.i2c.write(ADDRESS, &input).await?;
        self.data_format = f;
        Ok(())
    }

    /// Write to the given register
    pub async fn write_register(&mut self, register: Register, value: u8) -> Result<(), E> {
        debug_assert!(!register.read_only(), "can't write to read-only register");
        self.i2c.write(ADDRESS, &[register.addr(), value]).await?;
        Ok(())
    }

    /// Write to a given register, then read the result
    pub async fn write_read_register(&mut self, register: Register, buffer: &mut [u8]) -> Result<(), E> {
        self.i2c.write_read(ADDRESS, &[register.addr()], buffer).await
    }

    /// Get the device ID
    pub async fn get_device_id(&mut self) -> Result<u8, E> {
        let input = [Register::DEVID.addr()];
        let mut output = [0u8];
        self.i2c.write_read(ADDRESS, &input, &mut output).await?;
        Ok(output[0])
    }

    /// Set self test on/off
    pub async fn self_test(&mut self, active: bool) -> Result<(), E> {
        let default = self.data_format.bits();
        let val = if active { default | (1 << 7) } else { default };
        self.write_register(Register::DATA_FORMAT, val).await?;
        Ok(())
    }

    /// Write to a given register, then read a `i16` result
    ///
    /// From the ADXL343 data sheet (p.25):
    /// <https://www.analog.com/media/en/technical-documentation/data-sheets/adxl343.pdf>
    ///
    /// "The output data is twos complement, with DATAx0 as the least
    /// significant byte and DATAx1 as the most significant byte"
    // #[cfg(feature = "i16x3")]
    async fn write_read_i16(&mut self, register: Register) -> Result<i16, E> {
        let mut buffer = [0u8; 2];
        self.write_read_register(register, &mut buffer).await?;
        Ok(i16::from_be_bytes(buffer))
    }

    /// Write to a given register, then read a `u16` result
    ///
    /// Used for reading `JUSTIFY`-mode data. From the ADXL343 data sheet (p.25):
    /// <https://www.analog.com/media/en/technical-documentation/data-sheets/adxl343.pdf>
    ///
    /// "A setting of 1 in the justify bit selects left-justified (MSB) mode,
    /// and a setting of 0 selects right-justified mode with sign extension."
    // #[cfg(feature = "u16x3")]
    async fn write_read_u16(&mut self, register: Register) -> Result<u16, E> {
        let mut buffer = [0u8; 2];
        self.write_read_register(register, &mut buffer).await?;
        Ok(u16::from_le_bytes(buffer))
    }

    /// Get acceleration reading from the accelerometer
    pub async fn accel_raw_i16(&mut self) -> Result<AccelData, E> {
        if self.data_format.contains(DataFormatFlags::JUSTIFY) {
            panic!("MODE");
        }

        let x = self.write_read_i16(Register::DATAX0).await?;
        let y = self.write_read_i16(Register::DATAY0).await?;
        let z = self.write_read_i16(Register::DATAZ0).await?;

        Ok(AccelData { x, y, z })
    }

    /// Get acceleration reading from the accelerometer
    pub async fn accel_raw_u16(&mut self) -> Result<(u16, u16, u16), E> {
        if self.data_format.contains(DataFormatFlags::JUSTIFY) {
            panic!("MODE");
        }

        let x = self.write_read_u16(Register::DATAX0).await?;
        let y = self.write_read_u16(Register::DATAY0).await?;
        let z = self.write_read_u16(Register::DATAZ0).await?;

        Ok((x, y, z))
    }

    pub async fn accel_avg_i16(&mut self, n: u8) -> Result<AccelData, E> {
        let (mut sum_x, mut sum_y, mut sum_z) = (0_i32, 0_i32, 0_i32);
        for _ in 0..n {
            let AccelData { x, y, z } = self.accel_raw_i16().await?;
            sum_x += x as i32;
            sum_y += y as i32;
            sum_z += z as i32;
        }
        let n = n as i32;
        Ok(AccelData {
            x: (sum_x / n) as _,
            y: (sum_y / n) as _,
            z: (sum_z / n) as _,
        })
    }

    pub async fn accel_avg_u16(&mut self, n: u8) -> Result<(u16, u16, u16), E> {
        let (mut sum_x, mut sum_y, mut sum_z) = (0_u32, 0_u32, 0_u32);
        for _ in 0..n {
            let (x, y, z) = self.accel_raw_u16().await?;
            sum_x += x as u32;
            sum_y += y as u32;
            sum_z += z as u32;
        }
        let n = n as u32;
        Ok(((sum_x / n) as _, (sum_y / n) as _, (sum_z / n) as _))
    }
}
