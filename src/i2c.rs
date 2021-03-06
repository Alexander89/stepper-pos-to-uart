use cortex_m::interrupt;
use hal::{
    clock::GenericClockController,
    pac,
    prelude::*,
    sercom::{I2CError, I2CMaster0},
    time::Hertz,
};
use xiao_m0::{hal, Scl, Sda, I2C};

pub struct I2c {
    main: I2C,
}

impl I2c {
    pub fn init(
        clocks: &mut GenericClockController,
        sercom: pac::SERCOM0,
        pm: &mut pac::PM,
        sda: impl Into<Sda>,
        scl: impl Into<Scl>,
    ) -> Self {
        let gclk0 = clocks.gclk0();
        let clock = &clocks.sercom0_core(&gclk0).unwrap();
        let freq: Hertz = 1.mhz().into();
        let main = I2CMaster0::new(clock, freq, sercom, pm, sda.into(), scl.into());
        Self { main }
    }

    pub fn i2c_read_some(
        &mut self,
        address: u8,
        from: u8,
        count: usize,
        buffer: &mut [u8],
    ) -> Result<(), I2CError> {
        interrupt::free(|_| self.main.write(address, &[from]))?;
        for i in 0..count {
            let mut res = [0u8];
            interrupt::free(|_| self.main.read(address, &mut res))?;
            buffer[i] = res[0];
        }
        Ok(())
    }

    pub fn i2c_query(&mut self, address: u8, from: u8) -> Result<(), I2CError> {
        interrupt::free(|_| self.main.write(address, &[from]))?;
        Ok(())
    }

    pub fn i2c_read(
        &mut self,
        address: u8,
        count: usize,
        buffer: &mut [u8],
    ) -> Result<(), I2CError> {
        for i in 0..count {
            let mut res = [0u8];
            interrupt::free(|_| self.main.read(address, &mut res))?;
            buffer[i] = res[0];
        }
        Ok(())
    }
    pub fn i2c_read_one(&mut self, address: u8) -> Result<u8, I2CError> {
        let mut res = [0u8];
        interrupt::free(|_| self.main.read(address, &mut res))?;
        Ok(res[0])
    }
}
