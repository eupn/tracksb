use bno080::wrapper::{BNO080, WrapperError};
use bno080::interface::I2cInterface;
use embedded_hal::blocking::i2c::{WriteRead, Read, Write};
use embedded_hal::blocking::delay::DelayMs;

pub type Quaternion = [f32; 4];
pub type ImuError<E> = WrapperError<bno080::Error<E, ()>>;

pub struct Imu<E: core::fmt::Debug, I: Read<Error = E> + WriteRead<Error = E> + Write<Error = E>> {
    bno: BNO080<I2cInterface<I>>,
}

impl<E: core::fmt::Debug, I: Read<Error = E> + WriteRead<Error = E> + Write<Error = E>> Imu<E, I> {
    pub fn new(i2c: I) -> Self {
        let i2c_interface = I2cInterface::default(i2c);
        let imu_driver = BNO080::new_with_interface(i2c_interface);

        Self {
            bno: imu_driver,
        }
    }

    pub fn init(&mut self, delay: &mut impl DelayMs<u8>, interval_ms: u16) -> Result<(), ImuError<E>> {
        self.bno.init(delay)?;
        self.bno.enable_rotation_vector(interval_ms)?;

        Ok(())
    }

    pub fn quaternion(&mut self, delay: &mut impl DelayMs<u8>) -> Result<Option<Quaternion>, ImuError<E>> {
        let msg_count = self.bno.handle_all_messages(delay, 1u8);
        if msg_count > 0 {
            return Ok(Some(self.bno.rotation_quaternion()?));
        }

        Ok(None)
    }
}