use bno080::{
    interface::I2cInterface,
    wrapper::{WrapperError, BNO080},
};
use core::marker::PhantomData;
use embedded_hal::blocking::{
    delay::DelayMs,
    i2c::{Read, Write, WriteRead},
};

pub trait ImuState {}
pub struct Created;
pub struct Initialized;
impl ImuState for Created {}
impl ImuState for Initialized {}

pub type Quaternion = [f32; 4];
pub type ImuError<E> = WrapperError<bno080::Error<E, ()>>;

pub struct Imu<
    S: ImuState,
    E: core::fmt::Debug,
    I: Read<Error = E> + WriteRead<Error = E> + Write<Error = E>,
> {
    _s: PhantomData<S>,
    bno: BNO080<I2cInterface<I>>,
}

pub struct ImuBuilder<
    E: core::fmt::Debug,
    I: Read<Error = E> + WriteRead<Error = E> + Write<Error = E>,
> {
    _e: PhantomData<E>,
    _i: PhantomData<I>,
}

impl<E: core::fmt::Debug, I: Read<Error = E> + WriteRead<Error = E> + Write<Error = E>>
    ImuBuilder<E, I>
{
    pub fn new(i2c: I) -> Imu<Created, E, I> {
        let i2c_interface = I2cInterface::default(i2c);
        let imu_driver = BNO080::new_with_interface(i2c_interface);

        Imu {
            _s: PhantomData,
            bno: imu_driver,
        }
    }
}

impl<E: core::fmt::Debug, I: Read<Error = E> + WriteRead<Error = E> + Write<Error = E>>
    Imu<Created, E, I>
{
    pub fn init(
        mut self,
        delay: &mut impl DelayMs<u8>,
        interval_ms: u16,
    ) -> Result<Imu<Initialized, E, I>, ImuError<E>> {
        self.bno.init(delay)?;
        self.bno.enable_rotation_vector(interval_ms)?;

        Ok(Imu {
            _s: PhantomData,
            bno: self.bno,
        })
    }
}

impl<E: core::fmt::Debug, I: Read<Error = E> + WriteRead<Error = E> + Write<Error = E>>
    Imu<Initialized, E, I>
{
    pub fn quaternion(
        &mut self,
        delay: &mut impl DelayMs<u8>,
    ) -> Result<Option<Quaternion>, ImuError<E>> {
        let msg_count = self.bno.handle_all_messages(delay, 1u8);
        if msg_count > 0 {
            return Ok(Some(self.bno.rotation_quaternion()?));
        }

        Ok(None)
    }
}
