use core::{convert::Infallible, marker::PhantomData};

use bno080::{
    interface::I2cInterface,
    wrapper::{WrapperError, BNO080},
};
use embedded_hal::{
    blocking::{
        delay::DelayMs,
        i2c::{Read, Write, WriteRead},
    },
    digital::v2::OutputPin,
};

pub trait ImuState {}
pub struct Created;
pub struct Initialized;
impl ImuState for Created {}
impl ImuState for Initialized {}

pub type Quaternion = [f32; 4];
pub type ImuError<E> = WrapperError<bno080::Error<E, ()>>;

/// Holds either initialized I2C bus or initialized IMU that consumes I2C bus.
/// Used to first initialize I2C bus and then wait for "boot" interrupt
/// from the uninitialized IMU chip that will trigger the IMU initialization.
// TODO: extract reset pin from the enum
pub enum I2cOrImu<
    E: core::fmt::Debug,
    I: Sized + Read<Error = E> + WriteRead<Error = E> + Write<Error = E>,
    RST: OutputPin<Error = Infallible>,
> {
    I2c(I, RST),
    Imu(Imu<Initialized, E, I>, RST),
}

impl<
        E: core::fmt::Debug,
        I: Sized + Read<Error = E> + WriteRead<Error = E> + Write<Error = E>,
        RST: OutputPin<Error = Infallible>,
    > I2cOrImu<E, I, RST>
{
    /// Call this from IMU's start-up interrupt handler to finish its initialization.
    /// It will internally convert itself from an I2C bus to an IMU that consumes it.
    pub fn init_imu(&mut self, delay: &mut impl DelayMs<u8>, interval_ms: u16) {
        if let I2cOrImu::I2c(i2c, rst) = self {
            // Obtains an owned instance of an I2C that doesn't implement Copy
            let owned_i2c = unsafe { core::ptr::read(i2c) };
            let owned_rst = unsafe { core::ptr::read(rst) };

            let imu = ImuBuilder::new(owned_i2c);
            let imu = imu.init(delay, interval_ms).unwrap();

            *self = I2cOrImu::Imu(imu, owned_rst);
        } else {
            // Shouldn't happen since each call to this function should prevent it from
            // being called again
            unreachable!()
        }
    }

    /// Call this to de-initialize the IMU and get an I2C port back.
    pub fn deinit(&mut self) {
        if let I2cOrImu::Imu(imu, rst) = self {
            // Obtains an owned instance of the IMU that doesn't implement Copy
            let owned_imu = unsafe { core::ptr::read(imu) };
            let owned_rst = unsafe { core::ptr::read(rst) };

            *self = I2cOrImu::I2c(owned_imu.bno.free().free(), owned_rst);
        } else {
            // Shouldn't happen since each call to this function should prevent it from
            // being called again
            unreachable!()
        }
    }
}

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
