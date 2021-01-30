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

pub const IMU_REPORTING_RATE_HZ: u16 = 10;
pub const IMU_REPORTING_INTERVAL_MS: u16 = 1000 / IMU_REPORTING_RATE_HZ;

pub trait ImuState {}
pub struct Created;
pub struct Initialized;
impl ImuState for Created {}
impl ImuState for Initialized {}

pub type Quaternion = [f32; 4];
pub type Vec3 = [f32; 3];
pub type ImuError<E> = WrapperError<bno080::Error<E, ()>>;

pub struct MotionData {
    pub quat: Quaternion,
    pub accel: Vec3,
    pub gyro: Vec3,
}

pub enum ImuOrBus<
    E: core::fmt::Debug,
    I: Sized + Read<Error = E> + WriteRead<Error = E> + Write<Error = E>,
> {
    I2c(I),
    Imu(Imu<Initialized, E, I>),
}

/// Holds reset pin and either initialized I2C bus or initialized IMU that consumes I2C bus.
/// Used to first initialize I2C bus and then wait for "boot" interrupt
/// from the uninitialized IMU chip that will trigger the IMU initialization.
pub struct ImuWrapper<
    E: core::fmt::Debug,
    I: Sized + Read<Error = E> + WriteRead<Error = E> + Write<Error = E>,
    RST: OutputPin<Error = Infallible>,
> {
    reset_pin: RST,
    inner: ImuOrBus<E, I>,
}

impl<
        E: core::fmt::Debug,
        I: Sized + Read<Error = E> + WriteRead<Error = E> + Write<Error = E>,
        RST: OutputPin<Error = Infallible>,
    > ImuWrapper<E, I, RST>
{
    pub fn new(i2c: I, reset_pin: RST) -> Self {
        Self {
            reset_pin,
            inner: ImuOrBus::I2c(i2c),
        }
    }

    /// Call this from IMU's start-up interrupt handler to finish its initialization.
    /// It will internally convert itself from an I2C bus to an IMU that consumes it.
    pub fn init_imu(&mut self, delay: &mut impl DelayMs<u8>, interval_ms: u16) {
        if let ImuOrBus::I2c(i2c) = &mut self.inner {
            // Obtains an owned instance of an I2C that doesn't implement Copy
            let owned_i2c = unsafe { core::ptr::read(i2c) };

            let imu = ImuBuilder::create(owned_i2c);
            let imu = imu.init(delay, interval_ms).unwrap();

            self.inner = ImuOrBus::Imu(imu);
        } else {
            // Shouldn't happen since each call to this function should prevent it from
            // being called again
            unreachable!()
        }
    }

    /// Call this to de-initialize the IMU and get an I2C port back.
    pub fn deinit(&mut self) {
        if let ImuOrBus::Imu(imu) = &mut self.inner {
            // Obtains an owned instance of the IMU that doesn't implement Copy
            let owned_imu = unsafe { core::ptr::read(imu) };

            self.inner = ImuOrBus::I2c(owned_imu.bno.free().free());
        } else {
            // Shouldn't happen since each call to this function should prevent it from
            // being called again
            unreachable!()
        }
    }

    pub fn reset_imu(&mut self, delay: &mut impl DelayMs<u8>) {
        self.reset_pin.set_low().unwrap();
        delay.delay_ms(100);
        self.reset_pin.set_high().unwrap();
    }

    pub fn is_initialized(&self) -> bool {
        matches!(self.inner, ImuOrBus::Imu(_))
    }

    pub fn motion_data(
        &mut self,
        delay: &mut impl DelayMs<u8>,
    ) -> Result<Option<MotionData>, ImuError<E>> {
        if let ImuOrBus::Imu(imu) = &mut self.inner {
            Ok(imu.motion_data(delay)?)
        } else {
            Ok(None)
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
    pub fn create(i2c: I) -> Imu<Created, E, I> {
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

        defmt::info!("Enabling accel");
        self.bno.enable_linear_accel(interval_ms)?;

        defmt::info!("Enabling gyro");
        self.bno.enable_gyro(interval_ms)?;

        Ok(Imu {
            _s: PhantomData,
            bno: self.bno,
        })
    }
}

impl<E: core::fmt::Debug, I: Read<Error = E> + WriteRead<Error = E> + Write<Error = E>>
    Imu<Initialized, E, I>
{
    pub fn motion_data(
        &mut self,
        delay: &mut impl DelayMs<u8>,
    ) -> Result<Option<MotionData>, ImuError<E>> {
        let msg_count = self.bno.handle_all_messages(delay, 10u8);
        if msg_count > 0 {
            return Ok(Some(MotionData {
                quat: self.bno.rotation_quaternion()?,
                accel: self.bno.linear_accel()?,
                gyro: self.bno.gyro()?,
            }));
        }

        Ok(None)
    }
}
