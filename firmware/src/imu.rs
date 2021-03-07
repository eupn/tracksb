use core::{convert::Infallible, marker::PhantomData};

use async_embedded_traits::{
    delay::AsyncDelayMs,
    i2c::{AsyncI2cRead, AsyncI2cTransfer, AsyncI2cWrite, I2cAddress7Bit},
};
use bno080::{
    interface::I2cInterface,
    wrapper::{WrapperError, BNO080},
};
use embassy_stm32wb55::i2c::StopDma;
use embedded_hal::digital::v2::OutputPin;

pub const IMU_REPORTING_RATE_HZ: u16 = 50;
pub const IMU_REPORTING_INTERVAL_MS: u16 = 1000 / IMU_REPORTING_RATE_HZ;

pub trait ImuState {}
pub struct Created;
pub struct Initialized;
impl ImuState for Created {}
impl ImuState for Initialized {}

pub type Quaternion = [i16; 4];
pub type Vec3 = [i16; 3];
pub type ImuError<E> = WrapperError<E>;

pub trait AsyncI2c<E: core::fmt::Debug> = AsyncI2cRead<I2cAddress7Bit, Error = E>
    + AsyncI2cTransfer<I2cAddress7Bit, Error = E>
    + AsyncI2cWrite<I2cAddress7Bit, Error = E>
    + StopDma;

#[derive(Copy, Clone, Default)]
pub struct MotionData {
    pub quat: Quaternion,
    pub accel: Vec3,
    pub gyro: Vec3,
}

pub enum ImuOrBusState {
    I2c,
    Imu,
}

pub struct ImuOrBus<E: core::fmt::Debug, I: Sized + AsyncI2c<E>> {
    _e: PhantomData<E>,
    pub state: ImuOrBusState,
    pub imu: Option<Imu<Initialized, E, I>>,
    pub bus: Option<I>,
}

impl<E: core::fmt::Debug, I: Sized + AsyncI2c<E>> ImuOrBus<E, I> {
    pub fn i2c(i2c: I) -> Self {
        Self {
            _e: PhantomData,
            state: ImuOrBusState::I2c,
            imu: None,
            bus: Some(i2c),
        }
    }

    pub fn imu(imu: Imu<Initialized, E, I>) -> Self {
        Self {
            _e: PhantomData,
            state: ImuOrBusState::Imu,
            imu: Some(imu),
            bus: None,
        }
    }
}

/// Holds reset pin and either initialized I2C bus or initialized IMU that consumes I2C bus.
/// Used to first initialize I2C bus and then wait for "boot" interrupt
/// from the uninitialized IMU chip that will trigger the IMU initialization.
pub struct ImuWrapper<
    E: core::fmt::Debug,
    I: Sized + AsyncI2c<E>,
    RST: OutputPin<Error = Infallible>,
> {
    reset_pin: RST,
    inner: ImuOrBus<E, I>,
}

impl<E: core::fmt::Debug, I: Sized + AsyncI2c<E>, RST: OutputPin<Error = Infallible>>
    ImuWrapper<E, I, RST>
{
    pub fn new(i2c: I, reset_pin: RST) -> Self {
        Self {
            reset_pin,
            inner: ImuOrBus::i2c(i2c),
        }
    }

    /// Call this from IMU's start-up interrupt handler to finish its initialization.
    /// It will internally convert itself from an I2C bus to an IMU that consumes it.
    pub async fn init_imu(&mut self, delay: &mut impl AsyncDelayMs<u8>, interval_ms: u16) {
        if let ImuOrBusState::I2c = &mut self.inner.state {
            // Obtains an owned instance of an I2C that doesn't implement Copy
            let owned_i2c = unsafe { core::ptr::read(self.inner.bus.as_ref().unwrap()) };

            let imu = ImuBuilder::create(owned_i2c);
            let imu = imu.init(delay, interval_ms).await.unwrap();

            self.inner = ImuOrBus::imu(imu);
        } else {
            // Shouldn't happen since each call to this function should prevent it from
            // being called again
            unreachable!()
        }
    }

    /// Call this to de-initialize the IMU and get an I2C port back.
    pub fn deinit(&mut self) {
        if let ImuOrBusState::Imu = &mut self.inner.state {
            // Obtains an owned instance of the IMU that doesn't implement Copy
            let owned_imu = unsafe { core::ptr::read(self.inner.imu.as_ref().unwrap()) };

            let async_i2c = owned_imu.bno.free();
            self.inner = ImuOrBus::i2c(async_i2c);
        } else {
            // Shouldn't happen since each call to this function should prevent it from
            // being called again
            unreachable!()
        }
    }

    pub async fn reset_imu(&mut self, delay: &mut impl AsyncDelayMs<u8>) {
        defmt::info!("Resetting IMU");
        self.reset_pin.set_low().unwrap();
        delay.async_delay_ms(100).await;
        self.reset_pin.set_high().unwrap();
    }

    pub fn is_initialized(&self) -> bool {
        matches!(self.inner.state, ImuOrBusState::Imu)
    }

    pub async fn motion_data(
        &mut self,
        delay: &mut impl AsyncDelayMs<u8>,
    ) -> Result<Option<MotionData>, ImuError<E>> {
        if let ImuOrBusState::Imu = &mut self.inner.state {
            Ok(if let Some(imu) = &mut self.inner.imu {
                let data = imu.motion_data(delay).await?;
                data
            } else {
                None
            })
        } else {
            Ok(None)
        }
    }

    pub async fn set_reporting_interval(&mut self, interval_ms: u16) -> Result<(), ImuError<E>> {
        if let ImuOrBusState::Imu = &mut self.inner.state {
            if let Some(imu) = &mut self.inner.imu {
                imu.set_reporting_interval(interval_ms).await?;
            }
        }

        Ok(())
    }
}

pub struct Imu<S: ImuState, E: core::fmt::Debug, I: AsyncI2c<E>> {
    _e: PhantomData<E>,
    _s: PhantomData<S>,
    bno: BNO080<E, I>,
}

pub struct ImuBuilder<E: core::fmt::Debug, I: AsyncI2c<E>> {
    _e: PhantomData<E>,
    _i: PhantomData<I>,
}

impl<E: core::fmt::Debug, I: AsyncI2c<E>> ImuBuilder<E, I> {
    pub fn create(i2c: I) -> Imu<Created, E, I> {
        let i2c_interface = I2cInterface::default(i2c);
        let imu_driver = BNO080::new_with_interface(i2c_interface);

        Imu {
            _s: PhantomData,
            _e: PhantomData,
            bno: imu_driver,
        }
    }
}

impl<E: core::fmt::Debug, I: AsyncI2c<E>> Imu<Created, E, I> {
    pub async fn init(
        mut self,
        delay: &mut impl AsyncDelayMs<u8>,
        interval_ms: u16,
    ) -> Result<Imu<Initialized, E, I>, ImuError<E>> {
        self.bno.init(delay).await?;

        let mut imu = Imu::<Initialized, E, I> {
            _s: PhantomData,
            _e: PhantomData,
            bno: self.bno,
        };
        imu.set_reporting_interval(interval_ms).await?;
        Ok(imu)
    }
}

impl<E: core::fmt::Debug, I: AsyncI2c<E>> Imu<Initialized, E, I> {
    pub async fn set_reporting_interval(&mut self, interval_ms: u16) -> Result<(), ImuError<E>> {
        defmt::info!("Setting reporting interval at {} ms", interval_ms);

        self.bno.enable_rotation_vector(interval_ms).await?;
        defmt::info!("Enabling accel");
        self.bno.enable_linear_accel(interval_ms).await?;
        defmt::info!("Enabling gyro");
        self.bno.enable_gyro(interval_ms).await?;

        Ok(())
    }

    pub async fn motion_data(
        &mut self,
        delay: &mut impl AsyncDelayMs<u8>,
    ) -> Result<Option<MotionData>, ImuError<E>> {
        let msg_count = self.bno.handle_all_messages(delay, 10u8).await;
        if msg_count > 0 {
            return Ok(Some(MotionData {
                quat: self.bno.rotation_quaternion_fixed()?,
                accel: self.bno.linear_accel_fixed()?,
                gyro: self.bno.gyro_fixed()?,
            }));
        }

        Ok(None)
    }
}
