//! Board Support Package (pins and peripherals)

use crate::rgbled::RgbLed;
use cortex_m::peripheral::NVIC;
use embassy_stm32wb55::i2c::{i2c1::AsyncI2c as AsyncI2c1, i2c3::AsyncI2c as AsyncI2c3};
use stm32wb_hal::{
    gpio::{
        gpioa::{PA15, PA4, PA5, PA6, PA7},
        gpiob::{PB1, PB3, PB4, PB5, PB6, PB7},
        Alternate, Edge, ExtiPin, Input, OpenDrain, Output, PullUp, PushPull, AF4,
    },
    pac::{Interrupt, EXTI, I2C1, I2C3, SYSCFG},
    time::Hertz,
};

pub type RedLedPin = PA4<Output<PushPull>>;
pub type GreenLedPin = PA5<Output<PushPull>>;
pub type BlueLedPin = PA6<Output<PushPull>>;
pub type Rgb = RgbLed<RedLedPin, GreenLedPin, BlueLedPin>;

pub type PullUpsPin = PB5<Output<PushPull>>;
pub type PmicI2cPort = I2C1;
pub type PmicI2cSclPin = PB6<Alternate<AF4, Output<OpenDrain>>>;
pub type PmicI2cSdaPin = PB7<Alternate<AF4, Output<OpenDrain>>>;
pub const PMIC_I2C_SPEED: Hertz = Hertz(401_000);

pub type PmicI2c = AsyncI2c1<
    PmicI2cPort,
    PmicI2cSclPin,
    PmicI2cSdaPin,
    embassy_stm32wb55::interrupt::DMA1_CHANNEL1,
    embassy_stm32wb55::interrupt::I2C1_ER,
    embassy_stm32wb55::interrupt::I2C1_EV,
>;

pub type PmicIntPin = PB1<Input<PullUp>>;

pub fn init_pmic_interrupt(
    mut pmic_int_pin: PmicIntPin,
    syscfg: &mut SYSCFG,
    exti: &mut EXTI,
) -> PmicIntPin {
    pmic_int_pin.make_interrupt_source(syscfg);
    pmic_int_pin.trigger_on_edge(exti, Edge::FALLING);
    pmic_int_pin.enable_interrupt(exti);
    unsafe {
        NVIC::unmask(Interrupt::EXTI1);
    }

    pmic_int_pin
}

pub type ImuI2c = AsyncI2c3<
    I2C3,
    ImuSclPin,
    ImuSdaPin,
    embassy_stm32wb55::interrupt::DMA1_CHANNEL2,
    embassy_stm32wb55::interrupt::I2C3_ER,
    embassy_stm32wb55::interrupt::I2C3_EV,
>;

pub type ImuIntPin = PB3<Input<PullUp>>;
pub type ImuResetPin = PA15<Output<PushPull>>;
pub type ImuSclPin = PA7<Alternate<AF4, Output<OpenDrain>>>;
pub type ImuSdaPin = PB4<Alternate<AF4, Output<OpenDrain>>>;
pub const IMU_I2C_SPEED: Hertz = Hertz(800_000);

pub fn init_imu_interrupt(
    mut imu_int_pin: ImuIntPin,
    syscfg: &mut SYSCFG,
    exti: &mut EXTI,
) -> ImuIntPin {
    imu_int_pin.make_interrupt_source(syscfg);
    imu_int_pin.trigger_on_edge(exti, Edge::FALLING);
    imu_int_pin.enable_interrupt(exti);
    unsafe {
        NVIC::unmask(Interrupt::EXTI3);
    }

    imu_int_pin
}
