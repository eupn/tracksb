//! Board Support Package (pins and peripherals)

use cortex_m::peripheral::NVIC;
use stm32wb_hal::{
    gpio::{
        gpioa::PA7,
        gpiob::{PB1, PB3, PB4, PB6, PB7},
        Alternate, Edge, ExtiPin, Input, OpenDrain, Output, PullUp, AF4,
    },
    i2c::I2c,
    pac::{Interrupt, EXTI, I2C1, I2C3, SYSCFG},
};

pub type PmicI2c = I2c<
    I2C1,
    (
        PB6<Alternate<AF4, Output<OpenDrain>>>,
        PB7<Alternate<AF4, Output<OpenDrain>>>,
    ),
>;

pub type ImuI2c = I2c<
    I2C3,
    (
        PA7<Alternate<AF4, Output<OpenDrain>>>,
        PB4<Alternate<AF4, Output<OpenDrain>>>,
    ),
>;

pub type ImuIntPin = PB3<Input<PullUp>>;

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
