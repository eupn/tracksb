//! Board Support Package (pins and peripherals)

use stm32wb_hal::{
    gpio::{
        gpioa::PA7,
        gpiob::{PB4, PB6, PB7},
        Alternate, OpenDrain, Output, AF4,
    },
    i2c::I2c,
    pac::{I2C1, I2C3},
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
