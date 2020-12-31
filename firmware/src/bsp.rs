//! Board Support Package (pins and peripherals)

use stm32wb_hal::pac::{I2C1, I2C3};
use stm32wb_hal::gpio::gpiob::{PB6, PB7, PB4};
use stm32wb_hal::gpio::{Alternate, AF4, Output, OpenDrain};
use stm32wb_hal::i2c::I2c;
use stm32wb_hal::gpio::gpioa::PA7;

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
