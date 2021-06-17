#![no_std]
#![feature(trait_alias)]
#![feature(min_type_alias_impl_trait)]

use core::sync::atomic::{AtomicUsize, Ordering};

use defmt_persist as _; // global logger

use stm32wb_hal as _; // memory layout

//use panic_reset as _;
use panic_probe as _;

pub mod ble;
pub mod bsp;
pub mod imu;
pub mod led;
pub mod pmic;
pub mod storage;

defmt::timestamp! {"{=u64}", {
        static COUNT: AtomicUsize = AtomicUsize::new(0);
        // NOTE(no-CAS) `timestamps` runs with interrupts disabled
        let n = COUNT.load(Ordering::Relaxed);
        COUNT.store(n + 1, Ordering::Relaxed);
        n as u64
    }
}

/// Terminates the application and makes `probe-run` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}
