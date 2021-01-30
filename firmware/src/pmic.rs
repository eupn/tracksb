//! Power Management IC (AXP173) routines

use crate::bsp;
use axp173::{
    AdcSampleRate, AdcSettings, Axp173, ChargingCurrent, Irq, Ldo, LdoKind, ShutdownLongPressTime,
    TsPinMode,
};
use core::marker::PhantomData;
use embedded_hal::blocking::{
    delay::DelayMs,
    i2c::{Write, WriteRead},
};
use stm32wb_hal::{
    i2c::{Error as I2cError, I2c},
    rcc::Rcc,
    time::U32Ext,
};

pub trait PmicState {}
pub struct Created;
pub struct Initialized {
    charge_coulombs_default: u32,
}

impl PmicState for Created {}
impl PmicState for Initialized {}

/// The capacity of the device's battery in mAhs.
const DEFAULT_BATTERY_CAPACITY_MAH: f32 = 120.0;

#[derive(Debug, Copy, Clone)]
pub enum ImuPowerState {
    Shutdown,
    Enabled,
    Unchanged,
}

pub struct Pmic<S: PmicState, E: core::fmt::Debug, I: WriteRead<Error = E> + Write<Error = E>> {
    axp173: Axp173<I>,
    state: S,
}

pub struct PmicBuilder<E: core::fmt::Debug, I: WriteRead<Error = E> + Write<Error = E>> {
    _e: PhantomData<E>,
    i2c: I,
}

impl<E: core::fmt::Debug, I: WriteRead<Error = E> + Write<Error = E>> PmicBuilder<E, I> {
    pub fn new(i2c: I) -> Self {
        Self {
            _e: PhantomData,
            i2c,
        }
    }

    pub fn free(self) -> I {
        self.i2c
    }

    pub fn check(&mut self) -> bool {
        Axp173::check(&mut self.i2c)
    }

    pub fn build(self) -> Pmic<Created, E, I> {
        let axp173 = Axp173::new(self.i2c);

        Pmic {
            axp173,
            state: Created,
        }
    }
}

impl<E: core::fmt::Debug, I: WriteRead<Error = E> + Write<Error = E>> Pmic<Created, E, I> {
    pub fn init(mut self) -> Result<Pmic<Initialized, E, I>, axp173::Error<E>> {
        // Check the presence of AXP173
        self.axp173.init()?;

        // Set charging current to 100mA
        self.axp173
            .set_charging_current(ChargingCurrent::CURRENT_100MA)?;

        // 25Hz sample rate, Disable TS, enable current sensing ADC
        self.axp173.set_adc_settings(
            AdcSettings::default()
                .set_adc_sample_rate(AdcSampleRate::RATE_25HZ)
                .ts_adc(false)
                .set_ts_pin_mode(TsPinMode::SHUT_DOWN)
                .vbus_voltage_adc(true)
                .vbus_current_adc(true)
                .batt_voltage_adc(true)
                .batt_current_adc(true),
        )?;

        self.axp173.set_coulomb_counter(true)?;
        self.axp173.resume_coulomb_counter()?;

        self.axp173
            .set_shutdown_long_press_time(ShutdownLongPressTime::SEC_4)?;
        self.axp173.set_shutdown_long_press(false)?;

        self.axp173.disable_ldo(&LdoKind::LDO4)?;

        self.axp173.clear_all_irq()?;
        self.enable_irqs()?;

        status(&mut self.axp173);

        Ok(Pmic {
            axp173: self.axp173,
            state: Initialized {
                charge_coulombs_default: axp173::mah_to_coulombs_adc(
                    DEFAULT_BATTERY_CAPACITY_MAH,
                    25.0,
                ) as u32,
            },
        })
    }

    /// Enables interesting IRQs from PMIC.
    fn enable_irqs(&mut self) -> Result<(), axp173::Error<E>> {
        self.axp173.set_irq(axp173::Irq::ButtonLongPress, true)?;
        self.axp173.set_irq(axp173::Irq::BatteryCharged, true)?;
        self.axp173.set_irq(axp173::Irq::LowBatteryWarning, true)?;
        self.axp173.set_irq(axp173::Irq::VbusPluggedIn, true)?;
        self.axp173.set_irq(axp173::Irq::VbusUnplugged, true)?;

        Ok(())
    }
}

impl<E: core::fmt::Debug, I: WriteRead<Error = E> + Write<Error = E>> Pmic<Initialized, E, I> {
    pub fn set_imu_power(&mut self, enable: bool) -> Result<(), axp173::Error<E>> {
        if enable {
            // Provide 2.8V for IMU via LDO3
            self.axp173.enable_ldo(&Ldo::ldo3_with_voltage(10, true))?;
        } else {
            self.axp173.disable_ldo(&LdoKind::LDO3)?;
        }

        Ok(())
    }

    pub fn imu_enabled(&mut self) -> Result<bool, axp173::Error<E>> {
        Ok(self.axp173.read_ldo(LdoKind::LDO3)?.enabled())
    }

    /// Called from PMIC IRQ pin ISR. Checks IRQ flags and clears pending IRQs.
    pub fn process_irqs(&mut self) -> Result<ImuPowerState, axp173::Error<E>> {
        let mut imu_power_state = ImuPowerState::Unchanged;

        if self.axp173.check_irq(Irq::ButtonLongPress)? {
            self.axp173.clear_irq(Irq::ButtonLongPress)?;

            if self.imu_enabled()? {
                defmt::info!("Shutting down");
                self.set_imu_power(false)?;
                imu_power_state = ImuPowerState::Shutdown;
            } else {
                defmt::info!("Turning on");
                self.set_imu_power(true)?;
                imu_power_state = ImuPowerState::Enabled;
            }
        }
        if self.axp173.check_irq(Irq::BatteryCharged)? {
            self.axp173.clear_irq(Irq::BatteryCharged)?;
        }
        if self.axp173.check_irq(Irq::LowBatteryWarning)? {
            self.axp173.clear_irq(Irq::LowBatteryWarning)?;
            let discharge_coulombs = self.axp173.read_discharge_coulomb_counter()?;
            defmt::info!("Low battery, discharge cc: {:u32}", discharge_coulombs);
        }
        if self.axp173.check_irq(Irq::VbusPluggedIn)? {
            self.axp173.clear_irq(Irq::VbusPluggedIn)?;
            defmt::info!("USB charger plugged in");
        }
        if self.axp173.check_irq(Irq::VbusUnplugged)? {
            self.axp173.clear_irq(Irq::VbusUnplugged)?;
            defmt::info!("USB charger unplugged");
        }

        // Clear other possible IRQs otherwise we can't rely on the IRQ pin
        self.axp173.clear_all_irq()?;

        Ok(imu_power_state)
    }

    /// Shows battery discharge current for reference purposes.
    pub fn show_current(&mut self) -> Result<(), axp173::Error<E>> {
        let batt_discharge = self.axp173.batt_discharge_current()?;
        defmt::info!(
            "Batt current consumption: {:f32} mA",
            batt_discharge.as_milliamps()
        );

        Ok(())
    }

    pub fn battery_level(&mut self) -> Result<u8, axp173::Error<E>> {
        let _coulombs_out = self.axp173.read_discharge_coulomb_counter()?;
        let mut coulombs_in = self.axp173.read_charge_coulomb_counter()?;

        // Assume full battery if there's not enough charge coulombs
        if coulombs_in < self.state.charge_coulombs_default {
            coulombs_in = self.state.charge_coulombs_default + 1;
        }

        let remaining_charge = self.axp173.estimate_charge_level(Some(coulombs_in))?;
        if let Some(remaining_charge) = remaining_charge {
            let level_pct = remaining_charge as f32 / DEFAULT_BATTERY_CAPACITY_MAH * 100_f32;
            let level_pct = (level_pct as u8).clamp(0, 100);

            // defmt::info!(
            //     "Charge level: {:?} [^ {:?} / v {:?}], draining {:?} mA",
            //     level_pct,
            //     coulombs_out,
            //     coulombs_in,
            //     self.axp173.batt_discharge_current()?.as_milliamps(),
            // );

            Ok(level_pct)
        } else {
            Ok(100)
        }
    }
}

// TODO: refactor waiting for PMIC turn-on via button
// TODO: probe PWROK signal available in revision D.
/// Waits for PMIC to power on.
/// On the very first power-on the PMIC is shutdown by default.
/// The long button press will power on the PMIC and it will become available on I2C.
pub fn wait_init_pmic(
    mut i2c1: bsp::PmicI2cPort,
    mut scl: bsp::PmicI2cSclPin,
    mut sda: bsp::PmicI2cSdaPin,
    rcc: &mut Rcc,
    delay: &mut impl DelayMs<u8>,
) -> Pmic<Initialized, I2cError, bsp::PmicI2c> {
    loop {
        let i2c = I2c::i2c1(i2c1, (scl, sda), 100.khz(), rcc);

        let mut pmic = PmicBuilder::new(i2c);
        let pmic_check = pmic.check();

        if pmic_check {
            break pmic.build();
        } else {
            delay.delay_ms(100_u8);
        }
        let (i2c, (scl_pin, sda_pin)) = pmic.free().free();
        i2c1 = i2c;
        scl = scl_pin;
        sda = sda_pin;
    }
    .init()
    .unwrap()
}

fn status<E: core::fmt::Debug, I: WriteRead<Error = E> + Write<Error = E>>(axp173: &mut Axp173<I>) {
    // Is the device connected to the USB power supply?
    if axp173.vbus_present().unwrap() {
        defmt::info!("VBUS is present");
    } else {
        defmt::info!("VBUS is not present");
    }

    // Is the battery connected to the device?
    if axp173.battery_present().unwrap() {
        defmt::info!("Battery is present");
    } else {
        defmt::info!("Battery is not present");
    }

    // Is the battery currently being charged?
    if axp173.battery_charging().unwrap() {
        defmt::info!("Battery is charging");
    } else {
        defmt::info!("Battery is not charging");
    }

    let vbus = axp173.vbus_voltage().unwrap();
    defmt::info!("VBUS: {:f32}", vbus.as_volts());

    let vbus = axp173.vbus_current().unwrap();
    defmt::info!("VBUS current: {:f32} mA", vbus.as_milliamps());

    let batt = axp173.batt_voltage().unwrap();
    defmt::info!("Batt: {:f32} V", batt.as_volts());

    let batt_charge = axp173.batt_charge_current().unwrap();
    let batt_discharge = axp173.batt_discharge_current().unwrap();
    defmt::info!(
        "Batt: ^ {:f32} mA | v {:f32} mA",
        batt_charge.as_milliamps(),
        batt_discharge.as_milliamps()
    );

    defmt::info!(
        "Charge coulombs: {:u32}",
        axp173.read_charge_coulomb_counter().unwrap()
    );
    defmt::info!(
        "Discharge coulombs: {:u32}",
        axp173.read_discharge_coulomb_counter().unwrap()
    );

    let ldo = axp173.read_ldo(LdoKind::LDO2).unwrap();
    defmt::info!(
        "LDO2: enabled: {:bool}, voltage: {:f32} V",
        ldo.enabled(),
        ldo.voltage().0 as f32 / 1000.0,
    );

    let ldo = axp173.read_ldo(LdoKind::LDO3).unwrap();
    defmt::info!(
        "LDO3: enabled: {:bool}, voltage: {:f32} V",
        ldo.enabled(),
        ldo.voltage().0 as f32 / 1000.0,
    );

    let ldo = axp173.read_ldo(LdoKind::LDO4).unwrap();
    defmt::info!(
        "LDO4: enabled: {:bool}, voltage: {:f32} V",
        ldo.enabled(),
        ldo.voltage().0 as f32 / 1000.0,
    );
}
