//! Power Management IC (AXP173) routines

use axp173::{
    AdcSampleRate, AdcSettings, Axp173, ChargingCurrent, Ldo, LdoKind, ShutdownLongPressTime,
    TsPinMode,
};
use embedded_hal::blocking::i2c::{WriteRead, Write};

pub struct Pmic<E: core::fmt::Debug, I: WriteRead<Error = E> + Write<Error = E>> {
    axp173: Axp173<I>,
}

impl<E: core::fmt::Debug, I: WriteRead<Error = E> + Write<Error = E>> Pmic<E, I> {
    pub fn new(i2c: I) -> Result<Self, axp173::Error<E>> {
        let axp173 = Axp173::new(i2c);

        Ok(Self {
            axp173,
        })
    }

    pub fn init(&mut self) -> Result<(), axp173::Error<E>> {
        // Check the presence of AXP173
        self.axp173.init()?;

        // Set charging current to 100mA
        self.axp173
            .set_charging_current(ChargingCurrent::CURRENT_100MA)?;

        // 25Hz sample rate, Disable TS, enable current sensing ADC
        self.axp173
            .set_adc_settings(
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
        self.axp173.set_shutdown_long_press(true)?;

        self.axp173.disable_ldo(&LdoKind::LDO4)?;

        self.set_imu_power(true)?;

        status(&mut self.axp173);

        Ok(())
    }

    pub fn set_imu_power(&mut self, enable: bool) -> Result<(), axp173::Error<E>> {
        if enable {
            // Provide 2.8V for IMU via LDO3
            self.axp173.enable_ldo(&Ldo::ldo3_with_voltage(10, true))?;
        } else {
            self.axp173.disable_ldo(&LdoKind::LDO3)?;
        }

        Ok(())
    }
}

fn status<E: core::fmt::Debug, I: WriteRead<Error = E> + Write<Error = E>>(
    axp173: &mut Axp173<I>
) {
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
