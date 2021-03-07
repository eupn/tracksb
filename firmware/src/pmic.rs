//! Power Management IC (AXP173) routines

use crate::{bsp, bsp::PMIC_I2C_SPEED};
use async_embedded_traits::{
    delay::AsyncDelayMs,
    i2c::{AsyncI2cTransfer, AsyncI2cWrite, I2cAddress7Bit},
};
use axp173::{
    AdcSampleRate, AdcSettings, Axp173, ChargingCurrent, Irq, Ldo, LdoKind, ShutdownLongPressTime,
    TsPinMode,
};
use core::marker::PhantomData;
use cortex_m::singleton;
use embassy_stm32wb55::{i2c::i2c1::AsyncI2c as AsyncI2c1, interrupt};
use embedded_hal::blocking::i2c::Read;
use stm32wb_hal::{
    dma::dma1impl::C1,
    i2c::{Error as I2cError, I2c},
    rcc::Rcc,
};

pub trait AsyncI2c<E: core::fmt::Debug> =
    AsyncI2cTransfer<I2cAddress7Bit, Error = E> + AsyncI2cWrite<I2cAddress7Bit, Error = E>;

pub trait PmicState {}
pub struct Created;
pub struct Initialized {
    charge_coulombs_default: u32,
    is_imu_enabled: bool,
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

pub struct Pmic<S: PmicState, E: core::fmt::Debug, I: AsyncI2c<E>> {
    axp173: Axp173<I>,
    state: S,
    _e: PhantomData<E>,
}

pub struct PmicBuilder<E: core::fmt::Debug, I: AsyncI2c<E>> {
    _e: PhantomData<E>,
    i2c: I,
}

impl<I: AsyncI2c<I2cError>> PmicBuilder<I2cError, I> {
    pub fn new(i2c: I) -> Self {
        Self {
            _e: PhantomData,
            i2c,
        }
    }

    pub fn free(self) -> I {
        self.i2c
    }

    pub fn build(self) -> Pmic<Created, I2cError, I> {
        let axp173 = Axp173::new(self.i2c);

        Pmic {
            axp173,
            _e: PhantomData,
            state: Created,
        }
    }
}

impl<E: core::fmt::Debug, I: AsyncI2c<E>> Pmic<Created, E, I> {
    pub async fn init(mut self) -> Result<Pmic<Initialized, E, I>, axp173::Error<E>> {
        // Check the presence of AXP173
        self.axp173.init().await?;

        // Set charging current to 100mA
        self.axp173
            .set_charging_current(ChargingCurrent::CURRENT_100MA)
            .await?;

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
            )
            .await?;

        self.axp173.set_coulomb_counter(true).await?;
        self.axp173.resume_coulomb_counter().await?;

        self.axp173
            .set_shutdown_long_press_time(ShutdownLongPressTime::SEC_4)
            .await?;
        self.axp173.set_shutdown_long_press(false).await?;

        self.axp173.disable_ldo(&LdoKind::LDO4).await?;

        self.axp173.clear_all_irq().await?;
        self.enable_irqs().await?;

        status(&mut self.axp173).await;

        Ok(Pmic {
            axp173: self.axp173,
            state: Initialized {
                charge_coulombs_default: axp173::mah_to_coulombs_adc(
                    DEFAULT_BATTERY_CAPACITY_MAH,
                    25.0,
                ) as u32,
                is_imu_enabled: false,
            },
            _e: PhantomData,
        })
    }

    /// Enables interesting IRQs from PMIC.
    async fn enable_irqs(&mut self) -> Result<(), axp173::Error<E>> {
        self.axp173
            .set_irq(axp173::Irq::ButtonLongPress, true)
            .await?;
        self.axp173
            .set_irq(axp173::Irq::BatteryCharged, true)
            .await?;
        self.axp173
            .set_irq(axp173::Irq::LowBatteryWarning, true)
            .await?;
        self.axp173
            .set_irq(axp173::Irq::VbusPluggedIn, true)
            .await?;
        self.axp173
            .set_irq(axp173::Irq::VbusUnplugged, true)
            .await?;

        Ok(())
    }
}

impl<E: core::fmt::Debug, I: AsyncI2c<E>> Pmic<Initialized, E, I> {
    pub async fn set_imu_power(&mut self, enable: bool) -> Result<(), axp173::Error<E>> {
        defmt::info!("Setting IMU enabled: {}", enable);
        if enable {
            // Provide 2.8V for IMU via LDO3
            self.axp173
                .enable_ldo(&Ldo::ldo3_with_voltage(10, true))
                .await?;
        } else {
            self.axp173.disable_ldo(&LdoKind::LDO3).await?;
        }

        self.state.is_imu_enabled = self.axp173.read_ldo(LdoKind::LDO3).await?.enabled();

        Ok(())
    }

    pub fn imu_enabled(&mut self) -> Result<bool, axp173::Error<E>> {
        Ok(self.state.is_imu_enabled)
    }

    /// Checks IRQ flags and clears pending IRQs.
    pub async fn process_irqs(&mut self) -> Result<ImuPowerState, axp173::Error<E>> {
        defmt::info!("PMIC IRQ START");
        let mut imu_power_state = ImuPowerState::Unchanged;

        if self.axp173.check_irq(Irq::ButtonLongPress).await? {
            self.axp173.clear_irq(Irq::ButtonLongPress).await?;

            if self.imu_enabled()? {
                imu_power_state = ImuPowerState::Shutdown;
            } else {
                imu_power_state = ImuPowerState::Enabled;
            }
        }
        if self.axp173.check_irq(Irq::BatteryCharged).await? {
            self.axp173.clear_irq(Irq::BatteryCharged).await?;
            self.axp173.reset_coulomb_counter().await?;
        }
        if self.axp173.check_irq(Irq::LowBatteryWarning).await? {
            self.axp173.clear_irq(Irq::LowBatteryWarning).await?;
            let discharge_coulombs = self.axp173.read_discharge_coulomb_counter().await?;
            defmt::info!("Low battery, discharge cc: {}", discharge_coulombs);
        }
        if self.axp173.check_irq(Irq::VbusPluggedIn).await? {
            self.axp173.clear_irq(Irq::VbusPluggedIn).await?;
            defmt::info!("USB charger plugged in");
        }
        if self.axp173.check_irq(Irq::VbusUnplugged).await? {
            self.axp173.clear_irq(Irq::VbusUnplugged).await?;
            defmt::info!("USB charger unplugged");
        }

        // Clear other possible IRQs otherwise we can't rely on the IRQ pin
        self.axp173.clear_all_irq().await?;

        defmt::info!("PMIC IRQ FINISH");

        Ok(imu_power_state)
    }

    /// Shows battery discharge current for reference purposes.
    pub async fn show_current(&mut self) -> Result<(), axp173::Error<E>> {
        let batt_discharge = self.axp173.batt_discharge_current().await?;
        defmt::info!(
            "Batt current consumption: {} mA",
            batt_discharge.as_milliamps()
        );

        Ok(())
    }

    pub async fn battery_level(&mut self) -> Result<u8, axp173::Error<E>> {
        let _coulombs_out = self.axp173.read_discharge_coulomb_counter().await?;
        let mut coulombs_in = self.axp173.read_charge_coulomb_counter().await?;

        // Assume full battery if there's not enough charge coulombs
        if coulombs_in < self.state.charge_coulombs_default {
            coulombs_in = self.state.charge_coulombs_default + 1;
        }

        let remaining_charge = self.axp173.estimate_charge_level(Some(coulombs_in)).await?;
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
pub async fn wait_init_pmic(
    mut i2c1: bsp::PmicI2cPort,
    dma_c1: C1,
    mut scl: bsp::PmicI2cSclPin,
    mut sda: bsp::PmicI2cSdaPin,
    rcc: &mut Rcc,
    delay: &mut impl AsyncDelayMs<u8>,
) -> Pmic<Initialized, I2cError, bsp::PmicI2c> {
    defmt::info!("Waiting for PMIC");

    // TODO: use async check
    loop {
        let mut i2c = I2c::i2c1(i2c1, (scl, sda), PMIC_I2C_SPEED, rcc);
        let mut buf = [0; 1];
        let pmic_check = i2c.read(0x34, &mut buf).is_ok();
        if pmic_check {
            let async_i2c = AsyncI2c1::new(
                singleton!(: [u8; 128] = [0; 128]).unwrap(),
                i2c,
                interrupt::take!(I2C1_EV),
                interrupt::take!(I2C1_ER),
                interrupt::take!(DMA1_CHANNEL1),
                dma_c1,
            );
            break PmicBuilder::new(async_i2c).build();
        } else {
            delay.async_delay_ms(100_u8).await;
        }
        let (i2c, (scl_pin, sda_pin)) = i2c.free();
        i2c1 = i2c;
        scl = scl_pin;
        sda = sda_pin;
    }
    .init()
    .await
    .unwrap()
}

async fn status<E: core::fmt::Debug, I: AsyncI2c<E>>(axp173: &mut Axp173<I>) {
    // Is the device connected to the USB power supply?
    if axp173.vbus_present().await.unwrap() {
        defmt::info!("VBUS is present");
    } else {
        defmt::info!("VBUS is not present");
    }

    // Is the battery connected to the device?
    if axp173.battery_present().await.unwrap() {
        defmt::info!("Battery is present");
    } else {
        defmt::info!("Battery is not present");
    }

    // Is the battery currently being charged?
    if axp173.battery_charging().await.unwrap() {
        defmt::info!("Battery is charging");
    } else {
        defmt::info!("Battery is not charging");
    }

    let vbus = axp173.vbus_voltage().await.unwrap();
    defmt::info!("VBUS: {}", vbus.as_volts());

    let vbus = axp173.vbus_current().await.unwrap();
    defmt::info!("VBUS current: {} mA", vbus.as_milliamps());

    let batt = axp173.batt_voltage().await.unwrap();
    defmt::info!("Batt: {} V", batt.as_volts());

    let batt_charge = axp173.batt_charge_current().await.unwrap();
    let batt_discharge = axp173.batt_discharge_current().await.unwrap();
    defmt::info!(
        "Batt: ^ {} mA | v {} mA",
        batt_charge.as_milliamps(),
        batt_discharge.as_milliamps()
    );

    defmt::info!(
        "Charge coulombs: {}",
        axp173.read_charge_coulomb_counter().await.unwrap()
    );
    defmt::info!(
        "Discharge coulombs: {}",
        axp173.read_discharge_coulomb_counter().await.unwrap()
    );

    let ldo = axp173.read_ldo(LdoKind::LDO2).await.unwrap();
    defmt::info!(
        "LDO2: enabled: {}, voltage: {} V",
        ldo.enabled(),
        ldo.voltage().0 as f32 / 1000.0,
    );

    let ldo = axp173.read_ldo(LdoKind::LDO3).await.unwrap();
    defmt::info!(
        "LDO3: enabled: {}, voltage: {} V",
        ldo.enabled(),
        ldo.voltage().0 as f32 / 1000.0,
    );

    let ldo = axp173.read_ldo(LdoKind::LDO4).await.unwrap();
    defmt::info!(
        "LDO4: enabled: {}, voltage: {} V",
        ldo.enabled(),
        ldo.voltage().0 as f32 / 1000.0,
    );
}
