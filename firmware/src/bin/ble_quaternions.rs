//! Enables PMIC (AXP173), turns IMU (BNO080) on and starts to stream
//! current board's position (rotation quaternions) via Bluetooth Low Energy (BLE).

#![no_std]
#![no_main]
#![feature(trait_alias)]
#![feature(type_alias_impl_trait)]

use tracksb as _;

use core::cell::UnsafeCell;
use cortex_m_rt::{entry, exception, ExceptionFrame};
use embassy::{
    executor::{task, Executor},
    util::{Forever, Signal},
};
use embassy_stm32wb55::{ble::Ble, interrupt};
use embedded_hal::blocking::delay::DelayMs;
use stm32wb_hal::{
    delay::DelayCM,
    flash::FlashExt,
    gpio::{ExtiPin, State},
    i2c::{Error as I2cError, I2c},
    ipcc::Ipcc,
    pac,
    prelude::*,
    pwr,
    rcc::{
        ApbDivider, Config, HDivider, HseDivider, PllConfig, PllSrc, Rcc, RfWakeupClock, RtcClkSrc,
        StopWakeupClock, SysClkSrc,
    },
    rtc, stm32,
    tl_mbox::{shci::ShciBleInitCmdParam, TlMbox},
};
use tracksb::{
    ble::{batt_service::BatteryService, service::QuaternionsService},
    bsp,
    bsp::{ImuIntPin, PmicIntPin, Rgb},
    imu::{ImuWrapper, Quaternion, IMU_REPORTING_INTERVAL_MS, IMU_REPORTING_RATE_HZ},
    pmic,
    pmic::{wait_init_pmic, ImuPowerState, Pmic},
    rgbled::{startup_animate, LedColor, RgbLed},
};

struct SyncWrapper<T>(pub UnsafeCell<Option<T>>);
impl<T> SyncWrapper<T> {
    pub const fn new_none() -> Self {
        Self(UnsafeCell::new(None))
    }
}

unsafe impl<T> Sync for SyncWrapper<T> {}

static RGB: SyncWrapper<Rgb> = SyncWrapper::new_none();
static PMIC: SyncWrapper<Pmic<pmic::Initialized, I2cError, bsp::PmicI2c>> = SyncWrapper::new_none();
static PMIC_INT_PIN: SyncWrapper<PmicIntPin> = SyncWrapper::new_none();
static IMU: SyncWrapper<ImuWrapper<I2cError, bsp::ImuI2c, bsp::ImuResetPin>> =
    SyncWrapper::new_none();
static IMU_INT_PIN: SyncWrapper<ImuIntPin> = SyncWrapper::new_none();
static DELAY: SyncWrapper<DelayCM> = SyncWrapper::new_none();
static QUAT_SIGNAL: Signal<Quaternion> = Signal::new();

static BLE: SyncWrapper<Ble> = SyncWrapper::new_none();
static SERVICE: SyncWrapper<QuaternionsService> = SyncWrapper::new_none();
static BATT_SERVICE: SyncWrapper<BatteryService> = SyncWrapper::new_none();

static BLE_READY_SIGNAL: Signal<()> = Signal::new();

#[task]
async fn run_main(mbox: TlMbox, ipcc: Ipcc) {
    let config = ShciBleInitCmdParam {
        p_ble_buffer_address: 0,
        ble_buffer_size: 0,
        num_attr_record: 68,
        num_attr_serv: 8,
        attr_value_arr_size: 1344,
        num_of_links: 8,
        extended_packet_length_enable: 1,
        pr_write_list_size: 0x3A,
        mb_lock_count: 0x79,
        att_mtu: 156,
        slave_sca: 500,
        master_sca: 0,
        ls_source: 1,
        max_conn_event_length: 0xFFFFFFFF,
        hs_startup_time: 0x148,
        viterbi_enable: 1,
        ll_only: 0,
        hw_version: 0,
    };

    defmt::info!("Initializing BLE");

    let mut ble = Ble::init(
        interrupt::take!(IPCC_C1_RX_IT),
        interrupt::take!(IPCC_C1_TX_IT),
        config,
        mbox,
        ipcc,
    )
    .await
    .unwrap();

    defmt::info!("BLE Initialized");

    tracksb::ble::init_gap_and_gatt(&mut ble).await.unwrap();
    let service = tracksb::ble::service::QuaternionsService::new(&mut ble)
        .await
        .unwrap();
    let batt_service = tracksb::ble::batt_service::BatteryService::new(&mut ble)
        .await
        .unwrap();

    tracksb::ble::set_advertisement(true, &mut ble)
        .await
        .unwrap();

    defmt::info!("BLE Services Ready");

    unsafe { &mut *BLE.0.get() }.replace(ble);
    unsafe { &mut *SERVICE.0.get() }.replace(service);
    unsafe { &mut *BATT_SERVICE.0.get() }.replace(batt_service);

    BLE_READY_SIGNAL.signal(());

    loop {
        let ble = unsafe { &mut *BLE.0.get() };
        if let Some(ble) = ble {
            let quat = QUAT_SIGNAL.wait().await;
            if ble.has_events() {
                tracksb::ble::process_event(ble).await.unwrap();
            }
            let service = unsafe { &mut *SERVICE.0.get() };
            if let Some(service) = service {
                /*defmt::info!(
                    "Updating quaternion: {:?}",
                    defmt::Debug2Format::<defmt::consts::U128>(&quat)
                );*/
                service.update(ble, &quat).await.unwrap();
            }

            let batt_service = unsafe { &mut *BATT_SERVICE.0.get() };
            let pmic = unsafe { &mut *PMIC.0.get() };
            if let Some(pmic) = pmic {
                if let Some(batt_service) = batt_service {
                    let level = pmic.battery_level().unwrap();
                    batt_service.update(ble, level as u8).await.unwrap();
                }
            }
        }
    }
}

static EXECUTOR: Forever<Executor> = Forever::new();

#[entry]
fn main() -> ! {
    defmt::info!("Starting");

    let mut dp = stm32::Peripherals::take().unwrap();
    let _cp = cortex_m::peripheral::Peripherals::take().unwrap();

    // Allow using debugger and RTT during WFI/WFE (sleep)
    dp.DBGMCU.cr.modify(|_, w| {
        w.dbg_sleep().set_bit();
        w.dbg_standby().set_bit();
        w.dbg_stop().set_bit()
    });
    dp.RCC.ahb1enr.modify(|_, w| w.dma1en().set_bit());

    let mut rcc = dp.RCC.constrain();
    rcc.set_stop_wakeup_clock(StopWakeupClock::HSI16);

    // Fastest clock configuration.
    // * External low-speed crystal is used (LSE)
    // * 32 MHz HSE with PLL
    // * 64 MHz CPU1, 32 MHz CPU2
    // * 64 MHz for APB1, APB2
    // * HSI as a clock source after wake-up from low-power mode
    let clock_config = Config::new(SysClkSrc::Pll(PllSrc::Hse(HseDivider::NotDivided)))
        .with_lse()
        .cpu1_hdiv(HDivider::NotDivided)
        .cpu2_hdiv(HDivider::Div2)
        .apb1_div(ApbDivider::NotDivided)
        .apb2_div(ApbDivider::NotDivided)
        .pll_cfg(PllConfig {
            m: 2,
            n: 12,
            r: 3,
            q: Some(4),
            p: Some(3),
        })
        .rtc_src(RtcClkSrc::Lse)
        .rf_wkp_sel(RfWakeupClock::Lse);

    let mut rcc = rcc.apply_clock_config(clock_config, &mut dp.FLASH.constrain().acr);

    // RTC is required for proper operation of BLE stack
    let _rtc = rtc::Rtc::rtc(dp.RTC, &mut rcc);

    let mut ipcc = dp.IPCC.constrain();
    let mbox = TlMbox::tl_init(&mut rcc, &mut ipcc);
    pwr::set_cpu2(false);

    let mut gpioa = dp.GPIOA.split(&mut rcc);
    let mut gpiob = dp.GPIOB.split(&mut rcc);
    let mut delay = DelayCM::new(rcc.clocks);

    let red_led = gpioa.pa4.into_push_pull_output_with_state(
        &mut gpioa.moder,
        &mut gpioa.otyper,
        State::High,
    );
    let green_led = gpioa.pa5.into_push_pull_output_with_state(
        &mut gpioa.moder,
        &mut gpioa.otyper,
        State::High,
    );
    let blue_led = gpioa.pa6.into_push_pull_output_with_state(
        &mut gpioa.moder,
        &mut gpioa.otyper,
        State::High,
    );
    let mut rgb_led = RgbLed::new(red_led, green_led, blue_led);

    /* PMIC */

    // I2C pull-ups are controlled via pin
    let mut pull_ups = gpiob
        .pb5
        .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
    let _ = pull_ups.set_high();
    let scl = gpiob
        .pb6
        .into_floating_input(&mut gpiob.moder, &mut gpiob.pupdr);
    let sda = gpiob
        .pb7
        .into_floating_input(&mut gpiob.moder, &mut gpiob.pupdr);
    let scl = scl.into_open_drain_output(&mut gpiob.moder, &mut gpiob.otyper);
    let scl = scl.into_af4(&mut gpiob.moder, &mut gpiob.afrl);
    let sda = sda.into_open_drain_output(&mut gpiob.moder, &mut gpiob.otyper);
    let sda = sda.into_af4(&mut gpiob.moder, &mut gpiob.afrl);
    let mut pmic = wait_init_pmic(dp.I2C1, scl, sda, &mut rcc, &mut delay);
    pmic.set_imu_power(true).unwrap();
    let pmic_int_pin = bsp::init_pmic_interrupt(
        gpiob
            .pb1
            .into_pull_up_input(&mut gpiob.moder, &mut gpiob.pupdr),
        &mut dp.SYSCFG,
        &mut dp.EXTI,
    );

    unsafe { &mut *PMIC_INT_PIN.0.get() }.replace(pmic_int_pin);

    startup_animate(&mut rgb_led, &mut delay);

    unsafe { &mut *PMIC.0.get() }.replace(pmic);
    unsafe { &mut *RGB.0.get() }.replace(rgb_led);

    /* IMU */

    let mut imu_rst = gpioa
        .pa15
        .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
    imu_rst.set_low().unwrap();
    delay.delay_ms(100_u16);
    imu_rst.set_high().unwrap();

    let imu_int_pin = bsp::init_imu_interrupt(
        gpiob
            .pb3
            .into_pull_up_input(&mut gpiob.moder, &mut gpiob.pupdr),
        &mut dp.SYSCFG,
        &mut dp.EXTI,
    );
    unsafe { &mut *IMU_INT_PIN.0.get() }.replace(imu_int_pin);

    let scl = gpioa
        .pa7
        .into_floating_input(&mut gpioa.moder, &mut gpioa.pupdr);
    let scl = scl.into_open_drain_output(&mut gpioa.moder, &mut gpioa.otyper);
    let scl = scl.into_af4(&mut gpioa.moder, &mut gpioa.afrl);
    let sda = gpiob
        .pb4
        .into_open_drain_output(&mut gpiob.moder, &mut gpiob.otyper);
    let sda = sda.into_af4(&mut gpiob.moder, &mut gpiob.afrl);
    let i2c3 = I2c::i2c3(dp.I2C3, (scl, sda), 100.khz(), &mut rcc);

    let imu = ImuWrapper::new(i2c3, imu_rst);
    unsafe { &mut *IMU.0.get() }.replace(imu);
    unsafe { &mut *DELAY.0.get() }.replace(delay);

    defmt::info!(
        "Initialized PMIC, MCU at {:u32} MHz and IMU at {:u16} Hz",
        rcc.clocks.sysclk().0 / 1000,
        IMU_REPORTING_RATE_HZ
    );

    let executor = EXECUTOR.put(Executor::new(cortex_m::asm::sev));
    executor.spawn(run_main(mbox, ipcc)).unwrap();

    loop {
        executor.run();
        cortex_m::asm::wfe();
    }
}

fn poll_imu(imu: &mut ImuWrapper<I2cError, bsp::ImuI2c, bsp::ImuResetPin>) {
    let delay = unsafe { &mut *DELAY.0.get() };
    let rgb_led = unsafe { &mut *RGB.0.get() };

    if let Some(delay) = delay {
        if let Some(quat) = imu.quaternion(delay).unwrap() {
            if let Some(rgb_led) = rgb_led {
                rgb_led.toggle(LedColor::Green);
                QUAT_SIGNAL.signal(quat);
            }
        }
    }
}

#[interrupt]
#[allow(non_snake_case)]
fn EXTI3() {
    let int_pin = unsafe { &mut *IMU_INT_PIN.0.get() };
    if let Some(int_pin) = int_pin {
        let pmic = unsafe { &mut *PMIC.0.get() };
        if let Some(pmic) = pmic {
            let imu = unsafe { &mut *IMU.0.get() };
            if let Some(imu) = imu {
                let delay = unsafe { &mut *DELAY.0.get() };
                if let Some(delay) = delay {
                    if int_pin.check_interrupt() {
                        int_pin.clear_interrupt_pending_bit();

                        // Ignore interrupts if IMU isn't enabled
                        if !pmic.imu_enabled().unwrap() {
                            return;
                        }

                        // Initialize the IMU if it just booted up
                        if !imu.is_initialized() {
                            defmt::info!("BNO08x booted, initializing...");
                            imu.init_imu(delay, IMU_REPORTING_INTERVAL_MS);
                        } else {
                            poll_imu(imu);
                        }
                    }
                }
            }
        }
    }
}

fn imu_on_off(
    imu: &mut ImuWrapper<I2cError, bsp::ImuI2c, bsp::ImuResetPin>,
    delay: &mut impl DelayMs<u8>,
    turn_on: bool,
) {
    if turn_on {
        if !imu.is_initialized() {
            imu.reset_imu(delay);
        }
    } else {
        imu.deinit();
        let rgb_led = unsafe { &mut *RGB.0.get() };
        if let Some(rgb_led) = rgb_led {
            rgb_led.turn_off_all();
        }
        // TODO: put the MCU into deep sleep
    }
}

#[interrupt]
#[allow(non_snake_case)]
fn EXTI1() {
    let int_pin = unsafe { &mut *PMIC_INT_PIN.0.get() };
    if let Some(int_pin) = int_pin {
        let pmic = unsafe { &mut *PMIC.0.get() };
        if let Some(pmic) = pmic {
            let imu = unsafe { &mut *IMU.0.get() };
            if let Some(imu) = imu {
                let delay = unsafe { &mut *DELAY.0.get() };
                if let Some(delay) = delay {
                    if int_pin.check_interrupt() {
                        int_pin.clear_interrupt_pending_bit();

                        // Process IRQs and manage IMU power if it was a power button IRQ
                        let imu_power_state = pmic.process_irqs().unwrap();
                        match imu_power_state {
                            ImuPowerState::Shutdown => imu_on_off(imu, delay, false),
                            ImuPowerState::Enabled => imu_on_off(imu, delay, true),
                            ImuPowerState::Unchanged => (),
                        }

                        pmic.show_current().unwrap();
                    }
                }
            }
        }
    }
}

#[exception]
#[allow(non_snake_case)]
unsafe fn HardFault(_ef: &ExceptionFrame) -> ! {
    // Turn the red led on and turn the others off
    let mut rcc = Rcc {
        clocks: Default::default(),
        config: Default::default(),
        rb: pac::Peripherals::steal().RCC,
    };
    let mut gpioa = pac::Peripherals::steal().GPIOA.split(&mut rcc);
    let mut red_led = gpioa
        .pa4
        .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
    let mut green_led = gpioa
        .pa5
        .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
    let mut blue_led = gpioa
        .pa6
        .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);

    red_led.set_low().unwrap();
    green_led.set_high().unwrap();
    blue_led.set_high().unwrap();

    cortex_m::asm::udf();
}
