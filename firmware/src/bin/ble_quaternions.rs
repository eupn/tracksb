//! Enables PMIC (AXP173), turns IMU (BNO080) on and starts to stream
//! current board's position (rotation quaternions) via Bluetooth Low Energy (BLE).

#![no_std]
#![no_main]
#![feature(trait_alias)]
#![feature(type_alias_impl_trait)]

use tracksb as _;

use async_embedded_traits::delay::AsyncDelayMs;
use core::cell::UnsafeCell;
use cortex_m_rt::{entry, exception, ExceptionFrame};
use embassy::{
    executor::{task, Executor, Spawner},
    util::{Forever, InterruptFuture, Signal},
};
use embassy_stm32wb55::{
    ble::Ble,
    delay::{lptim1::LptimDelay as Lptim1Delay, lptim2::LptimDelay as Lptim2Delay},
    i2c::i2c3::AsyncI2c as AsyncI2c3,
};
use futures::{future::Either, pin_mut};
use futures_intrusive::sync::LocalMutex;
use heapless::{consts, spsc::Queue};
use if_chain::if_chain;
use stm32wb_hal::{
    dma::{
        dma1impl::{C1, C2},
        DmaExt,
    },
    flash::FlashExt,
    gpio::ExtiPin,
    i2c::{Error as I2cError, I2c},
    interrupt,
    ipcc::Ipcc,
    lptim::{lptim1::LpTimer as LpTimer1, lptim2::LpTimer as LpTimer2},
    pac,
    pac::{Peripherals, EXTI, I2C1, I2C3, SYSCFG},
    prelude::*,
    pwr,
    rcc::{
        ApbDivider, Config, HDivider, HseDivider, LptimClkSrc, PllConfig, PllSrc, Rcc,
        RfWakeupClock, RtcClkSrc, StopWakeupClock, SysClkSrc,
    },
    rtc, smps, stm32,
    tl_mbox::{shci::ShciBleInitCmdParam, TlMbox},
};
use tracksb::{
    ble::{batt_service::BatteryService, motion_service::MotionService},
    bsp,
    bsp::{
        ImuIntPin, ImuResetPin, ImuSclPin, ImuSdaPin, PmicI2cSclPin, PmicI2cSdaPin, PmicIntPin,
        PullUpsPin, IMU_I2C_SPEED,
    },
    imu::{ImuWrapper, MotionData, IMU_REPORTING_INTERVAL_MS, IMU_REPORTING_RATE_HZ},
    led::StatusLed,
    pmic,
    pmic::{wait_init_pmic, ImuPowerState, Pmic},
};

struct SyncWrapper<T>(pub UnsafeCell<Option<T>>);
impl<T> SyncWrapper<T> {
    pub const fn new_none() -> Self {
        Self(UnsafeCell::new(None))
    }
}

unsafe impl<T> Sync for SyncWrapper<T> {}

static STATUS_LED: SyncWrapper<StatusLed> = SyncWrapper::new_none();
static mut PMIC: Option<LocalMutex<Pmic<pmic::Initialized, I2cError, bsp::PmicI2c>>> = None;
static PMIC_INT_PIN: SyncWrapper<PmicIntPin> = SyncWrapper::new_none();
static IMU: SyncWrapper<ImuWrapper<I2cError, bsp::ImuI2c, bsp::ImuResetPin>> =
    SyncWrapper::new_none();
static IMU_INT_PIN: SyncWrapper<ImuIntPin> = SyncWrapper::new_none();

static PMIC_DELAY: SyncWrapper<Lptim1Delay> = SyncWrapper::new_none();
static IMU_DELAY: SyncWrapper<Lptim2Delay> = SyncWrapper::new_none();

static mut BLE: Option<LocalMutex<Ble>> = None;
static SERVICE: SyncWrapper<MotionService> = SyncWrapper::new_none();
static BATT_SERVICE: SyncWrapper<BatteryService> = SyncWrapper::new_none();

static MOTION_SIGNAL: Signal<()> = Signal::new();
static MOTION_DATA_QUEUE: SyncWrapper<
    Queue<MotionData, consts::U32, u8, heapless::spsc::SingleCore>,
> = SyncWrapper::new_none();

static BATTERY_LEVEL_UPDATE_SIGNAL: Signal<()> = Signal::new();
const BATTERY_LEVEL_UPDATE_INTERVAL_MS: u32 = 10_000;

/// Initializes the peripherals (PMIC, IMU, BLE, etc.) and
/// spawns the rest of the subtasks.
#[task]
async fn run_main(spawner: Spawner, mut rcc: Rcc, mbox: TlMbox, ipcc: Ipcc) {
    defmt::info!("Running main");

    let mut dp = unsafe { Peripherals::steal() };
    let mut gpioa = dp.GPIOA.split(&mut rcc);
    let green_led = gpioa
        .pa5
        .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper)
        .into_af1(&mut gpioa.moder, &mut gpioa.afrl);
    let status_led = StatusLed::new(dp.TIM2, green_led, &mut rcc);
    unsafe { &mut *STATUS_LED.0.get() }.replace(status_led);

    let mut gpiob = dp.GPIOB.split(&mut rcc);
    let dma_channels = dp.DMA1.split(&mut rcc, dp.DMAMUX1);
    let pull_ups = gpiob
        .pb5
        .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
    let pmic_scl = gpiob
        .pb6
        .into_floating_input(&mut gpiob.moder, &mut gpiob.pupdr);
    let pmic_sda = gpiob
        .pb7
        .into_floating_input(&mut gpiob.moder, &mut gpiob.pupdr);
    let pmic_scl = pmic_scl.into_open_drain_output(&mut gpiob.moder, &mut gpiob.otyper);
    let pmic_scl = pmic_scl.into_af4(&mut gpiob.moder, &mut gpiob.afrl);
    let pmic_sda = pmic_sda.into_open_drain_output(&mut gpiob.moder, &mut gpiob.otyper);
    let pmic_sda = pmic_sda.into_af4(&mut gpiob.moder, &mut gpiob.afrl);
    let pmic_int_pin = gpiob
        .pb1
        .into_pull_up_input(&mut gpiob.moder, &mut gpiob.pupdr);
    init_pmic(
        dp.I2C1,
        dma_channels.ch1,
        pull_ups,
        pmic_scl,
        pmic_sda,
        pmic_int_pin,
        &mut rcc,
        &mut dp.SYSCFG,
        &mut dp.EXTI,
    )
    .await;

    let imu_rst = gpioa
        .pa15
        .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
    let imu_int_pin = gpiob
        .pb3
        .into_pull_up_input(&mut gpiob.moder, &mut gpiob.pupdr);
    let imu_scl = gpioa
        .pa7
        .into_open_drain_output(&mut gpioa.moder, &mut gpioa.otyper);
    let imu_scl = imu_scl.into_af4(&mut gpioa.moder, &mut gpioa.afrl);
    let imu_sda = gpiob
        .pb4
        .into_open_drain_output(&mut gpiob.moder, &mut gpiob.otyper);
    let imu_sda = imu_sda.into_af4(&mut gpiob.moder, &mut gpiob.afrl);
    init_imu(
        dp.I2C3,
        dma_channels.ch2,
        imu_rst,
        imu_int_pin,
        imu_scl,
        imu_sda,
        &mut rcc,
        &mut dp.SYSCFG,
        &mut dp.EXTI,
    )
    .await;

    defmt::info!(
        "Initialized PMIC, MCU at {} MHz and IMU at {} Hz",
        rcc.clocks.sysclk().0 / 1000,
        IMU_REPORTING_RATE_HZ
    );

    let config = ShciBleInitCmdParam {
        p_ble_buffer_address: 0,
        ble_buffer_size: 0,
        num_attr_record: 68,
        num_attr_serv: 8,
        attr_value_arr_size: 4096,
        num_of_links: 2,
        extended_packet_length_enable: 1,
        pr_write_list_size: 0x3A,
        mb_lock_count: 0x80,
        att_mtu: 1024,
        slave_sca: 100,
        master_sca: 0,
        ls_source: 1,
        max_conn_event_length: 0xFFFFFFFF,
        hs_startup_time: 0x148,
        viterbi_enable: 1,
        ll_only: 0,
        hw_version: 0,
    };

    defmt::info!("Initializing BLE");

    use embassy_stm32wb55::interrupt;
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
    let service = tracksb::ble::motion_service::MotionService::new(&mut ble)
        .await
        .unwrap();
    let batt_service = tracksb::ble::batt_service::BatteryService::new(&mut ble)
        .await
        .unwrap();

    tracksb::ble::set_advertisement(true, &mut ble)
        .await
        .unwrap();

    defmt::info!("BLE Services Ready");

    unsafe { BLE = Some(LocalMutex::new(ble, true)) };
    unsafe { &mut *SERVICE.0.get() }.replace(service);
    unsafe { &mut *BATT_SERVICE.0.get() }.replace(batt_service);

    spawner.spawn(imu_int()).unwrap();
    spawner.spawn(pmic_int()).unwrap();
    spawner.spawn(ble_task()).unwrap();
    spawner.spawn(battery_level()).unwrap();
}

async fn init_pmic(
    i2c1: I2C1,
    dma_c1: C1,
    mut pull_ups: PullUpsPin,
    scl: PmicI2cSclPin,
    sda: PmicI2cSdaPin,
    pmic_int_pin: PmicIntPin,
    mut rcc: &mut Rcc,
    syscfg: &mut SYSCFG,
    exti: &mut EXTI,
) {
    let pmic_delay = unsafe { &mut *PMIC_DELAY.0.get() };
    if let Some(pmic_delay) = pmic_delay {
        // I2C pull-ups are controlled via pin
        let _ = pull_ups.set_high();
        let mut pmic = wait_init_pmic(i2c1, dma_c1, scl, sda, &mut rcc, pmic_delay).await;
        pmic.set_imu_power(true).await.unwrap();
        let pmic_int_pin = bsp::init_pmic_interrupt(pmic_int_pin, syscfg, exti);

        unsafe { &mut *PMIC_INT_PIN.0.get() }.replace(pmic_int_pin);
        unsafe {
            PMIC = Some(LocalMutex::new(pmic, true));
        }
    }
}

async fn init_imu(
    i2c3: I2C3,
    dma_c2: C2,
    imu_rst_pin: ImuResetPin,
    imu_int_pin: ImuIntPin,
    imu_scl_pin: ImuSclPin,
    imu_sda_pin: ImuSdaPin,
    mut rcc: &mut Rcc,
    syscfg: &mut SYSCFG,
    exti: &mut EXTI,
) {
    let imu_delay = unsafe { &mut *IMU_DELAY.0.get() };
    if let Some(imu_delay) = imu_delay {
        let i2c3 = I2c::i2c3(i2c3, (imu_scl_pin, imu_sda_pin), IMU_I2C_SPEED, &mut rcc);
        use embassy_stm32wb55::interrupt;
        let async_i2c3 = AsyncI2c3::new(
            cortex_m::singleton!(: [u8; 1024] = [0; 1024]).unwrap(),
            i2c3,
            interrupt::take!(I2C3_EV),
            interrupt::take!(I2C3_ER),
            interrupt::take!(DMA1_CHANNEL2),
            dma_c2,
        );

        let mut imu = ImuWrapper::new(async_i2c3, imu_rst_pin);
        imu.reset_imu(imu_delay).await;
        unsafe { &mut *IMU.0.get() }.replace(imu);

        let imu_int_pin = bsp::init_imu_interrupt(imu_int_pin, syscfg, exti);
        unsafe { &mut *IMU_INT_PIN.0.get() }.replace(imu_int_pin);
    }
}

/// Manages the BLE resource.
#[task]
async fn ble_task() {
    #[derive(defmt::Format)]
    enum UpdateAction {
        BleEvents,
        BatteryLevel,
        MotionData,
    }

    loop {
        if let Some(ble) = unsafe { &mut BLE } {
            let mut ble = ble.lock().await;
            let update_action = {
                let events_fut = tracksb::ble::process_event(&mut *ble);
                pin_mut!(events_fut);
                let bat_sig_fut = BATTERY_LEVEL_UPDATE_SIGNAL.wait();
                pin_mut!(bat_sig_fut);
                let mot_sig_fut = MOTION_SIGNAL.wait();
                pin_mut!(mot_sig_fut);

                let events_fut_w_bat = futures::future::select(events_fut, bat_sig_fut);
                let all_fut = futures::future::select(events_fut_w_bat, mot_sig_fut);
                match all_fut.await {
                    // Got either new BLE event or battery level update
                    Either::Left((events_bat_fut, _)) => match events_bat_fut {
                        Either::Left((e, _)) => {
                            e.unwrap();
                            UpdateAction::BleEvents
                        }
                        Either::Right(_) => UpdateAction::BatteryLevel,
                    },
                    // Got motion data
                    Either::Right(_) => UpdateAction::MotionData,
                }
            };

            match update_action {
                UpdateAction::BleEvents => (),
                UpdateAction::BatteryLevel => update_battery_level(&mut *ble).await,
                UpdateAction::MotionData => update_motion_data(&mut *ble).await,
            }

            core::mem::drop(ble);
        }
    }
}

/// Flushes the motion data queue into the BLE motion service.
async fn update_motion_data(ble: &mut Ble) {
    if_chain! {
        let motion_data_queue = unsafe { &mut *MOTION_DATA_QUEUE.0.get() };
        let service = unsafe { &mut *SERVICE.0.get() };
        if let Some(service) = service;
        if let Some(motion_data_queue) = motion_data_queue;
        then {
            while let Some(motion_data) = motion_data_queue.dequeue() {
                service.update(ble, &motion_data).await.unwrap();
            }
        }
    }
}

/// Reads current battery level from PMIC and updates the BLE service.
async fn update_battery_level(ble: &mut Ble) {
    if_chain! {
        if let Some(pmic) = unsafe { &mut PMIC };
        let pmic = &mut *pmic.lock().await;
        let batt_service = unsafe { &mut *BATT_SERVICE.0.get() };
        if let Some(batt_service) = batt_service;
        then {
            let level = pmic.battery_level().await.unwrap();
            defmt::info!("Updating battery level {}%", level);
            batt_service.update(ble, level as u8).await.unwrap();
        }
    }
}

/// A timer task that signals for battery level update.
#[task]
async fn battery_level() {
    loop {
        if let Some(pmic_delay) = unsafe { &mut *PMIC_DELAY.0.get() } {
            pmic_delay
                .async_delay_ms(BATTERY_LEVEL_UPDATE_INTERVAL_MS)
                .await;
            BATTERY_LEVEL_UPDATE_SIGNAL.signal(());
        }
    }
}

/// IMU interrupt handler task (EXTI3)
#[task]
async fn imu_int() {
    use embassy_stm32wb55::interrupt;
    let mut imu_int = interrupt::take!(EXTI3);

    loop {
        InterruptFuture::new(&mut imu_int).await;

        let int_pin = unsafe { &mut *IMU_INT_PIN.0.get() };
        if let Some(int_pin) = int_pin {
            if int_pin.check_interrupt() {
                int_pin.clear_interrupt_pending_bit();
            }
        }

        if_chain! {
            if let Some(pmic) = unsafe { &mut PMIC };
            let mut pmic = pmic.lock().await;
            let imu = unsafe { &mut *IMU.0.get() };
            let delay = unsafe { &mut *IMU_DELAY.0.get() };
            let status_led = unsafe { &mut *STATUS_LED.0.get() };
            let motion_data_queue = unsafe { &mut *MOTION_DATA_QUEUE.0.get() };
            if let Some(imu) = imu;
            if let Some(delay) = delay;
            if let Some(status_led) = status_led;
            if let Some(motion_data_queue) = motion_data_queue;
            then {
                // Ignore interrupts if IMU isn't enabled
                if !pmic.imu_enabled().unwrap() {
                    defmt::info!("IMU isn't enabled");
                    continue;
                }

                // Initialize the IMU if it just booted up
                if !imu.is_initialized() {
                    defmt::info!("BNO08x booted, initializing...");
                    imu.init_imu(delay, IMU_REPORTING_INTERVAL_MS).await;
                    defmt::info!("BNO08x initialized");
                } else if let Some(motion_data) = imu.motion_data(delay).await.unwrap() {
                    status_led.tick_animation();
                    motion_data_queue.enqueue(motion_data).ok();
                    MOTION_SIGNAL.signal(());
                }
            }
        }
    }
}

/// PMIC interrupt handler task (EXTI1)
#[task]
async fn pmic_int() {
    use embassy_stm32wb55::interrupt;
    let mut pmic_int = interrupt::take!(EXTI1);

    loop {
        InterruptFuture::new(&mut pmic_int).await;

        let int_pin = unsafe { &mut *PMIC_INT_PIN.0.get() };
        if let Some(int_pin) = int_pin {
            if int_pin.check_interrupt() {
                int_pin.clear_interrupt_pending_bit();
            }
        }

        if let Some(pmic) = unsafe { &mut PMIC } {
            let pmic = &mut *pmic.lock().await;

            if_chain! {
                let imu_power_state = pmic.process_irqs().await.unwrap();
                let imu = unsafe { &mut *IMU.0.get() };
                if let Some(imu) = imu;
                let delay = unsafe { &mut *IMU_DELAY.0.get() };
                if let Some(delay) = delay;
                then {
                    // Process IRQs and manage IMU power if it was a power button IRQ
                    match imu_power_state {
                        ImuPowerState::Shutdown => imu_on_off(pmic, imu, delay, false).await,
                        ImuPowerState::Enabled => imu_on_off(pmic, imu, delay, true).await,
                        ImuPowerState::Unchanged => (),
                    }

                    pmic.show_current().await.unwrap();
                }
            }
        }
    }
}

async fn imu_on_off(
    pmic: &mut Pmic<pmic::Initialized, I2cError, bsp::PmicI2c>,
    imu: &mut ImuWrapper<I2cError, bsp::ImuI2c, bsp::ImuResetPin>,
    delay: &mut impl AsyncDelayMs<u8>,
    turn_on: bool,
) {
    if turn_on {
        defmt::info!("Turning on");
        pmic.set_imu_power(true).await.unwrap();
        imu.reset_imu(delay).await;
    } else {
        defmt::info!("Shutting down");
        imu.reset_imu(delay).await;
        pmic.set_imu_power(false).await.unwrap();
        imu.deinit();

        let status_led = unsafe { &mut *STATUS_LED.0.get() };
        if let Some(status_led) = status_led {
            status_led.turn_off();
        }
        // TODO: put the MCU into deep sleep here (STOP1 mode, for example)
    }
}

static EXECUTOR: Forever<Executor> = Forever::new();

#[entry]
fn main() -> ! {
    defmt::info!("Starting");

    let dp = stm32::Peripherals::take().unwrap();
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
        .rf_wkp_sel(RfWakeupClock::Lse)
        .lptim1_src(LptimClkSrc::Lse)
        .lptim2_src(LptimClkSrc::Lse);

    let mut rcc = rcc.apply_clock_config(clock_config, &mut dp.FLASH.constrain().acr);

    smps::Smps::enable();
    while !smps::Smps::is_enabled() {
        cortex_m::asm::nop();
    }

    // RTC is required for proper operation of BLE stack
    let _rtc = rtc::Rtc::rtc(dp.RTC, &mut rcc);

    let mut ipcc = dp.IPCC.constrain();
    let mbox = TlMbox::tl_init(&mut rcc, &mut ipcc);
    pwr::set_cpu2(false);

    /* DELAYS */
    use embassy_stm32wb55::interrupt;
    let lp_timer1 = LpTimer1::init_oneshot(dp.LPTIM1, &mut rcc);
    let lp_timer2 = LpTimer2::init_oneshot(dp.LPTIM2, &mut rcc);
    let pmic_delay = Lptim1Delay::new(lp_timer1, interrupt::take!(LPTIM1));
    let imu_delay = Lptim2Delay::new(lp_timer2, interrupt::take!(LPTIM2));
    unsafe { &mut *IMU_DELAY.0.get() }.replace(imu_delay);
    unsafe { &mut *PMIC_DELAY.0.get() }.replace(pmic_delay);

    unsafe { &mut *MOTION_DATA_QUEUE.0.get() }.replace(unsafe { Queue::u8_sc() });

    let executor = EXECUTOR.put(Executor::new());
    executor.run(|spawner| {
        spawner.spawn(run_main(spawner, rcc, mbox, ipcc)).unwrap();
    });
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

    panic!("HardFault");
}
