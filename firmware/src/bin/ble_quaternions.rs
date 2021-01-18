//! BLE Eddystone URL beacon example.
#![no_std]
#![no_main]
#![feature(trait_alias)]
#![feature(type_alias_impl_trait)]

use tracksb as _;

use cortex_m_rt::entry;
use embassy::{
    executor::{task, Executor},
    util::Forever,
};
use embassy_stm32wb55::{
    ble::Ble,
    interrupt,
};
use stm32wb_hal::{
    flash::FlashExt,
    prelude::*,
    pwr,
    rcc::{
        ApbDivider, Config, HDivider, HseDivider, PllConfig, PllSrc, RfWakeupClock, RtcClkSrc,
        StopWakeupClock, SysClkSrc,
    },
    rtc, stm32,
    tl_mbox::{shci::ShciBleInitCmdParam, TlMbox},
};
use tracksb::imu::Quaternion;

#[task]
async fn run(dp: stm32::Peripherals, _cp: cortex_m::Peripherals) {
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

    tracksb::ble::service::init_gap_and_gatt(&mut ble)
        .await
        .unwrap();
    let mut service = tracksb::ble::service::QuaternionsService::new(&mut ble)
        .await
        .unwrap();

    defmt::info!("Service Ready");

    let mut i = 0;
    loop {
        tracksb::ble::service::process_event(&mut ble)
            .await
            .unwrap();
        let quat: Quaternion = [i as f32, i as f32, i as f32, i as f32];
        service.update(&mut ble, &quat).await.unwrap();

        i += 1;
    }
}

static EXECUTOR: Forever<Executor> = Forever::new();

#[entry]
fn main() -> ! {
    defmt::info!("Starting");

    let dp = stm32::Peripherals::take().unwrap();
    let cp = cortex_m::peripheral::Peripherals::take().unwrap();

    let executor = EXECUTOR.put(Executor::new(cortex_m::asm::sev));
    executor.spawn(run(dp, cp)).unwrap();

    loop {
        executor.run();
        cortex_m::asm::wfe();
    }
}
