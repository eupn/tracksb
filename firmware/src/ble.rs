use crate::ble::batt_service::BatteryService;
use bluetooth_hci::{
    event::command::ReturnParameters,
    host::{uart::Packet, AdvertisingFilterPolicy, OwnAddressType},
    types::AdvertisingType,
    BdAddr, Event, Status,
};
use core::time::Duration;
use embassy_stm32wb55::ble::Ble;
use stm32wb55::{
    event::{AttExchangeMtuResponse, GattAttributeModified, Stm32Wb5xEvent},
    gap::{AdvertisingDataType, Commands as GapCommands, DiscoverableParameters, LocalName, Role},
    gatt::{Commands as GattCommands, UpdateCharacteristicValueParameters},
    hal::{Commands as HalCommands, ConfigData, PowerLevel},
};
use stm32wb_hal::tl_mbox::lhci::LhciC1DeviceInformationCcrp;

/// Advertisement interval in milliseconds.
const ADV_INTERVAL_MS: u64 = 250;

const BLE_DEVICE_LOCAL_NAME: &[u8] = b"Quaternions";
const BLE_GAP_DEVICE_NAME: &[u8] = b"Quaternions";

pub mod batt_service;
pub mod service;

type BleError = embassy_stm32wb55::ble::BleError<
    bluetooth_hci::host::uart::Error<(), stm32wb55::event::Stm32Wb5xError>,
>;

pub fn check_status(status: &Status<stm32wb55::event::Status>) -> bool {
    let success = matches!(status, Status::Success);

    if !success {
        defmt::warn!(
            "BLE non-success: {:?}",
            defmt::Debug2Format::<defmt::consts::U128>(&status)
        );
    }

    success
}

pub fn get_bd_addr() -> BdAddr {
    let mut bytes = [0u8; 6];

    let lhci_info = LhciC1DeviceInformationCcrp::new();
    bytes[0] = (lhci_info.uid64 & 0xff) as u8;
    bytes[1] = ((lhci_info.uid64 >> 8) & 0xff) as u8;
    bytes[2] = ((lhci_info.uid64 >> 16) & 0xff) as u8;
    bytes[3] = lhci_info.device_type_id;
    bytes[4] = (lhci_info.st_company_id & 0xff) as u8;
    bytes[5] = (lhci_info.st_company_id >> 8 & 0xff) as u8;

    BdAddr(bytes)
}

pub fn get_random_addr() -> BdAddr {
    let mut bytes = [0u8; 6];

    let lhci_info = LhciC1DeviceInformationCcrp::new();
    bytes[0] = (lhci_info.uid64 & 0xff) as u8;
    bytes[1] = ((lhci_info.uid64 >> 8) & 0xff) as u8;
    bytes[2] = ((lhci_info.uid64 >> 16) & 0xff) as u8;
    bytes[3] = 0;
    bytes[4] = 0x6E;
    bytes[5] = 0xED;

    BdAddr(bytes)
}

/*
const BLE_CFG_IRK: [u8; 16] = [
    0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0, 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0,
];
const BLE_CFG_ERK: [u8; 16] = [
    0xfe, 0xdc, 0xba, 0x09, 0x87, 0x65, 0x43, 0x21, 0xfe, 0xdc, 0xba, 0x09, 0x87, 0x65, 0x43, 0x21,
];

fn get_irk() -> EncryptionKey {
    EncryptionKey(BLE_CFG_IRK)
}

fn get_erk() -> EncryptionKey {
    EncryptionKey(BLE_CFG_ERK)
}
*/

pub async fn init_gap_and_gatt(ble: &mut Ble) -> Result<(), BleError> {
    ble.perform_command(|rc| {
        rc.write_config_data(&ConfigData::public_address(get_bd_addr()).build())
    })
    .await?;

    ble.perform_command(|rc| {
        rc.write_config_data(&ConfigData::random_address(get_random_addr()).build())
    })
    .await?;

    ble.perform_command(|rc| rc.set_tx_power_level(PowerLevel::ZerodBm))
        .await?;
    //
    ble.perform_command(|rc| rc.init_gatt()).await?;

    let return_params = ble
        .perform_command(|rc| rc.init_gap(Role::PERIPHERAL, false, BLE_GAP_DEVICE_NAME.len() as u8))
        .await?;
    let (service_handle, dev_name_handle, appearance_handle) = if let ReturnParameters::Vendor(
        stm32wb55::event::command::ReturnParameters::GapInit(stm32wb55::event::command::GapInit {
            service_handle,
            dev_name_handle,
            appearance_handle,
            ..
        }),
    ) = return_params
    {
        (service_handle, dev_name_handle, appearance_handle)
    } else {
        return Err(BleError::UnexpectedEvent);
    };

    ble.perform_command(|rc| {
        rc.update_characteristic_value(&UpdateCharacteristicValueParameters {
            service_handle,
            characteristic_handle: dev_name_handle,
            offset: 0,
            value: BLE_GAP_DEVICE_NAME,
        })
        .map_err(|_| nb::Error::Other(()))
    })
    .await?;

    ble.perform_command(|rc| {
        rc.update_characteristic_value(&UpdateCharacteristicValueParameters {
            service_handle,
            characteristic_handle: appearance_handle,
            offset: 0,
            value: &[0],
        })
        .map_err(|_| nb::Error::Other(()))
    })
    .await?;

    Ok(())
}

pub async fn set_advertisement(enabled: bool, ble: &mut Ble) -> Result<(), BleError> {
    if enabled {
        let res = ble
            .perform_command(|rc| {
                let mut services = [0u8; 3];
                services[0] = AdvertisingDataType::Uuid16 as u8;
                services[1..=2].copy_from_slice(&BatteryService::UUID_16.to_le_bytes());

                let params = DiscoverableParameters {
                    advertising_type: AdvertisingType::ConnectableUndirected,
                    advertising_interval: Some((
                        Duration::from_millis(ADV_INTERVAL_MS),
                        Duration::from_millis(ADV_INTERVAL_MS),
                    )),
                    address_type: OwnAddressType::Public,
                    filter_policy: AdvertisingFilterPolicy::AllowConnectionAndScan,
                    local_name: Some(LocalName::Complete(BLE_DEVICE_LOCAL_NAME)),
                    advertising_data: services.as_ref(),
                    conn_interval: (None, None),
                };

                rc.set_discoverable(&params)
                    .map_err(|_| nb::Error::Other(()))
            })
            .await?;
        defmt::info!(
            "Adv: {:?}",
            defmt::Debug2Format::<defmt::consts::U128>(&res)
        );
    } else {
        ble.perform_command(|rc| rc.set_nondiscoverable()).await?;
    }

    Ok(())
}

pub async fn process_event(ble: &mut Ble) -> Result<(), BleError> {
    let event = ble.receive_event().await?;
    match event {
        Packet::Event(Event::LeConnectionComplete(cc)) => {
            defmt::info!(
                "Connected: {:?}",
                defmt::Debug2Format::<defmt::consts::U128>(&cc)
            );
        }

        Packet::Event(Event::DisconnectionComplete(dc)) => {
            defmt::info!(
                "Disconnected: {:?}",
                defmt::Debug2Format::<defmt::consts::U128>(&dc)
            );
        }

        Packet::Event(Event::Vendor(Stm32Wb5xEvent::AttExchangeMtuResponse(
            AttExchangeMtuResponse {
                conn_handle: _,
                server_rx_mtu,
            },
        ))) => {
            defmt::info!("Mtu changed to {:usize}", server_rx_mtu);
        }

        Packet::Event(Event::Vendor(Stm32Wb5xEvent::GattAttributeModified(
            GattAttributeModified { .. },
        ))) => {
            defmt::info!("Gatt attribute modified");
        }

        _ => defmt::warn!(
            "Unhandled event: {:?}",
            defmt::Debug2Format::<defmt::consts::U256>(&event)
        ),
    }

    Ok(())
}
