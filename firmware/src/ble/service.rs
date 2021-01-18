use stm32wb55::gatt::{AddServiceParameters, CharacteristicHandle, Commands as GattCommands, ServiceHandle, ServiceType, Uuid, CharacteristicProperty, CharacteristicPermission, CharacteristicEvent, AddCharacteristicParameters, EncryptionKeySize, UpdateCharacteristicValueParameters};
use stm32wb55::gap::{Commands as GapCommands, Role, DiscoverableParameters, LocalName, AdvertisingDataType, AddressType};
use stm32wb55::hal::Commands as HalCommands;
use bluetooth_hci::host::{EncryptionKey, OwnAddressType, AdvertisingFilterPolicy};
use stm32wb55::hal::{ConfigData, PowerLevel};
use bluetooth_hci::{BdAddr, Event, Status};
use bluetooth_hci::event::command::{CommandComplete, ReturnParameters};
use bluetooth_hci::types::AdvertisingType;
use core::time::Duration;
use embassy_stm32wb55::ble::{Ble, BleError};
use stm32wb55::event::command::GattService;
use stm32wb_hal::tl_mbox::lhci::LhciC1DeviceInformationCcrp;
use stm32wb55::event::command::GattCharacteristic;
use bluetooth_hci::host::uart::Packet;
use crate::imu::Quaternion;

/// Advertisement interval in milliseconds.
const ADV_INTERVAL_MS: u64 = 250;

const BLE_DEVICE_LOCAL_NAME: &[u8] = b"Quaternions";
const BLE_GAP_DEVICE_NAME: &[u8] = b"Quaternions";

#[derive(Debug)]
pub struct QuaternionsService {
    handle: ServiceHandle,
    notify_char: QuaternionsCharacteristic,
}

impl QuaternionsService {
    /// 0000fe40-cc7a-482a-984a7f2ed5b3e58f
    const UUID: Uuid = Uuid::Uuid128([
        0x00, 0x00, 0xfe, 0x40, 0xcc, 0x7a, 0x48, 0x2a,
        0x98, 0x4a, 0x7f, 0x2e, 0xd5, 0xb3, 0xe5, 0x8f,
    ]);

    /// n = 1 for `QuaternionsService`
    ///     + 2 for `QuaternionsCharacteristic`
    ///     + 1 for client characteristic configuration descriptor
    const MAX_ATTRIBUTE_RECORDS: usize = 8; // TODO: fix to appropriate value

    pub async fn new(ble: &mut Ble) -> Result<Self, crate::ble::BleError> {
        let return_params = ble.perform_command(|rc| rc.add_service(&AddServiceParameters {
            uuid: QuaternionsService::UUID,
            service_type: ServiceType::Primary,
            max_attribute_records: Self::MAX_ATTRIBUTE_RECORDS as u8,
        })).await?;

        let handle = if let ReturnParameters::Vendor(
            stm32wb55::event::command::ReturnParameters::GattAddService(GattService { status: _, service_handle })) = return_params {
            // TODO: check status
            service_handle
        } else {
            return Err(crate::ble::BleError::UnexpectedEvent)
        };

        let notify_char = QuaternionsCharacteristic::new(handle, ble).await?;

        set_advertisement(true, ble).await?;

        Ok(Self {
            handle,
            notify_char,
        })
    }

    pub async fn update(&mut self, ble: &mut Ble, quat: &Quaternion) -> Result<(), super::BleError> {
        let binary_quat: QuaternionsCharacteristicValue = quat.into();

        ble.perform_command(|rc| {
            let params = UpdateCharacteristicValueParameters {
                service_handle: self.handle,
                characteristic_handle: self.notify_char.handle,
                offset: 0,
                value: binary_quat.as_slice(),
            };

            rc.update_characteristic_value(&params).map_err(|_| nb::Error::Other(()))
        })
            .await?;

        Ok(())
    }
}

#[derive(Debug)]
pub struct QuaternionsCharacteristic {
    handle: CharacteristicHandle,
}

impl QuaternionsCharacteristic {
    /// 00000080-0001-11e1-ac36-0002a5d5c51b
    const UUID: Uuid = Uuid::Uuid128([
        0x00, 0x00, 0x00, 0x80, 0x00, 0x01, 0x11, 0xe1,
        0xac, 0x36, 0x00, 0x02, 0xa5, 0xd5, 0xc5, 0x1b,
    ]);

    pub async fn new(service_handle: ServiceHandle, ble: &mut Ble) -> Result<QuaternionsCharacteristic, super::BleError> {
        let return_params = ble.perform_command(|rc| {
            rc.add_characteristic(&AddCharacteristicParameters {
                service_handle,
                characteristic_uuid: Self::UUID,
                characteristic_value_len: core::mem::size_of::<QuaternionsCharacteristicValue>(),
                characteristic_properties: CharacteristicProperty::NOTIFY,
                security_permissions: CharacteristicPermission::AUTHENTICATED_READ,
                gatt_event_mask: CharacteristicEvent::ATTRIBUTE_WRITE,
                encryption_key_size: EncryptionKeySize::with_value(10).expect("ec size"),
                is_variable: true,
                fw_version_before_v72: false,
            })
        }).await?;

        let handle = if let ReturnParameters::Vendor(
            stm32wb55::event::command::ReturnParameters::GattAddCharacteristic(GattCharacteristic { status: _, characteristic_handle  })) = return_params {
            // TODO: check status
            characteristic_handle
        } else {
            return Err(crate::ble::BleError::UnexpectedEvent)
        };

        Ok(Self {
            handle,
        })
    }
}

#[repr(C, packed)]
pub struct QuaternionsCharacteristicValue {
    i: f32,
    j: f32,
    k: f32,
    s: f32,
}

impl QuaternionsCharacteristicValue {
    /// Reinterprets it as an array.
    fn as_slice(&self) -> &[u8] {
        let len = core::mem::size_of::<Self>();

        // Safety: the struct has C ABI and packed
        unsafe {
            core::slice::from_raw_parts(self as *const Self as *const u8, len)
        }
    }
}

impl From<&crate::imu::Quaternion> for QuaternionsCharacteristicValue {
    fn from(quat: &crate::imu::Quaternion) -> Self {
        Self {
            i: quat[0],
            j: quat[1],
            k: quat[2],
            s: quat[3],
        }
    }
}

#[derive(Debug)]
pub struct GapContext {
    pub service_handle: ServiceHandle,
    pub dev_name_handle: CharacteristicHandle,
    pub appearance_handle: CharacteristicHandle,
}

fn get_bd_addr() -> BdAddr {
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

fn get_random_addr() -> BdAddr {
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

pub async fn init_gap_and_gatt(ble: &mut Ble) -> Result<(), super::BleError> {
    ble.perform_command(|rc| rc.write_config_data(&ConfigData::public_address(get_bd_addr()).build()))
        .await?;

    ble.perform_command(|rc| rc.write_config_data(&ConfigData::random_address(get_random_addr()).build()))
        .await?;

    ble.perform_command(|rc| rc.set_tx_power_level(PowerLevel::ZerodBm))
        .await?;
    //
    ble.perform_command(|rc| rc.init_gatt())
        .await?;

    let return_params = ble.perform_command(|rc| rc.init_gap(Role::PERIPHERAL, false, BLE_GAP_DEVICE_NAME.len() as u8))
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
        return Err(super::BleError::UnexpectedEvent)
    };

    ble.perform_command(|rc| {
        rc.update_characteristic_value(&UpdateCharacteristicValueParameters {
            service_handle,
            characteristic_handle: dev_name_handle,
            offset: 0,
            value: BLE_GAP_DEVICE_NAME,
        }).map_err(|_| nb::Error::Other(()))
    })
        .await?;

    ble.perform_command(|rc| {
        rc.update_characteristic_value(&UpdateCharacteristicValueParameters {
            service_handle,
            characteristic_handle: appearance_handle,
            offset: 0,
            value: &[0],
        }).map_err(|_| nb::Error::Other(()))
    })
        .await?;

    Ok(())
}

pub async fn set_advertisement(enabled: bool, ble: &mut Ble) -> Result<(), super::BleError> {
    if enabled {
        ble.perform_command(|rc| {
            let addr = get_bd_addr().0;

            let mut manufacturer_specific_data = [0u8; 14];
            manufacturer_specific_data[0] = 13;
            manufacturer_specific_data[1] = AdvertisingDataType::ManufacturerSpecificData as u8;

            manufacturer_specific_data[2] = 0x01; // BlueST protocol version
            manufacturer_specific_data[3] = 0x83; // P2PServer1

            manufacturer_specific_data[4] = 0x00; // BlueST Feature Mask bits 24~31
            manufacturer_specific_data[5] = 0x00; // BlueST Feature Mask bits 16~23
            manufacturer_specific_data[6] = 0x00; // BlueST Feature Mask bits 8~15
            manufacturer_specific_data[7] = 0x00; // BlueST Feature Mask bits 0~7

            manufacturer_specific_data[8..(8 + addr.len())].copy_from_slice(&addr[..]);

            rc.update_advertising_data(&manufacturer_specific_data[..]).map_err(|_| nb::Error::Other(()))
        })
            .await?;

        ble.perform_command(|rc| {
            let params = DiscoverableParameters {
                advertising_type: AdvertisingType::ConnectableUndirected,
                advertising_interval: Some((
                    Duration::from_millis(ADV_INTERVAL_MS),
                    Duration::from_millis(ADV_INTERVAL_MS),
                )),
                address_type: OwnAddressType::Public,
                filter_policy: AdvertisingFilterPolicy::AllowConnectionAndScan,
                local_name: Some(LocalName::Complete(BLE_DEVICE_LOCAL_NAME)),
                advertising_data: &[],
                conn_interval: (None, None),
            };

            rc.set_discoverable(&params).map_err(|_| nb::Error::Other(()))
        })
            .await?;
    } else {
        ble.perform_command(|rc| rc.set_nondiscoverable())
            .await?;
    }

    Ok(())
}

pub async fn process_event(ble: &mut Ble) -> Result<(), super::BleError> {
    let event = ble.receive_event().await?;
    match event {
        Packet::Event(Event::LeConnectionComplete(_)) => {
            defmt::info!("Connection complete");

            // Start advertising again
            set_advertisement(true, ble).await?;
        }

        Packet::Event(Event::DisconnectionComplete(_)) => {
            defmt::info!("Disconnection complete");

            // Start advertising again
            set_advertisement(true, ble).await?;
        }

        _ => defmt::warn!("Unhandled event: {:?}", defmt::Debug2Format::<defmt::consts::U256>(&event)),
    }

    Ok(())
}
