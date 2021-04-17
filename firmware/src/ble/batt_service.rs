use bluetooth_hci::{
    event::command::{CommandComplete, ReturnParameters},
    Event,
};
use embassy_stm32wb55::ble::Ble;
use stm32wb55::{
    event::command::{GattCharacteristic, GattService},
    gatt::{
        AccessPermission, AddCharacteristicParameters, AddDescriptorParameters,
        AddServiceParameters, CharacteristicEvent, CharacteristicHandle, CharacteristicPermission,
        CharacteristicProperty, Commands, DescriptorPermission, EncryptionKeySize, KnownDescriptor,
        ServiceHandle, ServiceType, UpdateCharacteristicValueParameters, Uuid,
    },
};

#[derive(Debug)]
pub struct BatteryService {
    handle: ServiceHandle,
    notify_char: BatteryLevelCharacteristic,
}

impl BatteryService {
    pub const UUID_16: u16 = 0x180F;
    pub const UUID: Uuid = Uuid::Uuid16(Self::UUID_16);
    const MAX_ATTRIBUTE_RECORDS: u8 = 6;

    pub async fn new(ble: &mut Ble) -> Result<Self, crate::ble::BleError> {
        let return_params = ble
            .perform_command(|rc| {
                rc.add_service(&AddServiceParameters {
                    uuid: Self::UUID,
                    service_type: ServiceType::Primary,
                    max_attribute_records: Self::MAX_ATTRIBUTE_RECORDS as u8,
                })
            })
            .await?;

        let handle = if let Event::CommandComplete(CommandComplete {
            return_params:
                ReturnParameters::Vendor(stm32wb55::event::command::ReturnParameters::GattAddService(
                    GattService {
                        status,
                        service_handle,
                    },
                )),
            ..
        }) = return_params
        {
            super::check_status(&status);
            service_handle
        } else {
            return Err(crate::ble::BleError::UnexpectedPacket);
        };

        let batt_char = BatteryLevelCharacteristic::new(handle, ble).await?;

        Ok(Self {
            handle,
            notify_char: batt_char,
        })
    }

    pub async fn update(&mut self, ble: &mut Ble, level: u8) -> Result<(), super::BleError> {
        ble.perform_command(|rc| {
            let params = UpdateCharacteristicValueParameters {
                service_handle: self.handle,
                characteristic_handle: self.notify_char.handle,
                offset: 0,
                value: &[level],
            };

            rc.update_characteristic_value(&params)
                .map_err(|_| nb::Error::Other(()))
        })
        .await?;

        Ok(())
    }
}

#[derive(Debug)]
pub struct BatteryLevelCharacteristic {
    handle: CharacteristicHandle,
}

impl BatteryLevelCharacteristic {
    const UUID: Uuid = Uuid::Uuid16(0x2A19);

    pub async fn new(
        service_handle: ServiceHandle,
        ble: &mut Ble,
    ) -> Result<BatteryLevelCharacteristic, super::BleError> {
        let res = ble
            .perform_command(|rc| {
                rc.add_characteristic(&AddCharacteristicParameters {
                    service_handle,
                    characteristic_uuid: Self::UUID,
                    characteristic_value_len: 1,
                    characteristic_properties: CharacteristicProperty::READ
                        | CharacteristicProperty::NOTIFY,
                    security_permissions: CharacteristicPermission::AUTHENTICATED_WRITE,
                    gatt_event_mask: CharacteristicEvent::ATTRIBUTE_WRITE,
                    encryption_key_size: EncryptionKeySize::with_value(7).expect("ec size"),
                    is_variable: false,
                    fw_version_before_v72: false,
                })
            })
            .await?;

        let handle = if let Event::CommandComplete(CommandComplete {
            return_params:
                ReturnParameters::Vendor(
                    stm32wb55::event::command::ReturnParameters::GattAddCharacteristic(
                        GattCharacteristic {
                            status,
                            characteristic_handle,
                        },
                    ),
                ),
            ..
        }) = res
        {
            super::check_status(&status);
            characteristic_handle
        } else {
            return Err(crate::ble::BleError::UnexpectedPacket);
        };

        ble.perform_command(|rc| {
            let desc = AddDescriptorParameters {
                service_handle,
                characteristic_handle: handle,
                descriptor_uuid: KnownDescriptor::CharacteristicPresentationFormat.into(),
                descriptor_value_max_len: 1,
                descriptor_value: &[0x04], // 8-bit unsigned integer
                security_permissions: DescriptorPermission::AUTHENTICATED,
                access_permissions: AccessPermission::READ,
                gatt_event_mask: CharacteristicEvent::ATTRIBUTE_WRITE,
                encryption_key_size: EncryptionKeySize::with_value(10).expect("ec size"),
                is_variable: false,
            };

            rc.add_characteristic_descriptor(&desc)
                .map_err(|_| nb::Error::Other(()))
        })
        .await?;

        Ok(Self { handle })
    }
}
