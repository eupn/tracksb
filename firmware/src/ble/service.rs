use crate::imu::Quaternion;
use bluetooth_hci::event::command::ReturnParameters;
use embassy_stm32wb55::ble::Ble;
use stm32wb55::{
    event::command::{GattCharacteristic, GattService},
    gatt::{
        AddCharacteristicParameters, AddServiceParameters, CharacteristicEvent,
        CharacteristicHandle, CharacteristicPermission, CharacteristicProperty,
        Commands as GattCommands, EncryptionKeySize, ServiceHandle, ServiceType,
        UpdateCharacteristicValueParameters, Uuid,
    },
};

#[derive(Debug)]
pub struct QuaternionsService {
    handle: ServiceHandle,
    notify_char: QuaternionsCharacteristic,
}

impl QuaternionsService {
    pub const UUID_BYTES: [u8; 16] = [
        0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x11, 0xe1, 0x9a, 0xb4, 0x00, 0x02, 0xa5, 0xd5, 0xc5,
        0x1b,
    ];
    pub const UUID: Uuid = Uuid::Uuid128(Self::UUID_BYTES);

    /// n = 1 for `QuaternionsService`
    ///     + 2 for `QuaternionsCharacteristic`
    ///     + 1 for client characteristic configuration descriptor
    const MAX_ATTRIBUTE_RECORDS: usize = 8; // TODO: fix to appropriate value

    pub async fn new(ble: &mut Ble) -> Result<Self, crate::ble::BleError> {
        let return_params = ble
            .perform_command(|rc| {
                rc.add_service(&AddServiceParameters {
                    uuid: QuaternionsService::UUID,
                    service_type: ServiceType::Primary,
                    max_attribute_records: Self::MAX_ATTRIBUTE_RECORDS as u8,
                })
            })
            .await?;

        let handle = if let ReturnParameters::Vendor(
            stm32wb55::event::command::ReturnParameters::GattAddService(GattService {
                status: _,
                service_handle,
            }),
        ) = return_params
        {
            // TODO: check status
            service_handle
        } else {
            return Err(super::BleError::UnexpectedEvent);
        };

        let notify_char = QuaternionsCharacteristic::new(handle, ble).await?;

        Ok(Self {
            handle,
            notify_char,
        })
    }

    pub async fn update(
        &mut self,
        ble: &mut Ble,
        quat: &Quaternion,
    ) -> Result<(), super::BleError> {
        let binary_quat: QuaternionsCharacteristicValue = quat.into();

        ble.perform_command(|rc| {
            let params = UpdateCharacteristicValueParameters {
                service_handle: self.handle,
                characteristic_handle: self.notify_char.handle,
                offset: 0,
                value: binary_quat.as_slice(),
            };

            rc.update_characteristic_value(&params)
                .map_err(|_| nb::Error::Other(()))
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
    const UUID: Uuid = Uuid::Uuid128([
        0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x11, 0xe1, 0xac, 0x36, 0x00, 0x02, 0xa5, 0xd5, 0xc5,
        0x1b,
    ]);

    pub async fn new(
        service_handle: ServiceHandle,
        ble: &mut Ble,
    ) -> Result<QuaternionsCharacteristic, super::BleError> {
        let return_params = ble
            .perform_command(|rc| {
                rc.add_characteristic(&AddCharacteristicParameters {
                    service_handle,
                    characteristic_uuid: Self::UUID,
                    characteristic_value_len: core::mem::size_of::<QuaternionsCharacteristicValue>(
                    ),
                    characteristic_properties: CharacteristicProperty::NOTIFY,
                    security_permissions: CharacteristicPermission::AUTHENTICATED_READ,
                    gatt_event_mask: CharacteristicEvent::ATTRIBUTE_WRITE,
                    encryption_key_size: EncryptionKeySize::with_value(10).expect("ec size"),
                    is_variable: true,
                    fw_version_before_v72: false,
                })
            })
            .await?;

        let handle = if let ReturnParameters::Vendor(
            stm32wb55::event::command::ReturnParameters::GattAddCharacteristic(
                GattCharacteristic {
                    status,
                    characteristic_handle,
                },
            ),
        ) = return_params
        {
            // TODO: check status
            defmt::info!(
                "Status: {:?}",
                defmt::Debug2Format::<defmt::consts::U128>(&status)
            );
            characteristic_handle
        } else {
            return Err(crate::ble::BleError::UnexpectedEvent);
        };

        Ok(Self { handle })
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
        unsafe { core::slice::from_raw_parts(self as *const Self as *const u8, len) }
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
