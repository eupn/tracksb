use crate::imu::MotionData;
use bluetooth_hci::{
    event::command::{CommandComplete, ReturnParameters},
    Event,
};
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
pub struct MotionService {
    handle: ServiceHandle,
    motion_char: MotionCharacteristic,
}

impl MotionService {
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
        let res = ble
            .perform_command(|rc| {
                rc.add_service(&AddServiceParameters {
                    uuid: MotionService::UUID,
                    service_type: ServiceType::Primary,
                    max_attribute_records: Self::MAX_ATTRIBUTE_RECORDS as u8,
                })
            })
            .await?;

        let handle = if let Event::CommandComplete(CommandComplete {
            return_params:
                ReturnParameters::Vendor(stm32wb55::event::command::ReturnParameters::GattAddService(
                    GattService {
                        status: _,
                        service_handle,
                    },
                )),
            ..
        }) = res
        {
            // TODO: check status
            service_handle
        } else {
            return Err(super::BleError::UnexpectedPacket);
        };

        let notify_char = MotionCharacteristic::new(handle, ble).await?;

        Ok(Self {
            handle,
            motion_char: notify_char,
        })
    }

    pub async fn update(
        &mut self,
        ble: &mut Ble,
        motion_data: &MotionData,
    ) -> Result<(), super::BleError> {
        let binary_motion_data: MotionCharacteristicValue = motion_data.into();

        let _result = ble
            .perform_command(|rc| {
                let params = UpdateCharacteristicValueParameters {
                    service_handle: self.handle,
                    characteristic_handle: self.motion_char.handle,
                    offset: 0,
                    value: binary_motion_data.as_slice(),
                };

                rc.update_characteristic_value(&params)
                    .map_err(|_| nb::Error::Other(()))
            })
            .await?;
        // defmt::info!("{:?}", defmt::Debug2Format(&result));

        Ok(())
    }
}

#[derive(Debug)]
pub struct MotionCharacteristic {
    handle: CharacteristicHandle,
}

impl MotionCharacteristic {
    const UUID: Uuid = Uuid::Uuid128([
        0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x11, 0xe1, 0xac, 0x36, 0x00, 0x02, 0xa5, 0xd5, 0xc5,
        0x1b,
    ]);

    pub async fn new(
        service_handle: ServiceHandle,
        ble: &mut Ble,
    ) -> Result<MotionCharacteristic, super::BleError> {
        let return_params = ble
            .perform_command(|rc| {
                rc.add_characteristic(&AddCharacteristicParameters {
                    service_handle,
                    characteristic_uuid: Self::UUID,
                    characteristic_value_len: core::mem::size_of::<MotionCharacteristicValue>(),
                    characteristic_properties: CharacteristicProperty::NOTIFY,
                    security_permissions: CharacteristicPermission::AUTHENTICATED_READ,
                    gatt_event_mask: CharacteristicEvent::ATTRIBUTE_WRITE,
                    encryption_key_size: EncryptionKeySize::with_value(10).expect("ec size"),
                    is_variable: true,
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
        }) = return_params
        {
            // TODO: check status
            defmt::info!("Status: {:?}", defmt::Debug2Format(&status));
            characteristic_handle
        } else {
            return Err(crate::ble::BleError::UnexpectedPacket);
        };

        Ok(Self { handle })
    }
}

#[repr(C, packed)]
pub struct MotionCharacteristicValue {
    quat_x: i16,
    quat_y: i16,
    quat_z: i16,
    quat_w: i16,

    accel_x: i16,
    accel_y: i16,
    accel_z: i16,

    gyro_x: i16,
    gyro_y: i16,
    gyro_z: i16,
}

impl MotionCharacteristicValue {
    /// Reinterprets it as an array.
    fn as_slice(&self) -> &[u8] {
        let len = core::mem::size_of::<Self>();

        // Safety: the struct has C ABI and packed
        unsafe { core::slice::from_raw_parts(self as *const Self as *const u8, len) }
    }
}

impl From<&crate::imu::MotionData> for MotionCharacteristicValue {
    fn from(motion_data: &crate::imu::MotionData) -> Self {
        Self {
            quat_x: motion_data.quat[0],
            quat_y: motion_data.quat[1],
            quat_z: motion_data.quat[2],
            quat_w: motion_data.quat[3],

            accel_x: motion_data.accel[0],
            accel_y: motion_data.accel[1],
            accel_z: motion_data.accel[2],

            gyro_x: motion_data.gyro[0],
            gyro_y: motion_data.gyro[1],
            gyro_z: motion_data.gyro[2],
        }
    }
}

#[derive(Debug)]
pub struct GapContext {
    pub service_handle: ServiceHandle,
    pub dev_name_handle: CharacteristicHandle,
    pub appearance_handle: CharacteristicHandle,
}
