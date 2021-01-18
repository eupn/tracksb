pub mod service;

type BleError = embassy_stm32wb55::ble::BleError<bluetooth_hci::host::uart::Error<(), stm32wb55::event::Stm32Wb5xError>>;
