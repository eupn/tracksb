//! Logs motion data (quaternions, accel & gyro) received over BLE to the file.

use std::io::{Cursor, Write};
use std::path::Path;
use std::str::FromStr;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use btleplug::api::{Central, Peripheral, ValueNotification, UUID};
#[cfg(target_os = "linux")]
use btleplug::bluez::{adapter::ConnectedAdapter, manager::Manager};
#[cfg(target_os = "macos")]
use btleplug::corebluetooth::{adapter::Adapter, manager::Manager};
#[cfg(target_os = "windows")]
use btleplug::winrtble::{adapter::Adapter, manager::Manager};
use byteorder::{LittleEndian, ReadBytesExt};

const LOG_PATH: &str = "motion_log.txt";

const MOTION_CHARACTERISTIC_UUID: &str = "1B:C5:D5:A5:02:00:36:AC:E1:11:01:00:00:01:00:00";

const DEVICE_NAME: &str = "TrackSB";

#[cfg(any(target_os = "windows", target_os = "macos"))]
fn get_central(manager: &Manager) -> Adapter {
    let adapters = manager.adapters().unwrap();
    adapters.into_iter().nth(0).unwrap()
}

#[cfg(target_os = "linux")]
fn get_central(manager: &Manager) -> ConnectedAdapter {
    let adapters = manager.adapters().unwrap();
    let adapter = adapters.into_iter().nth(0).unwrap();
    adapter.connect().unwrap()
}

pub fn main() {
    let manager = Manager::new().unwrap();
    let central = get_central(&manager);
    central.start_scan().unwrap();

    // Wait for some devices to show up
    thread::sleep(Duration::from_secs(5));

    // find the device we're interested in
    let tracksb = central
        .peripherals()
        .into_iter()
        .find(|p| {
            p.properties()
                .local_name
                .iter()
                .any(|name| name.contains(DEVICE_NAME))
        })
        .expect("No TrackSBs found");

    println!("TrackSB found, connecting");

    // connect to the device
    tracksb.connect().unwrap();

    // discover characteristics
    tracksb.discover_characteristics().unwrap();

    // find the characteristic we want
    let chars = tracksb.characteristics();
    let quaternions_char = chars
        .iter()
        .find(|c| c.uuid == UUID::from_str(MOTION_CHARACTERISTIC_UUID).unwrap())
        .unwrap();

    let log = Path::new(LOG_PATH);
    let log_file = Arc::new(Mutex::new(std::fs::File::create(log).unwrap()));
    let log_file = log_file.clone();

    tracksb.on_notification(Box::new(move |notif: ValueNotification| {
        let mut rdr = Cursor::new(notif.value);
        let q_x = rdr.read_f32::<LittleEndian>().unwrap();
        let q_y = rdr.read_f32::<LittleEndian>().unwrap();
        let q_z = rdr.read_f32::<LittleEndian>().unwrap();
        let q_w = rdr.read_f32::<LittleEndian>().unwrap();

        let accel_x = rdr.read_f32::<LittleEndian>().unwrap();
        let accel_y = rdr.read_f32::<LittleEndian>().unwrap();
        let accel_z = rdr.read_f32::<LittleEndian>().unwrap();

        let gyro_x = rdr.read_f32::<LittleEndian>().unwrap();
        let gyro_y = rdr.read_f32::<LittleEndian>().unwrap();
        let gyro_z = rdr.read_f32::<LittleEndian>().unwrap();

        let log_line = format!(
            "{} {} {} {} {} {} {} {} {} {}\n",
            q_x, q_y, q_z, q_w, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z,
        );

        let mut log_file = log_file.lock().unwrap();
        log_file.write_all(log_line.as_bytes()).unwrap();
        log_file.flush().unwrap();
    }));

    println!("Connected, starting streaming data");
    tracksb.subscribe(quaternions_char).unwrap();

    loop {
        thread::sleep(Duration::from_millis(1));
    }
}
