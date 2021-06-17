use embedded_storage::{ReadStorage, Storage};
use stm32wb_hal::flash::{FlashPage, FlashProgramming, Read, WriteErase};

const FLASH_PAGE_SIZE_BYTES: u32 = 4096;
const FLASH_BASE_ADDRESS: u32 = 0x0800_0000;
const LOGS_BASE_ADDRESS: u32 = FLASH_BASE_ADDRESS + 64 * FLASH_PAGE_SIZE_BYTES;
const LOGS_END_ADDRESS: u32 = LOGS_BASE_ADDRESS + 5 * FLASH_PAGE_SIZE_BYTES;

pub struct LogsStorage<'a>(FlashProgramming<'a>);

impl<'a> LogsStorage<'a> {
    pub fn erase(&mut self) -> Result<(), ()> {
        let mut page_nr = LOGS_BASE_ADDRESS / FLASH_PAGE_SIZE_BYTES;
        loop {
            self.0
                .erase_page(FlashPage(page_nr as usize))
                .map_err(|_| ())?;
            page_nr += 1;

            if page_nr * FLASH_PAGE_SIZE_BYTES >= LOGS_END_ADDRESS {
                break;
            }
        }

        Ok(())
    }
}

impl<'a> From<FlashProgramming<'a>> for LogsStorage<'a> {
    fn from(fp: FlashProgramming<'a>) -> Self {
        LogsStorage(fp)
    }
}

impl<'a> ReadStorage for LogsStorage<'a> {
    type Error = ();

    fn try_read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.0
            .read(LOGS_BASE_ADDRESS as usize + offset as usize, bytes);
        Ok(())
    }

    fn capacity(&self) -> usize {
        (LOGS_END_ADDRESS - LOGS_BASE_ADDRESS) as usize
    }
}

impl<'a> Storage for LogsStorage<'a> {
    fn try_write(&mut self, address: u32, bytes: &[u8]) -> Result<(), ()> {
        let address = LOGS_BASE_ADDRESS + address;

        if address > LOGS_END_ADDRESS {
            return Err(());
        }

        if self.0.write(address as usize, bytes).is_err() {
            return Err(());
        }

        Ok(())
    }
}
