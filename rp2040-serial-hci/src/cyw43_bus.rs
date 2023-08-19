use embassy_futures::yield_now;
use embassy_time::{Duration, Timer, block_for};
use embedded_hal::digital::OutputPin;
use futures::FutureExt;

use crate::{cyw43_consts::*, cyw43_nvram::NVRAM};

/// Custom Spi Trait that _only_ supports the bus operation of the cyw43
/// Implementors are expected to hold the CS pin low during an operation.
pub trait SpiBusCyw43 {
    /// Issues a write command on the bus
    /// First 32 bits of `word` are expected to be a cmd word
    async fn cmd_write(&mut self, write: &[u32]) -> u32;

    /// Issues a read command on the bus
    /// `write` is expected to be a 32 bit cmd word
    /// `read` will contain the response of the device
    /// Backplane reads have a response delay that produces one extra unspecified word at the beginning of `read`.
    /// Callers that want to read `n` word from the backplane, have to provide a slice that is `n+1` words long.
    async fn cmd_read(&mut self, write: u32, read: &mut [u32]) -> u32;

    /// Wait for events from the Device. A typical implementation would wait for the IRQ pin to be high.
    /// The default implementation always reports ready, resulting in active polling of the device.
    async fn wait_for_event(&mut self) {
        yield_now().await;
    }
}

pub(crate) struct Bus<PWR, SPI> {
    backplane_window: u32,
    pwr: PWR,
    spi: SPI,
    status: u32,
}

impl<PWR, SPI> Bus<PWR, SPI>
where
    PWR: OutputPin,
    SPI: SpiBusCyw43,
{
    pub fn new(pwr: PWR, spi: SPI) -> Self {
        Self {
            backplane_window: 0xAAAA_AAAA,
            pwr,
            spi,
            status: 0,
        }
    }

    async fn core_disable(&mut self, core: Core) {
        let base = core.base_addr();

        // Dummy read?
        let _ = self.bp_read8(base + AI_RESETCTRL_OFFSET).await;

        // Check it isn't already reset
        let r = self.bp_read8(base + AI_RESETCTRL_OFFSET).await;
        if r & AI_RESETCTRL_BIT_RESET != 0 {
            return;
        }

        self.bp_write8(base + AI_IOCTRL_OFFSET, 0).await;
        let _ = self.bp_read8(base + AI_IOCTRL_OFFSET).await;

        block_for(Duration::from_millis(1));

        self.bp_write8(base + AI_RESETCTRL_OFFSET, AI_RESETCTRL_BIT_RESET)
            .await;
        let _ = self.bp_read8(base + AI_RESETCTRL_OFFSET).await;
    }

    async fn core_reset(&mut self, core: Core) {
        self.core_disable(core).await;

        let base = core.base_addr();
        self.bp_write8(base + AI_IOCTRL_OFFSET, AI_IOCTRL_BIT_FGC | AI_IOCTRL_BIT_CLOCK_EN)
            .await;
        let _ = self.bp_read8(base + AI_IOCTRL_OFFSET).await;

        self.bp_write8(base + AI_RESETCTRL_OFFSET, 0).await;

        Timer::after(Duration::from_millis(1)).await;

        self.bp_write8(base + AI_IOCTRL_OFFSET, AI_IOCTRL_BIT_CLOCK_EN)
            .await;
        let _ = self.bp_read8(base + AI_IOCTRL_OFFSET).await;

        Timer::after(Duration::from_millis(1)).await;
    }

    async fn core_is_up(&mut self, core: Core) -> bool {
        let base = core.base_addr();

        let io = self.bp_read8(base + AI_IOCTRL_OFFSET).await;
        if io & (AI_IOCTRL_BIT_FGC | AI_IOCTRL_BIT_CLOCK_EN) != AI_IOCTRL_BIT_CLOCK_EN {
            defmt::debug!("core_is_up: returning false due to bad ioctrl {:02x}", io);
            return false;
        }

        let r = self.bp_read8(base + AI_RESETCTRL_OFFSET).await;
        if r & (AI_RESETCTRL_BIT_RESET) != 0 {
            defmt::debug!("core_is_up: returning false due to bad resetctrl {:02x}", r);
            return false;
        }

        true
    }

    pub async fn init(&mut self, firmware: &[u8]) {
        // Reset
        self.pwr.set_low().unwrap();
        Timer::after(Duration::from_millis(20)).await;
        self.pwr.set_high().unwrap();
        Timer::after(Duration::from_millis(250)).await;

        while self
            .read32_swapped(REG_BUS_TEST_RO)
            .inspect(|v| defmt::trace!("{:#x}", v))
            .await
            != FEEDBEAD
        {}

        self.write32_swapped(REG_BUS_TEST_RW, TEST_PATTERN).await;
        let val = self.read32_swapped(REG_BUS_TEST_RW).await;
        defmt::trace!("{:#x}", val);
        assert_eq!(val, TEST_PATTERN);

        let val = self.read32_swapped(REG_BUS_CTRL).await;
        defmt::trace!("{:#010b}", (val & 0xff));

        // 32-bit word length, little endian (which is the default endianess).
        self.write32_swapped(
            REG_BUS_CTRL,
            WORD_LENGTH_32 | HIGH_SPEED | INTERRUPT_HIGH | WAKE_UP | STATUS_ENABLE | INTERRUPT_WITH_STATUS,
        )
        .await;

        let val = self.read8(FUNC_BUS, REG_BUS_CTRL).await;
        defmt::trace!("{:#b}", val);

        let val = self.read32(FUNC_BUS, REG_BUS_TEST_RO).await;
        defmt::trace!("{:#x}", val);
        assert_eq!(val, FEEDBEAD);
        let val = self.read32(FUNC_BUS, REG_BUS_TEST_RW).await;
        defmt::trace!("{:#x}", val);
        assert_eq!(val, TEST_PATTERN);

        // Init ALP (Active Low Power) clock
        self.write8(FUNC_BACKPLANE, REG_BACKPLANE_CHIP_CLOCK_CSR, BACKPLANE_ALP_AVAIL_REQ)
            .await;
        defmt::debug!("waiting for clock...");
        while self.read8(FUNC_BACKPLANE, REG_BACKPLANE_CHIP_CLOCK_CSR).await & BACKPLANE_ALP_AVAIL == 0 {}
        defmt::debug!("clock ok");

        let chip_id = self.bp_read16(0x1800_0000).await;
        defmt::debug!("chip ID: {}", chip_id);

        // Upload firmware.
        self.core_disable(Core::WLAN).await;
        self.core_reset(Core::SOCSRAM).await;
        self.bp_write32(CHIP.socsram_base_address + 0x10, 3).await;
        self.bp_write32(CHIP.socsram_base_address + 0x44, 0).await;

        let ram_addr = CHIP.atcm_ram_base_address;

        defmt::debug!("loading fw");
        self.bp_write(ram_addr, firmware).await;

        defmt::debug!("loading nvram");
        // Round up to 4 bytes.
        let nvram_len = (NVRAM.len() + 3) / 4 * 4;
        self.bp_write(ram_addr + CHIP.chip_ram_size - 4 - nvram_len as u32, NVRAM)
            .await;

        let nvram_len_words = nvram_len as u32 / 4;
        let nvram_len_magic = (!nvram_len_words << 16) | nvram_len_words;
        self.bp_write32(ram_addr + CHIP.chip_ram_size - 4, nvram_len_magic)
            .await;

        // Start core!
        defmt::debug!("starting up core...");
        self.core_reset(Core::WLAN).await;
        assert!(self.core_is_up(Core::WLAN).await);

        while self.read8(FUNC_BACKPLANE, REG_BACKPLANE_CHIP_CLOCK_CSR).await & 0x80 == 0 {}

        // "Set up the interrupt mask and enable interrupts"
        // self.bus.bp_write32(CHIP.sdiod_core_base_address + 0x24, 0xF0).await;
        // TODO: why not all of these F2_F3_FIFO_RD_UNDERFLOW | F2_F3_FIFO_WR_OVERFLOW | COMMAND_ERROR | DATA_ERROR | F2_PACKET_AVAILABLE | F1_OVERFLOW | F1_INTR
        self.write16(FUNC_BUS, REG_BUS_INTERRUPT_ENABLE, IRQ_F2_PACKET_AVAILABLE)
            .await;

        // "Lower F2 Watermark to avoid DMA Hang in F2 when SD Clock is stopped."
        // Sounds scary...
        self.write8(FUNC_BACKPLANE, REG_BACKPLANE_FUNCTION2_WATERMARK, 0x20) // TODO: bluetooth wants 0x10, wifi wants 0x20? cyw43_write_reg_u8(self, BACKPLANE_FUNCTION, SDIO_FUNCTION2_WATERMARK, 0x10);
            .await;

        // wait for wifi startup
        defmt::debug!("waiting for wifi init...");
        while self.read32(FUNC_BUS, REG_BUS_STATUS).await & STATUS_F2_RX_READY == 0 {}

        // Some random configs related to sleep.
        // These aren't needed if we don't want to sleep the bus.
        // TODO do we need to sleep the bus to read the irq line, due to
        // being on the same pin as MOSI/MISO?

        /*
        let mut val = self.bus.read8(FUNC_BACKPLANE, REG_BACKPLANE_WAKEUP_CTRL).await;
        val |= 0x02; // WAKE_TILL_HT_AVAIL
        self.bus.write8(FUNC_BACKPLANE, REG_BACKPLANE_WAKEUP_CTRL, val).await;
        self.bus.write8(FUNC_BUS, 0xF0, 0x08).await; // SDIOD_CCCR_BRCM_CARDCAP.CMD_NODEC = 1
        self.bus.write8(FUNC_BACKPLANE, REG_BACKPLANE_CHIP_CLOCK_CSR, 0x02).await; // SBSDIO_FORCE_HT

        let mut val = self.bus.read8(FUNC_BACKPLANE, REG_BACKPLANE_SLEEP_CSR).await;
        val |= 0x01; // SBSDIO_SLPCSR_KEEP_SDIO_ON
        self.bus.write8(FUNC_BACKPLANE, REG_BACKPLANE_SLEEP_CSR, val).await;
         */

        // clear pulls
        self.write8(FUNC_BACKPLANE, REG_BACKPLANE_PULL_UP, 0).await;
        let _ = self.read8(FUNC_BACKPLANE, REG_BACKPLANE_PULL_UP).await;

        // start HT clock
        //self.bus.write8(FUNC_BACKPLANE, REG_BACKPLANE_CHIP_CLOCK_CSR, 0x10).await;
        //debug!("waiting for HT clock...");
        //while self.bus.read8(FUNC_BACKPLANE, REG_BACKPLANE_CHIP_CLOCK_CSR).await & 0x80 == 0 {}
        //debug!("clock ok");

        #[cfg(feature = "firmware-logs")]
        self.log_init().await;

        defmt::debug!("wifi init done");
    }

    pub async fn wlan_read(&mut self, buf: &mut [u32], len_in_u8: u32) {
        let cmd = cmd_word(READ, INC_ADDR, FUNC_WLAN, 0, len_in_u8);
        let len_in_u32 = (len_in_u8 as usize + 3) / 4;

        self.status = self.spi.cmd_read(cmd, &mut buf[..len_in_u32]).await;
    }

    pub async fn wlan_write(&mut self, buf: &[u32]) {
        let cmd = cmd_word(WRITE, INC_ADDR, FUNC_WLAN, 0, buf.len() as u32 * 4);
        //TODO try to remove copy?
        let mut cmd_buf = [0_u32; 513];
        cmd_buf[0] = cmd;
        cmd_buf[1..][..buf.len()].copy_from_slice(buf);

        self.status = self.spi.cmd_write(&cmd_buf).await;
    }

    #[allow(unused)]
    pub async fn bp_read(&mut self, mut addr: u32, mut data: &mut [u8]) {
        // It seems the HW force-aligns the addr
        // to 2 if data.len() >= 2
        // to 4 if data.len() >= 4
        // To simplify, enforce 4-align for now.
        assert!(addr % 4 == 0);

        // Backplane read buffer has one extra word for the response delay.
        let mut buf = [0u32; BACKPLANE_MAX_TRANSFER_SIZE / 4 + 1];

        while !data.is_empty() {
            // Ensure transfer doesn't cross a window boundary.
            let window_offs = addr & BACKPLANE_ADDRESS_MASK;
            let window_remaining = BACKPLANE_WINDOW_SIZE - window_offs as usize;

            let len = data.len().min(BACKPLANE_MAX_TRANSFER_SIZE).min(window_remaining);

            self.backplane_set_window(addr).await;

            let cmd = cmd_word(READ, INC_ADDR, FUNC_BACKPLANE, window_offs, len as u32);

            // round `buf` to word boundary, add one extra word for the response delay
            self.status = self.spi.cmd_read(cmd, &mut buf[..(len + 3) / 4 + 1]).await;

            // when writing out the data, we skip the response-delay byte
            data[..len].copy_from_slice(&slice8_mut(&mut buf[1..])[..len]);

            // Advance ptr.
            addr += len as u32;
            data = &mut data[len..];
        }
    }

    pub async fn bp_write(&mut self, mut addr: u32, mut data: &[u8]) {
        // It seems the HW force-aligns the addr
        // to 2 if data.len() >= 2
        // to 4 if data.len() >= 4
        // To simplify, enforce 4-align for now.
        assert!(addr % 4 == 0);

        let mut buf = [0u32; BACKPLANE_MAX_TRANSFER_SIZE / 4 + 1];

        while !data.is_empty() {
            // Ensure transfer doesn't cross a window boundary.
            let window_offs = addr & BACKPLANE_ADDRESS_MASK;
            let window_remaining = BACKPLANE_WINDOW_SIZE - window_offs as usize;

            let len = data.len().min(BACKPLANE_MAX_TRANSFER_SIZE).min(window_remaining);
            slice8_mut(&mut buf[1..])[..len].copy_from_slice(&data[..len]);

            self.backplane_set_window(addr).await;

            let cmd = cmd_word(WRITE, INC_ADDR, FUNC_BACKPLANE, window_offs, len as u32);
            buf[0] = cmd;

            self.status = self.spi.cmd_write(&buf[..(len + 3) / 4 + 1]).await;

            // Advance ptr.
            addr += len as u32;
            data = &data[len..];
        }
    }

    pub async fn bp_read8(&mut self, addr: u32) -> u8 {
        self.backplane_readn(addr, 1).await as u8
    }

    pub async fn bp_write8(&mut self, addr: u32, val: u8) {
        self.backplane_writen(addr, val as u32, 1).await
    }

    pub async fn bp_read16(&mut self, addr: u32) -> u16 {
        self.backplane_readn(addr, 2).await as u16
    }

    #[allow(unused)]
    pub async fn bp_write16(&mut self, addr: u32, val: u16) {
        self.backplane_writen(addr, val as u32, 2).await
    }

    #[allow(unused)]
    pub async fn bp_read32(&mut self, addr: u32) -> u32 {
        self.backplane_readn(addr, 4).await
    }

    pub async fn bp_write32(&mut self, addr: u32, val: u32) {
        self.backplane_writen(addr, val, 4).await
    }

    async fn backplane_readn(&mut self, addr: u32, len: u32) -> u32 {
        self.backplane_set_window(addr).await;

        let mut bus_addr = addr & BACKPLANE_ADDRESS_MASK;
        if len == 4 {
            bus_addr |= BACKPLANE_ADDRESS_32BIT_FLAG
        }
        self.readn(FUNC_BACKPLANE, bus_addr, len).await
    }

    async fn backplane_writen(&mut self, addr: u32, val: u32, len: u32) {
        self.backplane_set_window(addr).await;

        let mut bus_addr = addr & BACKPLANE_ADDRESS_MASK;
        if len == 4 {
            bus_addr |= BACKPLANE_ADDRESS_32BIT_FLAG
        }
        self.writen(FUNC_BACKPLANE, bus_addr, val, len).await
    }

    async fn backplane_set_window(&mut self, addr: u32) {
        let new_window = addr & !BACKPLANE_ADDRESS_MASK;

        if (new_window >> 24) as u8 != (self.backplane_window >> 24) as u8 {
            self.write8(
                FUNC_BACKPLANE,
                REG_BACKPLANE_BACKPLANE_ADDRESS_HIGH,
                (new_window >> 24) as u8,
            )
            .await;
        }
        if (new_window >> 16) as u8 != (self.backplane_window >> 16) as u8 {
            self.write8(
                FUNC_BACKPLANE,
                REG_BACKPLANE_BACKPLANE_ADDRESS_MID,
                (new_window >> 16) as u8,
            )
            .await;
        }
        if (new_window >> 8) as u8 != (self.backplane_window >> 8) as u8 {
            self.write8(
                FUNC_BACKPLANE,
                REG_BACKPLANE_BACKPLANE_ADDRESS_LOW,
                (new_window >> 8) as u8,
            )
            .await;
        }
        self.backplane_window = new_window;
    }

    pub async fn read8(&mut self, func: u32, addr: u32) -> u8 {
        self.readn(func, addr, 1).await as u8
    }

    pub async fn write8(&mut self, func: u32, addr: u32, val: u8) {
        self.writen(func, addr, val as u32, 1).await
    }

    pub async fn read16(&mut self, func: u32, addr: u32) -> u16 {
        self.readn(func, addr, 2).await as u16
    }

    #[allow(unused)]
    pub async fn write16(&mut self, func: u32, addr: u32, val: u16) {
        self.writen(func, addr, val as u32, 2).await
    }

    pub async fn read32(&mut self, func: u32, addr: u32) -> u32 {
        self.readn(func, addr, 4).await
    }

    #[allow(unused)]
    pub async fn write32(&mut self, func: u32, addr: u32, val: u32) {
        self.writen(func, addr, val, 4).await
    }

    async fn readn(&mut self, func: u32, addr: u32, len: u32) -> u32 {
        let cmd = cmd_word(READ, INC_ADDR, func, addr, len);
        let mut buf = [0; 2];
        // if we are reading from the backplane, we need an extra word for the response delay
        let len = if func == FUNC_BACKPLANE { 2 } else { 1 };

        self.status = self.spi.cmd_read(cmd, &mut buf[..len]).await;

        // if we read from the backplane, the result is in the second word, after the response delay
        if func == FUNC_BACKPLANE {
            buf[1]
        } else {
            buf[0]
        }
    }

    async fn writen(&mut self, func: u32, addr: u32, val: u32, len: u32) {
        let cmd = cmd_word(WRITE, INC_ADDR, func, addr, len);

        self.status = self.spi.cmd_write(&[cmd, val]).await;
    }

    async fn read32_swapped(&mut self, addr: u32) -> u32 {
        let cmd = cmd_word(READ, INC_ADDR, FUNC_BUS, addr, 4);
        let cmd = swap16(cmd);
        let mut buf = [0; 1];

        self.status = self.spi.cmd_read(cmd, &mut buf).await;

        swap16(buf[0])
    }

    async fn write32_swapped(&mut self, addr: u32, val: u32) {
        let cmd = cmd_word(WRITE, INC_ADDR, FUNC_BUS, addr, 4);
        let buf = [swap16(cmd), swap16(val)];

        self.status = self.spi.cmd_write(&buf).await;
    }

    pub async fn wait_for_event(&mut self) {
        self.spi.wait_for_event().await;
    }

    pub fn status(&self) -> u32 {
        self.status
    }
}

fn swap16(x: u32) -> u32 {
    x.rotate_left(16)
}

fn cmd_word(write: bool, incr: bool, func: u32, addr: u32, len: u32) -> u32 {
    (write as u32) << 31 | (incr as u32) << 30 | (func & 0b11) << 28 | (addr & 0x1FFFF) << 11 | (len & 0x7FF)
}

fn slice8_mut(x: &mut [u32]) -> &mut [u8] {
    let len = x.len() * 4;
    unsafe { core::slice::from_raw_parts_mut(x.as_mut_ptr() as _, len) }
}
