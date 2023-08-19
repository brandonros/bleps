#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

mod pins;
mod led_state;

use cyw43::Control;
use cyw43_pio::PioSpi;
use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output, AnyPin, self};
use embassy_rp::peripherals::{DMA_CH0, PIN_23, PIN_25, PIO0, UART0};
use embassy_rp::pio::Pio;
use embassy_rp::{bind_interrupts, uart};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::{Receiver, Sender, Channel};
use embassy_time::{Duration, Timer};
use led_state::LedState;
use panic_probe as _;
use static_cell::make_static;

bind_interrupts!(struct Irqs {
    UART0_IRQ => embassy_rp::uart::InterruptHandler<UART0>;
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
});

#[link_section = ".boot_loader"]
#[used]
pub static BOOT_LOADER: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

#[link_section = ".firmware"]
#[used]
pub static FIRMWARE: [u8; 232408] = *include_bytes!("../../../embassy/cyw43-firmware/wb43439A0_7_95_49_00_combined.bin");

#[embassy_executor::task]
async fn wifi_task(
    runner: cyw43::Runner<
        'static,
        Output<'static, PIN_23>,
        PioSpi<'static, PIN_25, PIO0, 0, DMA_CH0>,
    >,
) -> ! {
    runner.run().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    // UART0
    let uart0 = uart::Uart::new(
        p.UART0,
        pins::get_uart0_tx_pin!(p),
        pins::get_uart0_rx_pin!(p),
        Irqs,      // unused?
        p.DMA_CH1, // unused?
        p.DMA_CH2, // unused?
        uart::Config::default(),
    );
    // defmt serial
    defmt_serial::defmt_serial(uart0);
    // firmware
    //let firmware = include_bytes!("../../../embassy/cyw43-firmware/wb43439A0_7_95_49_00.bin");
    //let clm = include_bytes!("../../../embassy/cyw43-firmware/wb43439A0_7_95_49_00_clm.bin");
    let clm = include_bytes!("../../../embassy/cyw43-firmware/wb43439A0_7_95_49_00_clm.bin");
    let bluetooth_firmware = include_bytes!("../../../embassy/cyw43-firmware/cyw43_btfw_43439.bin");
    // cyw43 init
    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        pio.irq0,
        cs,
        p.PIN_24,
        p.PIN_29,
        p.DMA_CH0,
    );
    let state = make_static!(cyw43::State::new());
    let (_net_device, mut control, runner) = cyw43::new(state, pwr, spi, &FIRMWARE, bluetooth_firmware).await;
    unwrap!(spawner.spawn(wifi_task(runner)));
    control.init(clm).await;
    control.set_power_management(cyw43::PowerManagementMode::PowerSave).await;
    // LED channel
    /*let led_channel = make_static!(Channel::new());
    let led_channel_receiver = led_channel.receiver();
    let led_channel_sender = led_channel.sender();
    unwrap!(spawner.spawn(led_task(control, led_channel_receiver)));
    unwrap!(spawner.spawn(blink_task(led_channel_sender)));*/
}

#[embassy_executor::task]
async fn led_task(mut control: Control<'static>, receiver: Receiver<'static, ThreadModeRawMutex, LedState, 1>) {
    loop {
        match receiver.recv().await {
            LedState::On => control.gpio_set(0, true).await,
            LedState::Off => control.gpio_set(0, false).await,
        }
    }
}

#[embassy_executor::task]
async fn blink_task(led_sender: Sender<'static, ThreadModeRawMutex, LedState, 1>) {
    loop {
        led_sender.send(LedState::On).await;
        Timer::after(Duration::from_millis(1000)).await;
        led_sender.send(LedState::Off).await;
        Timer::after(Duration::from_millis(1000)).await;
    }
}
