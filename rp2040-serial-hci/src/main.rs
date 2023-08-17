#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

mod led_state;
mod pins;

use defmt_serial as _;
use embassy_executor::Executor;
use embassy_rp::{
    bind_interrupts,
    gpio::{self, AnyPin, Level, Pin},
    peripherals::UART0,
    uart::{self, InterruptHandler},
};
use embassy_sync::{
    blocking_mutex::raw::ThreadModeRawMutex,
    channel::{Channel, Receiver, Sender},
};

use embassy_time::{Timer, Duration};
use panic_probe as _;
use static_cell::StaticCell;

use crate::led_state::LedState;

bind_interrupts!(struct Irqs {
    UART0_IRQ => InterruptHandler<UART0>;
});

static CORE0_EXECUTOR: StaticCell<Executor> = StaticCell::new();
static LED_CHANNEL: StaticCell<Channel<ThreadModeRawMutex, LedState, 1>> = StaticCell::new();

#[cortex_m_rt::entry]
fn main() -> ! {
    let p = embassy_rp::init(Default::default());
    // UART0
    let uart0 = uart::Uart::new(
        p.UART0,
        pins::get_uart0_tx_pin!(p),
        pins::get_uart0_rx_pin!(p),
        Irqs,      // unused?
        p.DMA_CH0, // unused?
        p.DMA_CH1, // unused?
        uart::Config::default(),
    );
    // defmt serial
    defmt_serial::defmt_serial(uart0);
    // log
    defmt::info!("init");
    // LED channel
    let led_channel = LED_CHANNEL.init(Channel::new());
    let led_channel_receiver = led_channel.receiver();
    let led_channel_sender = led_channel.sender();
    // spawn tasks (core0)
    let core0_executor = CORE0_EXECUTOR.init(Executor::new());
    core0_executor.run(|spawner| {
        spawner.must_spawn(led_task(
            pins::get_led_pin!(p).degrade(),
            led_channel_receiver,
        ));
        spawner.must_spawn(blink_task(
            led_channel_sender
        ));
    });
}

#[embassy_executor::task]
async fn led_task(led_pin: AnyPin, receiver: Receiver<'static, ThreadModeRawMutex, LedState, 1>) {
    let mut led_output = gpio::Output::new(led_pin, Level::Low);
    loop {
        match receiver.recv().await {
            LedState::On => led_output.set_high(),
            LedState::Off => led_output.set_low(),
        }
    }
}

#[embassy_executor::task]
async fn blink_task(led_sender: Sender<'static, ThreadModeRawMutex, LedState, 1>) {
    loop {
        led_sender.send(LedState::On).await;
        Timer::after(Duration::from_millis(100)).await;
        led_sender.send(LedState::Off).await;
        Timer::after(Duration::from_millis(5)).await;
    }
}