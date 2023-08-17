// UART0 (serial logs)
macro_rules! get_uart0_tx_pin { ($p:expr) => { $p.PIN_16 }; } // 21, UART0 TX (blue)
macro_rules! get_uart0_rx_pin { ($p:expr) => { $p.PIN_17 }; } // 22, UART0 RX (white)
pub(crate) use get_uart0_tx_pin;
pub(crate) use get_uart0_rx_pin;
