<!-- cargo-rdme start -->

Crate that provides an easy-to-use abstraction for using USB as a simple serial device on SAMD targets.

Due to the API design of the [`usb_device`] crate, using the USB as a serial device typically requires
static variables and careful synchronization between the main thread and interrupt handlers when accessing
them. The main [`UsbSerial`] struct handles these messy details and provides a safe interface.

To use this crate, the SAMD chip variant feature must be enabled, identical to how this is selected when
using [`atsamd_hal`] crate. Refer to the [`atsamd` project GitHub](https://github.com/atsamd-rs/atsamd?tab=readme-ov-file#pac-and-bsp---peripheral-access-crate-and-board-support-package)
for the variant feature names.

Additionally, a static read buffer size must be selected at compile time via one of the following features,
where the number is the desired size of the buffer in bytes:
- `read-buf-32`
- `read-buf-64`
- `read-buf-128`
- `read-buf-256`
- `read-buf-512`

<!-- cargo-rdme end -->
