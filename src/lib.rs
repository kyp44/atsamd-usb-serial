//! Crate that provides an easy-to-use abstraction for using USB as a simple
//! serial device on SAMD targets.
//!
//! Due to the API design of the [`usb_device`] crate, using the USB as a serial
//! device typically requires static variables and careful synchronization
//! between the main thread and interrupt handlers when accessing them. The main
//! [`UsbSerial`] struct handles these messy details and provides a safe
//! interface. This crate also implements the USB interrupt handlers so that
//! these should not be implemented manually.
//!
//! To use this crate, the SAMD chip variant feature must be enabled, identical to how this is selected when using [`atsamd_hal`] crate. Refer to the [`atsamd` project GitHub](https://github.com/atsamd-rs/atsamd?tab=readme-ov-file#pac-and-bsp---peripheral-access-crate-and-board-support-package) for the variant feature names.
//!
//! Additionally, a static read buffer size must be selected at compile time via
//! one of the following features, where the number is the desired size of the
//! buffer in bytes:
//! - `read-buf-32`
//! - `read-buf-64`
//! - `read-buf-128`
//! - `read-buf-256`
//! - `read-buf-512`
//!
//! Enable the `heapless` feature to add additional methods to [`UsbSerial`]
//! that are easier to use but depend on the [`heapless`] crate, which is also
//! re-exported.

#![no_std]
#![allow(static_mut_refs)]
#![warn(missing_docs)]
#![warn(rustdoc::missing_crate_level_docs)]
#![warn(clippy::missing_docs_in_private_items)]

#[featurecomb::comb]
mod _featurecomb {}

// Re-exports
pub use atsamd_hal::usb::usb_device;
#[cfg(feature = "heapless")]
pub use heapless;

use atsamd_hal::{
    pac::{self, interrupt},
    usb::UsbBus,
};
use atsamd_hal_macros::{hal_cfg, hal_macro_helper};
use core::{cell::OnceCell, fmt::Write};
use cortex_m::peripheral::NVIC;
use heapless::Vec;
#[cfg(feature = "heapless")]
use heapless::{VecView, string::StringView};
use usb_device::{
    bus::UsbBusAllocator,
    device::{StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbVidPid},
    prelude::BuilderError,
};
use usbd_serial::{SerialPort, USB_CLASS_CDC, UsbError};

mod heapless_ext;
#[cfg(feature = "heapless")]
use heapless_ext::StringExt;
use heapless_ext::VecExt;

/// Items needed for setting up and using a [`UsbSerial`].
pub mod prelude {
    pub use crate::{
        UsbSerial, UsbSerialError,
        usb_device::{
            descriptor::lang_id::LangID,
            device::{StringDescriptors, UsbVidPid},
        },
    };
}

/// Errors that can occur when using a [`UsbSerial`].
#[derive(Debug)]
pub enum UsbSerialError {
    /// The singular [`UsbSerial`] has already been setup and is in use.
    AlreadySetup,
    /// Something went wrong when building the
    /// [`usb_device::device::UsbDevice`].
    DeviceBuilder(BuilderError),
    /// The action could not be fully completed because the buffer is full.
    ///
    /// This contains the number of items written to the buffer.
    BufferFull(usize),
    /// The USB operation did not complete successfully.
    Usb(UsbError),
}
impl From<BuilderError> for UsbSerialError {
    fn from(value: BuilderError) -> Self {
        Self::DeviceBuilder(value)
    }
}
impl From<UsbError> for UsbSerialError {
    fn from(value: UsbError) -> Self {
        Self::Usb(value)
    }
}

/// Size of the static read buffer.
pub const READ_BUFFER_SIZE: usize = {
    #[cfg(feature = "read-buf-32")]
    {
        32
    }
    #[cfg(feature = "read-buf-64")]
    {
        64
    }
    #[cfg(feature = "read-buf-128")]
    {
        128
    }
    #[cfg(feature = "read-buf-256")]
    {
        256
    }
    #[cfg(feature = "read-buf-512")]
    {
        512
    }
};

/// The USB items that need to be static, other than the allocator.
struct UsbPackage {
    pub serial_port: SerialPort<'static, UsbBus>,
    pub usb_device: UsbDevice<'static, UsbBus>,
    pub echo: bool,
    pub read_buffer: Vec<u8, READ_BUFFER_SIZE>,
}

static mut USB_ALLOCATOR: OnceCell<UsbBusAllocator<UsbBus>> = OnceCell::new();
static mut USB_PACKAGE: OnceCell<UsbPackage> = OnceCell::new();

/// Abstraction to use the USB as a serial device.
///
/// Methods are provided to read and write raw data to the serial port. This
/// also implements TODO? WRITEBUFFER_SIZE. This is backed by static variables
/// to interact with the interrupt handlers, and so is a singleton.
///
/// TODO: Explain read buffer.
///
/// TODO: What happens when the read or write buffers get full?
///
/// # Example
/// TODO
pub struct UsbSerial<const WRITE_BUFFER_SIZE: usize = 128> {
    write_buffer: Vec<u8, WRITE_BUFFER_SIZE>,
}
impl<const WRITE_BUFFER_SIZE: usize> UsbSerial<WRITE_BUFFER_SIZE> {
    /// Creates a new USB serial device with the specified `bus_allocator`,
    /// `descriptors`, and `vid_pid`.
    ///
    /// The interrupt handler will always echo back anything it receives if
    /// `echo` is `true` and will never do so if it is `false`. This also
    /// enables the appropriate USB interrupts for the SAMD variant, which is
    /// why `nvic` is required. As [`UsbSerial`] is a singleton backed by static
    /// data, this returns an [`Err`] with [`UsbSerialError::AlreadySetup`] if
    /// this has already been called and the device already created.
    #[hal_macro_helper]
    pub fn new(
        nvic: &mut pac::NVIC,
        bus_allocator: UsbBusAllocator<UsbBus>,
        descriptors: StringDescriptors<'static>,
        vid_pid: UsbVidPid,
        echo: bool,
    ) -> Result<Self, UsbSerialError> {
        // Set the allocator
        let bus_allocator = unsafe {
            // Return if already set
            USB_ALLOCATOR
                .set(bus_allocator)
                .map_err(|_| UsbSerialError::AlreadySetup)?;
            USB_ALLOCATOR.get().unwrap()
        };

        // Setup the USB serial device
        let serial_port = SerialPort::new(bus_allocator);
        let usb_device = UsbDeviceBuilder::new(bus_allocator, vid_pid)
            .strings(&[descriptors])?
            .device_class(USB_CLASS_CDC)
            .build();
        unsafe {
            let _ = USB_PACKAGE.set(UsbPackage {
                serial_port,
                usb_device,
                echo,
                read_buffer: Vec::new(),
            });
        }

        // Enable the USB interrupts
        #[hal_cfg("usb-d5x")]
        unsafe {
            nvic.set_priority(interrupt::USB_OTHER, 1);
            nvic.set_priority(interrupt::USB_TRCPT0, 1);
            nvic.set_priority(interrupt::USB_TRCPT1, 1);
            NVIC::unmask(interrupt::USB_OTHER);
            NVIC::unmask(interrupt::USB_TRCPT0);
            NVIC::unmask(interrupt::USB_TRCPT1);
        }
        #[hal_cfg(any("usb-d11", "usb-d21"))]
        unsafe {
            nvic.set_priority(interrupt::USB, 1);
            NVIC::unmask(interrupt::USB);
        }

        Ok(Self {
            write_buffer: Vec::new(),
        })
    }

    /// Writes raw bytes into the write buffer.
    ///
    /// This will return an [`Err`] with [`UsbSerialError::BufferFull`]
    /// containing the number of bytes written if not all the data could be
    /// written due to the write buffer filling up. In this case, as much
    /// data was written to the buffer as possible so that it will be
    /// completely full.
    ///
    /// Data will not be written the serial device until [`UsbSerial::flush`] is
    /// called.
    #[inline]
    pub fn write(&mut self, data: &[u8]) -> Result<(), UsbSerialError> {
        self.write_buffer
            .extend_from_slice_until_full(data)
            .map_err(|n| UsbSerialError::BufferFull(n))
    }

    /// Flushes the write buffer, writing out all data to the serial device.
    ///
    /// After flushing, this clears the write buffer. If there was an issue
    /// writing to the serial device, this will return an [`Err`] with
    /// [`UsbSerialError::Usb`] containing the specific [`UsbError`] and the
    /// write buffer will not be cleared.
    pub fn flush(&mut self) -> Result<(), UsbSerialError> {
        // Ensure that all bytes are written
        Self::serial_get(|serial| -> Result<(), UsbError> {
            let mut remaining = self.write_buffer.as_slice();
            while !remaining.is_empty() {
                let num_sent = serial.write(remaining)?;
                remaining = &remaining[num_sent..];
            }
            Ok(())
        })?;

        self.write_buffer.clear();

        Ok(())
    }

    /// Reads raw bytes from the read buffer into the `data` buffer.
    ///
    /// After copying the data, the read buffer will be totally cleared, even if
    /// not all of its data could be copied into `data` due to `data` being too
    /// small. This can be avoided if `data` is at least as as large as
    /// [`READ_BUFFER_SIZE`]. Returns the number of bytes copied, which will be
    /// `0` if no data was available.
    pub fn read<'a>(&self, data: &'a mut [u8]) -> usize {
        Self::usb_free(|_| {
            let read_buffer = unsafe { &mut USB_PACKAGE.get_mut().unwrap().read_buffer };
            let copy_size = data.len().min(read_buffer.len());

            if copy_size > 0 {
                data[..copy_size].copy_from_slice(&read_buffer[..copy_size]);

                read_buffer.clear();
            }

            copy_size
        })
    }

    /// Appends raw bytes from the read buffer to `vec`.
    ///
    /// After copying the data, the read buffer will be totally cleared, even if
    /// not all of its data could be copied into `vec` due to `vec` not having
    /// enough capacity. In this case an [`Err`] will be returned with
    /// [`UsbSerialError::BufferFull`] containing the number of bytes copied.
    /// This can be avoided if `vec` is at least as as large as
    /// [`READ_BUFFER_SIZE`].
    ///
    /// If [`Ok`] is returned, it will contain whether or not data was available
    /// in the read buffer.
    #[cfg(feature = "heapless")]
    pub fn read_vec(&self, vec: &mut VecView<u8>) -> Result<bool, UsbSerialError> {
        Self::usb_free(|_| {
            let read_buffer = unsafe { &mut USB_PACKAGE.get_mut().unwrap().read_buffer };
            let res = vec
                .extend_from_slice_until_full(read_buffer)
                .map_err(|n| UsbSerialError::BufferFull(n))
                .map(|_| !read_buffer.is_empty());
            read_buffer.clear();

            res
        })
    }

    /// Appends a string from the read buffer to `string`.
    ///
    /// After copying the data, the read buffer will be totally cleared, even if
    /// not all of its data could be appended to `string`. If `string`
    /// does not have sufficient capacity, an [`Err`] will be returned with
    /// [`UsbSerialError::BufferFull`] containing the number of UTF8 characters
    /// copied. This can be avoided if `string` is at least as as large as
    /// [`READ_BUFFER_SIZE`]. The read buffer will only be appended for as long
    /// as it contains valid UTF8.
    ///
    /// If [`Ok`] is returned, it will contain whether or not data was available
    /// in the read buffer.
    #[cfg(feature = "heapless")]
    pub fn read_string(&self, string: &mut StringView) -> Result<bool, UsbSerialError> {
        Self::usb_free(|_| {
            let read_buffer = unsafe { &mut USB_PACKAGE.get_mut().unwrap().read_buffer };
            let res = string
                .push_raw_buffer_until_full(read_buffer)
                .map_err(|n| UsbSerialError::BufferFull(n));

            read_buffer.clear();

            res
        })
    }

    /// Borrows the global singleton `UsbSerial` for a brief period with
    /// interrupts disabled
    ///
    /// # Arguments
    /// `borrower`: The closure that gets run borrowing the global `UsbSerial`
    ///
    /// # Safety
    /// the global singleton `UsbSerial` can be safely borrowed because we
    /// disable interrupts while it is being borrowed, guaranteeing that
    /// interrupt handlers like `USB` cannot mutate `UsbSerial` while we are
    /// as well.
    ///
    /// # Panic
    /// If `init` has not been called and we haven't initialized our global
    /// singleton `UsbSerial`, we will panic.
    fn serial_get<T, R>(borrower: T) -> R
    where
        T: Fn(&mut SerialPort<UsbBus>) -> R,
    {
        Self::usb_free(|_| unsafe { borrower(&mut USB_PACKAGE.get_mut().unwrap().serial_port) })
    }

    /// Execute closure `f` in a context free of USB interrupts.
    ///
    /// This as also known as a "critical section".
    #[hal_macro_helper]
    #[inline]
    fn usb_free<F, R>(f: F) -> R
    where
        F: FnOnce(&cortex_m::interrupt::CriticalSection) -> R,
    {
        #[hal_cfg("usb-d5x")]
        {
            NVIC::mask(interrupt::USB_OTHER);
            NVIC::mask(interrupt::USB_TRCPT0);
            NVIC::mask(interrupt::USB_TRCPT1);
        }

        #[hal_cfg(any("usb-d11", "usb-d21"))]
        NVIC::mask(interrupt::USB);

        let r = f(&unsafe { cortex_m::interrupt::CriticalSection::new() });

        #[hal_cfg("usb-d5x")]
        unsafe {
            NVIC::unmask(interrupt::USB_OTHER);
            NVIC::unmask(interrupt::USB_TRCPT0);
            NVIC::unmask(interrupt::USB_TRCPT1);
        };
        #[hal_cfg(any("usb-d11", "usb-d21"))]
        unsafe {
            NVIC::unmask(interrupt::USB);
        };

        r
    }
}
impl<const WRITE_BUFFER_SIZE: usize> Write for UsbSerial<WRITE_BUFFER_SIZE> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.write_buffer.write_str(s)
    }
}

// TODO
fn poll_usb() {
    unsafe {
        if let Some(package) = USB_PACKAGE.get_mut() {
            package.usb_device.poll(&mut [&mut package.serial_port]);
            let mut buf = [0u8; 64];

            if let Ok(count) = package.serial_port.read(&mut buf)
                && count > 0
            {
                let _ = package.read_buffer.extend_from_slice(&buf[..count]);

                if package.echo {
                    package.serial_port.write(&buf[..count]).unwrap();
                }
            }
        }
    }
}

#[hal_cfg("usb-d5x")]
#[interrupt]
fn USB_OTHER() {
    poll_usb();
}

#[hal_cfg("usb-d5x")]
#[interrupt]
fn USB_TRCPT0() {
    poll_usb();
}

#[hal_cfg("usb-d5x")]
#[interrupt]
fn USB_TRCPT1() {
    poll_usb();
}

#[hal_cfg(any("usb-d11", "usb-d21"))]
#[interrupt]
fn USB() {
    poll_usb();
}
