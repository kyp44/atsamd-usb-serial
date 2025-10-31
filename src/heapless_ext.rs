//! TODO doc module and items

use heapless::{
    LenType,
    vec::{VecInner, VecStorage},
};

#[cfg(feature = "heapless")]
use heapless::string::{StringInner, StringStorage};

pub trait VecExt<T> {
    fn extend_from_slice_until_full(&mut self, other: &[T]) -> Result<(), usize>;
}

impl<T: Clone, LenT: LenType, S: VecStorage<T> + ?Sized> VecExt<T> for VecInner<T, LenT, S> {
    fn extend_from_slice_until_full(&mut self, other: &[T]) -> Result<(), usize> {
        self.extend_from_slice(other).map_err(|_| {
            // Since the slice is too large, at least copy until we are full
            let num_bytes = self.capacity() - self.len();
            let _ = self.extend_from_slice(&other[..num_bytes]);
            num_bytes
        })
    }
}

#[cfg(feature = "heapless")]
pub trait StringExt {
    fn push_str_until_full(&mut self, string: &str) -> Result<(), usize>;
    fn push_raw_buffer_until_full(&mut self, buffer: &[u8]) -> Result<bool, usize>;
}
#[cfg(feature = "heapless")]
impl<LenT: LenType, S: StringStorage + ?Sized> StringExt for StringInner<LenT, S> {
    fn push_str_until_full(&mut self, string: &str) -> Result<(), usize> {
        self.push_str(string).map_err(|_| {
            // Since the string is too large, at least copy until we are full
            let num_bytes = self.capacity() - self.len();
            let _ = self.push_str(&string[..num_bytes]);
            num_bytes
        })
    }

    // See [`UsbSerial::read_string`] for the function, except the simply the number
    // of characters copied is returned here.
    fn push_raw_buffer_until_full(&mut self, buffer: &[u8]) -> Result<bool, usize> {
        let s = str::from_utf8(&buffer)
            .unwrap_or_else(|e| str::from_utf8(&buffer[..e.valid_up_to()]).unwrap());
        self.push_str_until_full(s).map(|_| !buffer.is_empty())
    }
}

#[cfg(feature = "heapless")]
#[cfg(test)]
mod tests {
    use super::*;
    use heapless::{String, Vec};

    #[test]
    fn vec_extend_from_slice_until_full() {
        const SIZE: usize = 20;
        let mut vec: Vec<u8, SIZE> = Vec::new();
        let mut data = [0; SIZE + 1];

        // Fill the data.
        for n in 0..data.len() {
            data[n] = n as u8;
        }

        // Add data the the vec and verify we tried to copy too much
        // TODO: Verify bytes copied
        assert!(vec.extend_from_slice_until_full(&data).is_err());
        // Verify that the enough data was copied to fill the vec
        assert_eq!(vec, data[..SIZE]);

        // TODO: Verify smaller vec and num items returned
        // TODO: Verify empty vec
    }

    #[test]
    fn string_push_str_until_full() {
        const SIZE: usize = 20;
        let mut string: String<SIZE> = String::new();
        let mut data = [0u8; SIZE + 1];

        // Fill the data with the alphabet and make it a string
        for n in 0..data.len() {
            data[n] = 'A' as u8 + n as u8;
        }
        let data = str::from_utf8(&data).unwrap();

        // Add data the the string and verify we tried to copy too much
        // TODO: Verify characters copied
        assert!(string.push_str_until_full(data).is_err());
        // Verify that the enough data was copied to fill the string
        assert_eq!(string, data[..SIZE]);

        // TODO: Verify smaller string and num items returned
        // TODO: Verify empty string
    }

    // TODO
    #[test]
    fn string_push_raw_buffer_until_full() {
        // TODO: Verify everything in string_push_str_until_full
        // TODO: Verify partially valid UTF8
        panic!();
    }
}
