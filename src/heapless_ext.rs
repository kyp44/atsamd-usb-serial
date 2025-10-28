//! TODO

use heapless::{CapacityError, Vec};

pub trait VecExt<T> {
    fn extend_from_slice_until_full(&mut self, other: &[T]) -> Result<(), CapacityError>;
}

impl<T: Clone, const N: usize> VecExt<T> for Vec<T, N> {
    fn extend_from_slice_until_full(&mut self, other: &[T]) -> Result<(), CapacityError> {
        self.extend_from_slice(other).inspect_err(|_| {
            // Since the slice is too large, at last copy until we are full
            self.extend_from_slice(&other[..self.capacity() - self.len()])
                .unwrap();
        })
    }
}
