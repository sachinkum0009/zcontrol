// Copyright 2026 Sachin Kumar
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! Z Control wire format.
//!
//! The host side is `zenbedded_hardware_interface`. It reads an
//! `interface_schema.yaml` and expects every state/command payload to be the
//! listed fields packed back to back (no padding, no header), little-endian, in
//! the order they appear in the YAML: components in file order, fields in file
//! order within each component.
//!
//! On the device the same layout is written by hand with [`packed_struct!`]:
//!
//! ```
//! zc_proto::packed_struct! {
//!     /// state_interfaces of interface_schema.yaml
//!     pub struct State {
//!         pub left_wheel_position: f64,
//!         pub left_wheel_velocity: f64,
//!     }
//! }
//!
//! use zc_proto::Packed;
//! assert_eq!(State::SIZE, 16);
//! ```
//!
//! Only the field types the host schema parser knows are allowed:
//!
//! | schema YAML | Rust  |
//! |-------------|-------|
//! | `float32`   | `f32` |
//! | `float64` / `double` | `f64` |
//! | `int32`     | `i32` |
//! | `uint64`    | `u64` |
//! | `int16`     | `i16` |
//! | `uint8`     | `u8`  |

#![no_std]

/// A primitive that may appear in an interface schema.
pub trait Field: Copy + Default {
    /// Size on the wire in bytes.
    const SIZE: usize;

    /// Writes `self` little-endian into `out[..Self::SIZE]`.
    fn put(self, out: &mut [u8]);

    /// Reads a value little-endian from `buf[..Self::SIZE]`.
    fn get(buf: &[u8]) -> Self;
}

macro_rules! impl_field {
    ($($t:ty),*) => {$(
        impl Field for $t {
            const SIZE: usize = core::mem::size_of::<$t>();

            #[inline]
            fn put(self, out: &mut [u8]) {
                out[..Self::SIZE].copy_from_slice(&self.to_le_bytes());
            }

            #[inline]
            fn get(buf: &[u8]) -> Self {
                let mut bytes = [0u8; core::mem::size_of::<$t>()];
                bytes.copy_from_slice(&buf[..Self::SIZE]);
                <$t>::from_le_bytes(bytes)
            }
        }
    )*};
}

impl_field!(f32, f64, i32, u64, i16, u8);

/// A state or command payload with a fixed packed layout.
pub trait Packed: Sized {
    /// Payload size in bytes. Must equal the host's `state_buffer_size()` /
    /// `command_buffer_size()` for the same schema.
    const SIZE: usize;

    /// Writes the payload into `out[..Self::SIZE]`.
    ///
    /// # Panics
    /// If `out` is shorter than [`Self::SIZE`].
    fn encode(&self, out: &mut [u8]);

    /// Decodes a payload. Returns `None` if `buf.len() != Self::SIZE`, the same
    /// rule the host applies to state payloads.
    fn decode(buf: &[u8]) -> Option<Self>;
}

/// Declares a plain struct and implements [`Packed`] for it.
///
/// Fields are encoded in declaration order, which must match the order of the
/// schema YAML. Field names are free, but `<component>_<interface>` mirrors the
/// C header that `zenbedded_transport` generates.
#[macro_export]
macro_rules! packed_struct {
    (
        $(#[$meta:meta])*
        $vis:vis struct $name:ident {
            $(
                $(#[$fmeta:meta])*
                $fvis:vis $field:ident : $ty:ty
            ),* $(,)?
        }
    ) => {
        $(#[$meta])*
        #[derive(Clone, Copy, Debug, Default, PartialEq)]
        $vis struct $name {
            $(
                $(#[$fmeta])*
                $fvis $field: $ty,
            )*
        }

        impl $crate::Packed for $name {
            const SIZE: usize = 0 $(+ <$ty as $crate::Field>::SIZE)*;

            #[allow(unused_assignments, unused_variables, unused_mut)]
            fn encode(&self, out: &mut [u8]) {
                let out = &mut out[..Self::SIZE];
                let mut offset = 0usize;
                $(
                    $crate::Field::put(self.$field, &mut out[offset..]);
                    offset += <$ty as $crate::Field>::SIZE;
                )*
            }

            #[allow(unused_assignments, unused_variables, unused_mut)]
            fn decode(buf: &[u8]) -> Option<Self> {
                if buf.len() != Self::SIZE {
                    return None;
                }
                let mut offset = 0usize;
                $(
                    let $field = <$ty as $crate::Field>::get(&buf[offset..]);
                    offset += <$ty as $crate::Field>::SIZE;
                )*
                Some(Self { $($field),* })
            }
        }
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    packed_struct! {
        struct Mixed {
            a: f32,
            b: f64,
            c: i32,
            d: u64,
            e: i16,
            f: u8,
        }
    }

    packed_struct! {
        struct Empty {}
    }

    #[test]
    fn size_is_sum_of_fields_without_padding() {
        assert_eq!(Mixed::SIZE, 4 + 8 + 4 + 8 + 2 + 1);
        assert_eq!(Empty::SIZE, 0);
    }

    #[test]
    fn layout_matches_pragma_pack_1_little_endian() {
        let m = Mixed {
            a: 1.5,
            b: -2.25,
            c: -7,
            d: 0x0102_0304_0506_0708,
            e: -2,
            f: 0xAB,
        };
        let mut buf = [0u8; 27];
        m.encode(&mut buf);

        let mut expected = [0u8; 27];
        expected[0..4].copy_from_slice(&1.5f32.to_le_bytes());
        expected[4..12].copy_from_slice(&(-2.25f64).to_le_bytes());
        expected[12..16].copy_from_slice(&(-7i32).to_le_bytes());
        expected[16..24].copy_from_slice(&0x0102_0304_0506_0708u64.to_le_bytes());
        expected[24..26].copy_from_slice(&(-2i16).to_le_bytes());
        expected[26] = 0xAB;
        assert_eq!(buf, expected);

        assert_eq!(Mixed::decode(&buf), Some(m));
    }

    #[test]
    fn decode_rejects_wrong_size() {
        assert_eq!(Mixed::decode(&[0u8; 26]), None);
        assert_eq!(Mixed::decode(&[0u8; 28]), None);
    }
}
