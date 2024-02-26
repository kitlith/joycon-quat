//! The data format is as follows:
//!  - 2 bits for the Quaternion mode
//!  - 125-127 bits for a bitfield that depends on the mode indicated
//!  - 17 bits for a couple fields dealing with timestamps.
//! Values are read from bytes starting from the least significant bit,
//! and values split across bytes are read in little-endian order.

use super::bilge_util::Fix;
use bilge::prelude::*;

//type FP30 = fixed::types::I2F30;
pub(crate) type FP20 = fixed::types::I12F20;
pub(crate) type FP15 = fixed::types::I17F15;
pub(crate) type FP12 = fixed::types::I20F12;

#[bitsize(2)]
#[derive(FromBits, Copy, Clone, Debug)]
pub(crate) enum QuaternionMode {
    Individual,
    FirstLastDeltaMid,
    LastDeltaFirstDeltaMid,
    Unknown,
}

#[bitsize(123)]
#[derive(FromBits, DebugBits)]
pub(crate) struct Individual(pub [IndividualCompress; 3]);

// Bilge 0.2.0 has a bug in the macro expansion which we workaround here.
// Fix::<_, _> works, but rustfmt would reformat to Fix<_, _>, which triggers the bug.
// Fixed upstream, but not released yet.
#[rustfmt::skip]
#[bitsize(41)]
#[derive(FromBits, Copy, Clone, DebugBits)]
pub(crate) struct IndividualCompress {
    missing_idx: u2,
    components: [Fix::<u13, FP12>; 3],
}

impl IndividualCompress {
    pub fn pack(missing_idx: u2, data: crate::RoundedQuat3<FP12>) -> Self {
        Self::new(missing_idx, data.into())
    }

    pub fn unpack(self) -> (u2, crate::RoundedQuat3<FP12>) {
        (self.missing_idx(), self.components().into())
    }
}

// workaround for missing primitive impls
type B16 = UInt<u16, 16>;
type B8 = UInt<u8, 8>;

// Bilge 0.2.0 has a bug in the macro expansion which we workaround here.
// Fix::<_, _> works, but rustfmt would reformat to Fix<_, _>, which triggers the bug.
// Fixed upstream, but not released yet.
#[rustfmt::skip]
#[bitsize(123)]
#[derive(FromBits, DebugBits, Clone)]
pub(crate) struct FirstLastDeltaMid {
    /// indicates if mid_avg_delta was shifted down by an additional 2 bits and needs to be shifted back up
    additional_shift: bool,
    missing_idx: u2,
    first: [Fix::<B16, FP15>; 3],
    last: [Fix::<B16, FP15>; 3],
    mid_avg_delta: [Fix::<B8, FP15>; 3],
}

#[test]
fn confirm() {
    // TODO: test against FP3::fits_within directly
    let fits = |val: i32, bits: u32| (val << 32 - bits) >> (32 - bits) == val;
    assert!(fits(511, 10));
    assert!(!fits(512, 10));
    assert!(fits(-512, 10));
    assert!(!fits(-513, 10));

    assert!(fits(127, 8));
    assert!(!fits(128, 8));
    assert!(fits(-128, 8));
    assert!(!fits(-129, 8));
}

impl FirstLastDeltaMid {
    pub fn pack(missing_idx: u2, data: [crate::RoundedQuat3<FP15>; 3]) -> Option<Self> {
        let [first, mid, last] = data;

        let avg = last.avg(&first);
        let mut mid_avg_delta = mid - avg;

        if mid_avg_delta.fits_within(10) {
            let additional_shift = !mid_avg_delta.fits_within(8);
            if additional_shift {
                for elem in &mut mid_avg_delta.value {
                    *elem >>= 2;
                }
            }

            let res = FirstLastDeltaMid::new(
                additional_shift,
                missing_idx,
                first.into(),
                last.into(),
                mid_avg_delta.into(),
            );
            debug_assert_eq!((missing_idx, data), res.clone().unpack());
            Some(res)
        } else {
            None
        }
    }

    pub fn unpack(self) -> (u2, [crate::RoundedQuat3<FP15>; 3]) {
        let first = crate::RoundedQuat3::from(self.first());
        let last = crate::RoundedQuat3::from(self.last());
        let avg = last.avg(&first);
        let mut mid_avg_delta = crate::RoundedQuat3::from(self.mid_avg_delta());
        if self.additional_shift() {
            for elem in &mut mid_avg_delta.value {
                *elem <<= 2;
            }
        }

        // mid_avg_delta = mid - avg
        // mid_avg_delta + avg = mid
        let mid = avg + mid_avg_delta;

        (self.missing_idx(), [first, mid, last])
    }
}

// Bilge 0.2.0 has a bug in the macro expansion which we workaround here.
// Fix::<_, _> works, but rustfmt would reformat to Fix<_, _>, which triggers the bug.
// Fixed upstream, but not released yet.
#[rustfmt::skip]
#[bitsize(125)]
#[derive(FromBits, DebugBits, Clone)]
pub(crate) struct LastDeltaFirstDeltaMid {
    missing_idx: u2,
    last: [Fix::<u21, FP20>; 3],
    first_delta: [Fix::<u13, FP20>; 3],
    mid_avg_delta: [Fix::<u7, FP20>; 3],
}

impl LastDeltaFirstDeltaMid {
    pub fn pack(missing_idx: u2, data: [crate::RoundedQuat3<FP20>; 3]) -> Option<Self> {
        let [first, mid, last] = data;

        let first_delta = last - first;
        let avg = last.avg(&first);
        let mid_avg_delta = mid - avg; // delta_mid = mid - avg

        if first_delta.fits_within(13) && mid_avg_delta.fits_within(7) {
            let res = LastDeltaFirstDeltaMid::new(
                missing_idx,
                last.into(),
                first_delta.into(),
                mid_avg_delta.into(),
            );
            debug_assert_eq!((missing_idx, data), res.clone().unpack());
            Some(res)
        } else {
            None
        }
    }

    pub fn unpack(self) -> (u2, [crate::RoundedQuat3<FP20>; 3]) {
        let last = crate::RoundedQuat3::from(self.last());
        let first_delta = crate::RoundedQuat3::from(self.first_delta());
        let mid_avg_delta = crate::RoundedQuat3::from(self.mid_avg_delta());

        let first = last - first_delta;

        let avg = last.avg(&first);
        let mid = avg + mid_avg_delta;

        (self.missing_idx(), [first, mid, last])
    }
}

#[bitsize(17)]
#[derive(DebugBits, PartialEq, FromBits, Copy, Clone)]
pub struct Timestamp {
    start: u11,
    count: u6,
}
