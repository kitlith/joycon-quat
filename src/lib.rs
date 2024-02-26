mod bilge_util;
pub mod types;

use std::ops::{Add, Sub};

use bilge::prelude::*;
use bilge_util::BilgeBitvecExt;
use bitvec::{array::BitArray, order::Lsb0};
use fixed::traits::{FixedBits, FixedSigned};
use fixed_sqrt::FixedSqrt;
use types::*;

pub type FP30 = fixed::types::I2F30;

// TODO: interop
#[derive(Debug, Default, Clone)]
pub struct Quaternion(pub [FP30; 4]);

impl Quaternion {
    // QuaternionFp30MaxAbsQfp
    fn max_abs_idx(&self) -> usize {
        let mut index = 0;
        let mut max = self.0[0].abs();
        for i in 1..=3 {
            let val = self.0[i].abs();
            if val > max {
                index = i;
                max = val;
            }
        }
        return index;
    }

    // QuaternionFp30ConjugateQfp
    fn to_quat3(&self, max_idx: usize) -> Quat3 {
        let sign = self.0[max_idx].signum();
        Quat3(std::array::from_fn(|i| {
            self.0[(i + 1 + max_idx) & 3] * sign
        }))
    }

    fn reconstruct(item: Quat3, missing_idx: usize) -> Quaternion {
        let mut sqr_sum = FP30::ONE;
        let mut out = Quaternion::default();
        for i in 1..=3 {
            out.0[(i + missing_idx) & 3] = item.0[i - 1];
            sqr_sum -= item.0[i - 1] * item.0[i - 1];
        }
        //    a**2 + b**2 + c**2 + d**2 = 1
        // => a**2 + b**2 + c**2 - 1 = -d**2
        // => sqrt(1 - a**2 - b**2 - c**2) = d
        // TODO: compare cordic and fixed_sqrt at some point? or just do this in floating point
        out.0[missing_idx] = sqr_sum.sqrt();

        out
    }

    /// Decodes quaternions from joycons.
    ///
    /// The 18 byte input is made by concatenating the 3 6 byte slots used for gyro output
    /// in non-quaternion modes.
    pub fn parse(bytes: [u8; 18]) -> Option<([Quaternion; 3], types::Timestamp)> {
        let bits = BitArray::<_, Lsb0>::new(bytes);
        let bits = bits.as_bitslice();
        let (mode, mut bits) = QuaternionMode::read(bits);
        let quats = match mode {
            QuaternionMode::Individual => {
                let (data, rest) = Individual::read(bits);
                bits = rest;
                data.val_0().map(|q| {
                    let (idx, components) = q.unpack();
                    let idx = idx.value() as usize;
                    Self::reconstruct(components.into(), idx)
                })
            }
            QuaternionMode::FirstLastDeltaMid => {
                let (data, rest) = FirstLastDeltaMid::read(bits);
                bits = rest;
                let (idx, data) = data.unpack();
                let idx = idx.value() as usize;
                data.map(|q| Self::reconstruct(q.into(), idx))
            }
            QuaternionMode::LastDeltaFirstDeltaMid => {
                let (data, rest) = LastDeltaFirstDeltaMid::read(bits);
                bits = rest;
                let (idx, data) = data.unpack();
                let idx = idx.value() as usize;
                data.map(|q| Self::reconstruct(q.into(), idx))
            }
            QuaternionMode::Unknown => {
                return None;
            }
        };

        let (timestamp, _) = Timestamp::read(bits);

        Some((quats, timestamp))
    }
}

// Quat3, RoundedQuat3, and the implicit missing index effectively implement the technique from quatcompress: <https://github.com/jpreiss/quatcompress>

// implicit information: index of the missing element
#[derive(Copy, Clone, Default, Debug)]
struct Quat3([FP30; 3]);

// consider merging into Quat3, though the semantic difference may be useful.
// (Quat3 is essentially "full quality", and FP3 handles lower precision values)
#[derive(Default, Copy, Clone, Debug, PartialEq)]
struct RoundedQuat3<N: FixedSigned> {
    value: [N; 3],
}

impl<N: FixedSigned> From<Quat3> for RoundedQuat3<N> {
    // QuaternionFp30ConstructFP3FromQuaternion
    fn from(val: Quat3) -> Self {
        // the largest value that is too small to be represented in the new type
        let epsilon = FP30::ONE >> (N::FRAC_NBITS + 1);
        let value = val.0.map(|c| {
            // rounding away from 0: increment the first bit that's going to be truncated away
            let val = c + c.signum() * epsilon;
            // truncate
            val.to_num()
        });

        Self { value }
    }
}

impl<N: FixedSigned> From<RoundedQuat3<N>> for Quat3 {
    fn from(val: RoundedQuat3<N>) -> Quat3 {
        Quat3(val.value.map(N::to_num))
    }
}

impl<N: FixedSigned> Add for RoundedQuat3<N> {
    type Output = RoundedQuat3<N>;

    fn add(self, rhs: Self) -> Self::Output {
        RoundedQuat3 {
            value: std::array::from_fn(|i| self.value[i] + rhs.value[i]),
        }
    }
}

impl<N: FixedSigned> Sub for RoundedQuat3<N> {
    type Output = RoundedQuat3<N>;

    fn sub(self, rhs: Self) -> Self::Output {
        RoundedQuat3 {
            value: std::array::from_fn(|i| self.value[i] - rhs.value[i]),
        }
    }
}

impl<N: FixedSigned> RoundedQuat3<N> {
    fn avg(&self, other: &RoundedQuat3<N>) -> RoundedQuat3<N> {
        let mut res = RoundedQuat3::default();

        for i in 0..3 {
            let val = self.value[i] + other.value[i];
            res.value[i] = (val + (val.signum() * N::DELTA)) >> 1;
        }

        res
    }

    // QuaternionFp30HasFP3Bit
    fn fits_within(&self, bits: usize) -> bool {
        // returns true if the number can roundtrip through that number of bits
        let fits =
            |val| (val << N::Bits::BITS - bits as u32) >> (N::Bits::BITS - bits as u32) == val;

        fits(self.value[0]) && fits(self.value[1]) && fits(self.value[2])
    }
}

// QuaternionFp30EncodeUnitQuatTriplet
/// Takes 3 Quaternions and a Timestamp and encodes them like a joycon would.
///
/// Useful for roundtrip testing or potentially for pretending to be a joycon.
pub fn compress_quaternion_triplet(input: &[Quaternion; 3], timestamp: Timestamp) -> [u8; 18] {
    let mut res = BitArray::<_, Lsb0>::new([0u8; 18]);
    let mut stream = res.as_mut_bitslice();

    let max_idx = input[1].max_abs_idx();

    let working_quat3 = input.clone().map(|q| q.to_quat3(max_idx));

    let mut written = false;
    let missing_idx = u2::new(max_idx as u8);

    // replaces (1 << 29), a number with the first fractional bit set, aka 1/2 in a 30 fractional bit system.
    let half: FP30 = FP30::ONE >> 1;

    if let Some(data) = LastDeltaFirstDeltaMid::pack(missing_idx, working_quat3.map(<_>::into)) {
        stream = QuaternionMode::LastDeltaFirstDeltaMid.write_slice(stream);
        stream = data.write_slice(stream);
        written = true;
    } else if input[0].0[max_idx].abs() > half && input[2].0[max_idx] > half {
        if let Some(data) = FirstLastDeltaMid::pack(missing_idx, working_quat3.map(<_>::into)) {
            stream = QuaternionMode::FirstLastDeltaMid.write_slice(stream);
            stream = data.write_slice(stream);
            written = true;
        }
    }

    if !written {
        // QuaternionFp30Unk
        stream = QuaternionMode::Individual.write_slice(stream);
        stream = Individual::new(input.clone().map(|q| {
            let max_idx = q.max_abs_idx();
            let missing_idx = max_idx as u8;

            let quat3 = q.to_quat3(max_idx);
            let rounded: RoundedQuat3<FP12> = quat3.into();
            IndividualCompress::pack(u2::new(missing_idx), rounded.into())
        }))
        .write_slice(stream)
    }

    timestamp.write_slice(stream);

    res.data
}
