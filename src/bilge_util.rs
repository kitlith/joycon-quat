use az::{WrappingAs, WrappingCast};
use bilge::prelude::*;
use bitvec::macros::internal::funty::Integral;
use bitvec::{field::BitField, prelude::*};
use fixed::traits::{Fixed, FixedBits, FixedSigned};
use std::marker::PhantomData;
use std::ops::BitAnd;

// pub trait UnsignedVersion: Sized {
//     type Unsigned: WrappingCast<Self> + WrappingCastFrom<Self>;
// }

// macro_rules! impl_UnsignedVersion {
//     ($s:ident, $u:ident) => {
//         impl UnsignedVersion for $s {
//             type Unsigned = $u;
//         }
//         impl UnsignedVersion for $u {
//             type Unsigned = $u;
//         }
//     };
// }

// impl_UnsignedVersion!(i8, u8);
// impl_UnsignedVersion!(i16, u16);
// impl_UnsignedVersion!(i32, u32);
// impl_UnsignedVersion!(i64, u64);
// impl_UnsignedVersion!(i128, u128);

/// The fixed point numeric type N, whose least significant bits are serialized as Repr
#[derive(Copy, Clone, Debug)]
pub struct Fix<Repr, N>(pub N, PhantomData<Repr>);

impl<Repr: bilge::arbitrary_int::Number, N> Bitsized for Fix<Repr, N> {
    type ArbitraryInt = Repr;
    const BITS: usize = Repr::BITS;
    const MAX: Self::ArbitraryInt = Self::ArbitraryInt::MAX;
}

impl<Repr, N: FixedSigned> From<[Fix<Repr, N>; 3]> for crate::RoundedQuat3<N> {
    fn from(val: [Fix<Repr, N>; 3]) -> Self {
        crate::RoundedQuat3 {
            value: val.map(|e| e.0),
        }
    }
}

impl<Repr, N: FixedSigned> From<crate::RoundedQuat3<N>> for [Fix<Repr, N>; 3] {
    fn from(val: crate::RoundedQuat3<N>) -> Self {
        val.value.map(|e| Fix(e, PhantomData))
    }
}

impl<Repr: Number, N: Fixed> From<Repr> for Fix<Repr, N>
where
    Repr::UnderlyingType: WrappingCast<N::Bits>,
{
    fn from(repr: Repr) -> Self {
        let shift: u32 = N::Bits::BITS - Repr::BITS as u32;
        let val: N::Bits = repr.value().wrapping_as();
        // sign extend
        let val = val << shift >> shift;
        Fix(N::from_bits(val), PhantomData)
    }
}

// TODO: should really also do impls for the primitives of u8 -> u128
// for now i'm using workarounds of UInt<u8, 8>, etc
impl<N: Fixed, T: Copy + Clone + BitAnd<Output = T>, const BITS: usize> From<Fix<UInt<T, BITS>, N>>
    for UInt<T, BITS>
where
    UInt<T, BITS>: Number<UnderlyingType = T>,
    N::Bits: WrappingCast<<UInt<T, BITS> as Number>::UnderlyingType>,
{
    fn from(val: Fix<UInt<T, BITS>, N>) -> UInt<T, BITS> {
        let raw_value: <UInt<T, BITS> as Number>::UnderlyingType = val.0.to_bits().wrapping_as();
        // it'll be nice to have signed integers whenever that happens -- i'm not actually checking if it's in-range here.
        UInt::<T, BITS>::new(raw_value & UInt::<T, BITS>::MASK)
    }
}

pub trait BilgeBitvecExt: Bitsized + Sized {
    fn read<S: BitStore, B: BitOrder>(data: &BitSlice<S, B>) -> (Self, &BitSlice<S, B>)
    where
        BitSlice<S, B>: BitField,
        Self: From<<Self as Bitsized>::ArbitraryInt>,
        <Self as Bitsized>::ArbitraryInt: Number,
        <Self::ArbitraryInt as Number>::UnderlyingType: Integral,
    {
        assert!(data.len() >= Self::BITS);
        let (data, rest) = data.split_at(Self::BITS);
        let underlying = BitField::load_le(data);
        let storage = Self::ArbitraryInt::new(underlying);
        (storage.into(), rest)
    }

    fn write_vec<S: BitStore, B: BitOrder>(self, data: &mut BitVec<S, B>)
    where
        BitVec<S, B>: BitField,
        Self::ArbitraryInt: Number + From<Self>,
        <Self::ArbitraryInt as Number>::UnderlyingType: Integral,
    {
        let mut bits = bitvec!(S, B; 0; Self::BITS);
        bits.store_le(<Self::ArbitraryInt>::from(self).value());
        data.extend_from_bitslice(&bits);
    }

    fn write_slice<S: BitStore, B: BitOrder>(self, data: &mut BitSlice<S, B>) -> &mut BitSlice<S, B>
    where
        BitSlice<S, B>: BitField,
        Self::ArbitraryInt: Number + From<Self>,
        <Self::ArbitraryInt as Number>::UnderlyingType: Integral,
    {
        assert!(data.len() >= Self::BITS);
        let bits = &mut data[..Self::BITS];
        bits.store_le(<Self::ArbitraryInt>::from(self).value());
        &mut data[Self::BITS..]
    }
}

impl<T: Bitsized + Sized> BilgeBitvecExt for T {}

#[cfg(test)]
mod test {
    use super::*;

    type FP14 = fixed::types::I2F14;

    // Bilge 0.2.0 has a bug in the macro expansion which we workaround here.
    // Fix::<_, _> works, but rustfmt would reformat to Fix<_, _>, which triggers the bug.
    // Fixed upstream, but not released yet.
    #[rustfmt::skip]
    #[bitsize(47)]
    #[derive(TryFromBits)]
    struct TestStruct {
        e: TestEnum, 
        value: [Fix::<u15, FP14>; 3],
    }

    #[bitsize(2)]
    #[derive(TryFromBits)]
    enum TestEnum {
        Unfilled,
    }

    type TestFix = Fix<u12, FP14>;

    #[test]
    pub fn testfix() {
        let _ = TestFix::BITS;
        let _: TestFix = u12::new(0).into();
        let _: u12 = TestFix::from(u12::new(0)).into();
        let a: TestStruct = u47::new(0).try_into().unwrap();
        let _ = a.value();
    }
}
