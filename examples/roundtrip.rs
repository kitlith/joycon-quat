use az::CastFrom;
use bilge::prelude::*;
use joycon_quat::{compress_quaternion_triplet, Quaternion, types::*, FP30};
use nalgebra::{Unit, UnitQuaternion};
use rand::{thread_rng, Rng};


fn main() {
    let mut rng = thread_rng();
    let quat: UnitQuaternion<f32> = Unit::new_normalize(rng.gen());
    //let quat = Quaternion(res.coords.data.0[0].map(FP30::cast_from));

    let diff = UnitQuaternion::<f32>::from_axis_angle(&Unit::new_normalize(rng.gen()), 0.01);

    // macro_rules! from_debug_output {
    //     ([$(Quaternion([$($num:literal),*])),*]) => { [$(Quaternion([$(($num as f32).az()),*])),*] }
    // }

    let quats = [diff * quat, quat, quat * diff]
        .map(|q| Quaternion(q.coords.data.0[0].map(FP30::cast_from)));
    //let quats = from_debug_output!([Quaternion([0.71590066, 0.582434773, 0.384578496, 0.018848188]), Quaternion([0.715922713, 0.582408905, 0.38457495, 0.018884633]), Quaternion([0.715945482, 0.582383394, 0.384573132, 0.018848188])]);

    let timestamp = Timestamp::new(
        u11::new(rng.gen::<u16>() & u11::MASK),
        u6::new(rng.gen::<u8>() & u6::MASK),
    );
    let buf = compress_quaternion_triplet(&quats, timestamp);

    let (parsed_quats, parsed_timestamp) = Quaternion::parse(buf).unwrap();
    assert_eq!(timestamp, parsed_timestamp);

    println!("before: {:?}\nafter: {:?}", quats, parsed_quats);
}