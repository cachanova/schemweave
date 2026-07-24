use schemweave::Layout;

pub fn layout_digest(layout: &Layout) -> u64 {
    const FNV_OFFSET_BASIS: u64 = 14_695_981_039_346_656_037;
    const FNV_PRIME: u64 = 1_099_511_628_211;

    serde_json::to_vec(layout)
        .expect("layout must serialize")
        .into_iter()
        .fold(FNV_OFFSET_BASIS, |digest, byte| {
            (digest ^ u64::from(byte)).wrapping_mul(FNV_PRIME)
        })
}
