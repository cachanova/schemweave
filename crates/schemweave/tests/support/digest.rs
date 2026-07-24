use std::io::{self, Write};

use schemweave::Layout;

pub fn layout_digest(layout: &Layout) -> u64 {
    const FNV_OFFSET_BASIS: u64 = 14_695_981_039_346_656_037;
    const FNV_PRIME: u64 = 1_099_511_628_211;

    struct FnvWriter(u64);

    impl Write for FnvWriter {
        fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
            for &byte in bytes {
                self.0 = (self.0 ^ u64::from(byte)).wrapping_mul(FNV_PRIME);
            }
            Ok(bytes.len())
        }

        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }

    let mut writer = FnvWriter(FNV_OFFSET_BASIS);
    serde_json::to_writer(&mut writer, layout).expect("layout must serialize");
    writer.0
}
