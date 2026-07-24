mod support;

use criterion::{Criterion, criterion_group, criterion_main};

fn smoke(c: &mut Criterion) {
    c.bench_function("smoke/noop", |b| b.iter(|| std::hint::black_box(1 + 1)));
}

criterion_group!(benches, smoke);
criterion_main!(benches);
