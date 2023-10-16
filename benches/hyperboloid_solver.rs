use criterion::{black_box, criterion_group, criterion_main, Criterion};
use glam::Vec3;
use tdoa_solver::{multi_hyperboloid_value, TdoaEntry};

pub fn criterion_benchmark(c: &mut Criterion) {
    const SIGNAL_SPEED: f32 = 299_792_458.0;
    const SOURCE_LOCATION: Vec3 = Vec3::new(2.5, 2.5, 2.5);

    let mut entries = [
        TdoaEntry::new(Vec3::new(0.0, 0.0, 0.0), 0.0),
        TdoaEntry::new(Vec3::new(10.0, 0.0, 0.0), 0.0),
        TdoaEntry::new(Vec3::new(10.0, 10.0, 10.0), 0.0),
        TdoaEntry::new(Vec3::new(0.0, 0.0, 10.0), 0.0),
    ];

    for entry in entries.iter_mut() {
        entry.time_difference =
            entry.location.distance(SOURCE_LOCATION) * 1_000_000_000.0 / SIGNAL_SPEED;
    }

    let lowest_td = entries
        .iter()
        .map(|entry| entry.time_difference)
        .min_by(|a, b| a.total_cmp(&b))
        .unwrap();

    for entry in entries.iter_mut() {
        entry.time_difference -= lowest_td;
    }

    c.bench_function("Hyperboloid value", |b| {
        b.iter(|| {
            multi_hyperboloid_value(
                black_box(Vec3::new(1.0, 1.0, 1.0)),
                black_box(&entries),
                SIGNAL_SPEED,
            )
        })
    });
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);
