use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion};
use tdoa_solver::{get_test_setup, solve};

pub fn criterion_benchmark(c: &mut Criterion) {
    let (mut entries, signal_speed, _) = get_test_setup();

    c.bench_with_input(BenchmarkId::new("Solve", 1), &1, |b, cycles| {
        b.iter(|| {
            black_box(solve(
                black_box(&mut entries),
                black_box(signal_speed),
                *cycles,
            ))
        })
    });
    c.bench_with_input(BenchmarkId::new("Solve", 10), &10, |b, cycles| {
        b.iter(|| {
            black_box(solve(
                black_box(&mut entries),
                black_box(signal_speed),
                *cycles,
            ))
        })
    });
    c.bench_with_input(BenchmarkId::new("Solve", 100), &100, |b, cycles| {
        b.iter(|| {
            black_box(solve(
                black_box(&mut entries),
                black_box(signal_speed),
                *cycles,
            ))
        })
    });
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);
