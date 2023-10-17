use glam::Vec3;
use itertools::Itertools;
use rand::Rng;

#[derive(Debug, Clone, Copy)]
pub struct TdoaEntry {
    /// The location of the anchor that received the message for this entry
    pub location: Vec3,
    /// The time difference to the first receiver in nanoseconds.
    /// If this entry is from the first receiver, then this should be 0.
    ///
    /// This is in nanoseconds to aid the precision of the float.
    pub time_difference: f32,
}

impl TdoaEntry {
    pub fn new(location: Vec3, time_difference: f32) -> Self {
        Self {
            location,
            time_difference,
        }
    }
}

pub fn solve(
    entries: &[TdoaEntry],
    signal_speed_per_second: f32,
    cycles: usize,
) -> Vec<(Vec3, f32)> {
    assert!(!entries.is_empty(), "The entry list may not be empty");

    let test_locations = entries.iter().map(|entry| entry.location).chain(
        entries
            .iter()
            .tuple_combinations()
            .map(|(a, b)| (a.location + b.location) / 2.0),
    );

    let num_locations = test_locations.clone().count();
    let test_locations = test_locations.cycle().take(num_locations * cycles);

    let found_locations = test_locations.map(|test_location| {
        let mut current_point = test_location;
        let mut current_value =
            multi_hyperboloid_value(current_point, entries, signal_speed_per_second);

        let mut last_change_iteration = 0;

        for iteration in 0..10000 {
            let direction = random_unit_vector();

            let mut step_size = 1.0;

            while step_size > 0.001 {
                let next_point = current_point + direction * step_size;
                let next_value =
                    multi_hyperboloid_value(next_point, entries, signal_speed_per_second);

                if next_value < current_value {
                    current_point = next_point;
                    current_value = next_value;
                    last_change_iteration = iteration;
                } else {
                    step_size /= 2.0;
                }
            }

            if iteration - last_change_iteration > 100 {
                break;
            }
        }

        (current_point, current_value)
    });

    let mut location_clusters: Vec<Vec<(Vec3, f32)>> = Vec::new();

    'location_loop: for (found_location, found_value) in found_locations {
        for cluster in location_clusters.iter_mut() {
            let distance_to_cluster = cluster[0].0.distance(found_location);

            if distance_to_cluster < 0.1 {
                cluster.push((found_location, found_value));
                continue 'location_loop;
            }
        }

        // Not already near a cluster, so create a new one
        location_clusters.push(vec![(found_location, found_value)]);
    }

    // Locations with a weight
    let mut location_clusters: Vec<(Vec3, f32)> = location_clusters
        .into_iter()
        .map(|cluster| {
            let cluster_len = cluster.len();
            let (location_sum, value_sum) = cluster
                .into_iter()
                .reduce(|acc, next| (acc.0 + next.0, acc.1 + next.1))
                .unwrap();
            (
                location_sum / cluster_len as f32,
                (value_sum / cluster_len as f32 / (cluster_len as f32).powi(2)).recip(),
            )
        })
        .collect();

    let avg_weight = location_clusters
        .iter()
        .map(|cluster| cluster.1)
        .sum::<f32>()
        / location_clusters.len() as f32;
    location_clusters.retain(|cluster| cluster.1 > avg_weight);

    location_clusters.sort_by(|l, r| l.1.total_cmp(&(r.1 as f32)).reverse());
    location_clusters
}

fn random_unit_vector() -> Vec3 {
    let mut rng = rand::thread_rng();

    let theta = 2.0 * std::f32::consts::PI * rng.gen::<f32>();
    let phi = (2.0 * rng.gen::<f32>() - 1.0).acos();
    let sin_phi = phi.sin();

    Vec3::new(theta.cos() * sin_phi, theta.sin() * sin_phi, phi.cos())
}

fn multi_hyperboloid_value(
    test_location: Vec3,
    entries: &[TdoaEntry],
    signal_speed_per_second: f32,
) -> f32 {
    let mut acc = 0.0;

    for (l, r) in entries.iter().tuple_combinations() {
        acc += hyperboloid_value(test_location, l, r, signal_speed_per_second).abs();
    }

    acc
}

#[inline(always)]
fn hyperboloid_value(
    test_location: Vec3,
    l: &TdoaEntry,
    r: &TdoaEntry,
    signal_speed_per_second: f32,
) -> f32 {
    let l_distance = test_location.distance(l.location);
    let r_distance = test_location.distance(r.location);

    let distance_difference = (l_distance - r_distance).abs();
    let signal_distance =
        (l.time_difference - r.time_difference).abs() * (signal_speed_per_second / 1_000_000_000.0);

    distance_difference - signal_distance
}

#[doc(hidden)]
pub fn get_test_setup() -> ([TdoaEntry; 4], f32, Vec3) {
    let mut entries = [
        TdoaEntry::new(Vec3::new(0.0, 5.0, 0.0), 0.0),
        TdoaEntry::new(Vec3::new(10.0, 0.0, 0.0), 0.0),
        TdoaEntry::new(Vec3::new(10.0, 10.0, 10.0), 0.0),
        TdoaEntry::new(Vec3::new(0.0, 0.0, 10.0), 0.0),
    ];

    let signal_speed = rand::thread_rng().gen_range(1.0..=300_000_000.0);
    let source_location = Vec3::new(
        rand::thread_rng().gen_range(0.0..=10.0),
        rand::thread_rng().gen_range(0.0..=10.0),
        rand::thread_rng().gen_range(0.0..=10.0),
    );

    for entry in entries.iter_mut() {
        entry.time_difference =
            entry.location.distance(source_location) * 1_000_000_000.0 / signal_speed;
    }

    let lowest_td = entries
        .iter()
        .map(|entry| entry.time_difference)
        .min_by(|a, b| a.total_cmp(&b))
        .unwrap();

    for entry in entries.iter_mut() {
        entry.time_difference -= lowest_td;
    }

    (entries, signal_speed, source_location)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hyperboloid_value_test() {
        let (entries, signal_speed, source_location) = get_test_setup();

        approx::assert_abs_diff_eq!(
            multi_hyperboloid_value(source_location, &entries, signal_speed),
            0.0,
            epsilon = 0.001
        );
        approx::assert_abs_diff_ne!(
            multi_hyperboloid_value(
                source_location + Vec3::new(0.0, 0.1, 0.0),
                &entries,
                signal_speed
            ),
            0.0,
            epsilon = 0.001
        );
    }

    #[test]
    fn solve_test() {
        for _ in 0..50 {
            let (entries, signal_speed, source_location) = get_test_setup();

            println!("Testing setup with location: {source_location}, signal_speed: {signal_speed}");

            let solved_location = solve(&entries, signal_speed, 10);

            let any_correct_location = solved_location.iter().any(|(location, weight)| {
                let distance = location.distance(source_location);
                println!("location: {location}, weight: {weight}, distance: {distance}");

                distance < 1.0
            });

            assert!(any_correct_location);
        }
    }
}
