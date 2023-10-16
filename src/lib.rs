use glam::Vec3;
use itertools::Itertools;

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

pub fn solve(entries: &[TdoaEntry], signal_speed_per_second: f32) -> Vec3 {
    todo!();
}

#[doc(hidden)]
pub fn multi_hyperboloid_value(
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

#[cfg(test)]
mod tests {
    use super::*;

    const SIGNAL_SPEED: f32 = 299_792_458.0;
    const SOURCE_LOCATION: Vec3 = Vec3::new(1.126, 2.962, 8.451);

    fn get_test_setup() -> [TdoaEntry; 4] {
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

        entries
    }

    #[test]
    fn hyperboloid_value_test() {
        let entries = get_test_setup();

        approx::assert_abs_diff_eq!(
            multi_hyperboloid_value(SOURCE_LOCATION, &entries, SIGNAL_SPEED),
            0.0,
            epsilon = 0.000001
        );
        approx::assert_abs_diff_eq!(
            multi_hyperboloid_value(
                SOURCE_LOCATION + Vec3::new(0.0, 0.1, 0.0),
                &entries,
                SIGNAL_SPEED
            ),
            0.0,
            epsilon = 0.000001
        );
    }

    #[test]
    fn solve_test() {
        let entries = get_test_setup();

        let solved_location = solve(&entries, SIGNAL_SPEED);

        approx::assert_abs_diff_eq!(
            solved_location.distance(SOURCE_LOCATION),
            0.0,
            epsilon = 0.1
        );
    }
}
