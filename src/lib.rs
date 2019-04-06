//! Missile guidance helpers
//!
//! # References
//! https://nptel.ac.in/courses/101108056/9

use na::RealField;

#[derive(Debug, Copy, Clone)]
pub struct Target<N: RealField> {
    pub position: na::Vector3<N>,
    pub velocity: na::Vector3<N>,
}

impl<N: RealField> Target<N> {
    /// Whether the target is currently approaching the origin
    ///
    /// Shorthand for `position . velocity < 0`.
    #[inline]
    pub fn is_closing(&self) -> bool {
        self.position.dot(&self.velocity) < na::zero()
    }
}

/// Ideal Proportional Navigation
///
/// Returns the desired instantaneous acceleration vector.
///
/// `target.is_closing()` must be true.
pub fn ipn<N: RealField>(navigation_constant: N, target: &Target<N>) -> na::Vector3<N> {
    debug_assert!(target.is_closing());
    let w_s = target.position.cross(&target.velocity) / target.position.norm_squared();
    (target.velocity * navigation_constant).cross(&w_s)
}

/// Direction to aim a projectile that will travel at `speed` to hit `target` and time of impact
pub fn linear_aim<N: RealField>(
    target: &Target<N>,
    speed: N,
) -> Option<(na::Unit<na::Vector3<N>>, N)> {
    // t^2 * -s^2 + t * (2.0 * v_T·p_T) + p_T·p_T = 0
    let a = target.velocity.norm_squared() - (speed * speed);
    if a.abs() < na::convert(1e-3) {
        return Some((na::Unit::new_normalize(target.position), na::zero()));
    }
    let b: N = na::convert::<_, N>(2.) * target.position.dot(&target.velocity);
    let c = target.position.norm_squared();
    let rt = (b * b - na::convert::<_, N>(4.) * a * c).try_sqrt()?;
    let t0 = (-b + rt) / (na::convert::<_, N>(2.) * a);
    let t1 = (-b - rt) / (na::convert::<_, N>(2.) * a);
    let t = [t0, t1]
        .iter()
        .cloned()
        .filter(|&x| x >= na::zero())
        .min_by(|x, y| x.partial_cmp(y).unwrap())
        .unwrap();
    let pos = target.position + target.velocity * t;
    Some((na::Unit::new_normalize(pos), t))
}

/// Change in velocity to steer an in-flight projectile towards `target`
pub fn linear_steer<N: RealField>(target: &Target<N>, current_velocity: &na::Vector3<N>, average_speed: N) -> Option<(na::Vector3<N>, N)> {
    let target = Target {
        position: target.position,
        velocity: target.velocity + current_velocity,
    };
    let current_speed = current_velocity.norm();
    let (dir, t) = linear_aim(&target, average_speed)?;
    let goal = dir.into_inner() * current_speed;
    Some((goal - current_velocity, t))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn linear() {
        assert!(
            miss(Target {
                position: na::Vector3::new(0.0, 0.0, -10.0),
                velocity: na::Vector3::new(0.0, 0.0, 1.0),
            }) < 1.0
        );
    }

    #[test]
    fn deflection() {
        assert!(
            miss(Target {
                position: na::Vector3::new(0.0, -1.0, -10.0),
                velocity: na::Vector3::new(0.0, 1.0, 0.1),
            }) < 1.0
        );
    }

    #[test]
    fn behind() {
        assert!(
            miss(Target {
                position: na::Vector3::new(0.0, 0.0, 10.0),
                velocity: na::Vector3::new(0.0, 1.0, -1.0),
            }) < 1.0
        );
    }

    /// Find the miss distance
    fn miss(mut target: Target<f64>) -> f64 {
        const TIMESTEP: f64 = 1e-2;
        while target.is_closing() {
            // Semi-implicit euler integration
            let acceleration = ipn(3.0, &target);
            assert!(acceleration.dot(&target.velocity).abs() < 1e-3);
            target.velocity += TIMESTEP * -acceleration;
            target.position += TIMESTEP * target.velocity;
            println!(
                "x: {:?}; v: {:?}; a: {:?}",
                target.position.data,
                (-target.velocity).data,
                acceleration.data
            );
        }
        let distance = target.position.norm();
        println!("miss: {}", distance);
        distance
    }
}
