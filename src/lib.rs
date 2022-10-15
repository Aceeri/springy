use bevy::prelude::*;
use bevy_inspector_egui::Inspectable;

pub mod prelude {
    #[cfg(any(feature = "rapier2d", feature = "rapier3d"))]
    pub use crate::rapier::RapierParticleQuery;
    pub use crate::{Particle, Spring};
}

#[cfg(any(feature = "rapier2d", feature = "rapier3d"))]
pub mod rapier;
#[cfg(any(feature = "rapier2d", feature = "rapier3d"))]
pub use rapier::RapierParticleQuery;

#[derive(Default, Debug, Copy, Clone, Component, Reflect, Inspectable)]
#[reflect(Component)]
pub struct Spring {
    /// Strength of the spring-like impulse. This is a range between 0 and 1
    /// where 1 will bring the spring to equilibrium in 1 timestep.
    #[inspectable(min = 0.0, max = 1.0, speed = 0.01)]
    pub strength: f32,
    /// Damping of the spring-like impulse. <1 will be treated as under-dampened
    /// and will overshoot the target, >=1 will be treated as critically dampened
    /// and shouldn't overshoot the target.
    ///
    /// Note this will not be completely respected to avoid instability in the spring.
    /// So overshooting *may* happen if you have a really high strength value.
    #[inspectable(min = 0.0, max = 4.0, speed = 0.05)]
    pub damp_ratio: f32,
    /// Rest distance around the particle, it will try to push the particle out
    /// when too close.
    #[inspectable(min = 0.0)]
    pub rest_distance: f32,
    /// Similar to rest distance except it will not push outwards if it is too close.
    #[inspectable(min = 0.0)]
    pub limp_distance: f32,
}

#[derive(Default, Debug)]
pub struct Particle<S>
where
    S: Springable,
{
    /// Mass of the particle, set to 0.0 or f32::INFINITY to set to `static`.
    pub mass: f32,
    /// Translation of the particle.
    pub position: S,
    /// Current velocity of the particle.
    pub velocity: S,
}

pub trait Springable:
    std::ops::Sub<Self, Output = Self>
    + std::ops::Add<Self, Output = Self>
    + std::ops::Mul<f32, Output = Self>
    + std::ops::Neg<Output = Self>
    + Sized
    + Copy
    + std::fmt::Debug
{
    fn springable_length(self) -> f32;
    fn springable_normalize_or_zero(self) -> Self;
    fn springable_dot(self, other: Self) -> f32;
}

impl Springable for f32 {
    fn springable_length(self) -> f32 {
        self
    }
    fn springable_normalize_or_zero(self) -> Self {
        1.0
    }
    fn springable_dot(self, other: Self) -> f32 {
        self * other
    }
}

impl Springable for Vec2 {
    fn springable_length(self) -> f32 {
        self.length()
    }
    fn springable_normalize_or_zero(self) -> Self {
        self.normalize_or_zero()
    }
    fn springable_dot(self, other: Self) -> f32 {
        self.dot(other)
    }
}

impl Springable for Vec3 {
    fn springable_length(self) -> f32 {
        self.length()
    }
    fn springable_normalize_or_zero(self) -> Self {
        self.normalize_or_zero()
    }
    fn springable_dot(self, other: Self) -> f32 {
        self.dot(other)
    }
}

impl<S> Particle<S>
where
    S: Springable,
{
    pub fn inverse_mass(&self) -> f32 {
        if self.mass != 0.0 {
            1.0 / self.mass
        } else {
            0.0
        }
    }

    pub fn position(&self) -> S {
        self.position
    }

    pub fn velocity(&self) -> S {
        self.velocity
    }
}

//pub const MIN_DAMP_COEFFICIENT: f32 = 0.0075e-2f32;
pub const MIN_DAMP_COEFFICIENT: f32 = 0.0;

impl Spring {
    /// Impulse required to satisfy the spring constraint.
    ///
    /// This makes assumptions that the integrator for your physics is symplectic Euler.
    /// This allows us to make the spring stable with any provided user inputs by constraining
    /// the spring strength to the `reduced mass / timestep` and the damping to `reduced_mass`.
    pub fn impulse<S>(
        &self,
        timestep: f32,
        particle_a: impl Into<Particle<S>>,
        particle_b: impl Into<Particle<S>>,
        last_impulse_unit: Option<S>,
    ) -> (S, S)
    where
        S: Springable,
    {
        let particle_a = particle_a.into();
        let particle_b = particle_b.into();

        let inverse_timestep = 1.0 / timestep;

        let distance = particle_b.position() - particle_a.position();
        let velocity = particle_b.velocity() - particle_a.velocity();

        let unit_vector = distance.springable_normalize_or_zero();
        let distance_length: f32 = if self.limp_distance > distance.springable_length() {
            0.0
        } else {
            distance.springable_length() - self.rest_distance
        };

        let velocity_vector = last_impulse_unit.unwrap_or(unit_vector);
        let distance_error = unit_vector * distance_length;
        let velocity_error = velocity_vector * velocity.springable_dot(velocity_vector);

        let reduced_mass = 1.0 / (particle_a.inverse_mass() + particle_b.inverse_mass());

        let damping = self.damp_ratio * 2.0 * self.strength.sqrt();

        let distance_impulse = distance_error * self.strength * inverse_timestep * reduced_mass;
        let velocity_impulse =
            velocity_error * damping.clamp(MIN_DAMP_COEFFICIENT, 1.0) * reduced_mass;

        let impulse = -(distance_impulse + velocity_impulse);
        (impulse, unit_vector)
    }
}
