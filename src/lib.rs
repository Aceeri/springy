use bevy::prelude::*;

pub mod prelude {
    pub use super::{Particle, Spring};
    #[cfg(any(feature = "rapier2d", feature = "rapier3d"))]
    pub use rapier::RapierParticleQuery;
}

#[cfg(any(feature = "rapier2d", feature = "rapier3d"))]
pub mod rapier;

#[derive(Default, Debug, Copy, Clone, Component, Reflect)]
#[reflect(Component)]
pub struct Spring {
    /// Strength of the spring-like impulse. This is a range between 0 and 1
    /// where 1 will bring the spring to equilibrium in 1 timestep.
    pub strength: f32,
    /// Damping of the spring-like impulse. This is a range between 0 and 1
    /// where 1 will bring the spring to equilibrium in 1 timestep.
    pub damping: f32,
    /// Rest distance around the particle, it will try to push the particle out
    /// when too close.
    pub rest_distance: f32,
    /// Similar to rest distance except it will not push outwards if it is too close.
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
}

impl Springable for f32 {
    fn springable_length(self) -> f32 {
        self
    }
    fn springable_normalize_or_zero(self) -> Self {
        1.0
    }
}

impl Springable for Vec2 {
    fn springable_length(self) -> f32 {
        self.length()
    }
    fn springable_normalize_or_zero(self) -> Self {
        self.normalize_or_zero()
    }
}

impl Springable for Vec3 {
    fn springable_length(self) -> f32 {
        self.length()
    }
    fn springable_normalize_or_zero(self) -> Self {
        self.normalize_or_zero()
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

impl Spring {
    /// Impulse required to satisfy the spring constraint.
    ///
    /// This makes assumptions that the integrator for your physics is symplectic Euler.
    /// This allows us to make the spring stable with any provided user inputs by constraining
    /// the spring strength to the `reduced mass / timestep` and the damping to `reduced_mass`.
    pub fn impulse<S>(&self, timestep: f32, particle_a: Particle<S>, particle_b: Particle<S>) -> S
    where
        S: Springable,
    {
        let inverse_timestep = 1.0 / timestep;

        let distance = particle_b.position() - particle_a.position();
        let velocity = particle_b.velocity() - particle_a.velocity();

        let unit_vector = distance.springable_normalize_or_zero();
        let distance_length: f32 = if self.limp_distance > distance.springable_length() {
            0.0
        } else {
            distance.springable_length() - self.rest_distance
        };

        let distance_error = unit_vector * distance_length;
        let velocity_error = velocity;

        let reduced_mass = 1.0 / (particle_a.inverse_mass() + particle_b.inverse_mass());

        let distance_impulse = distance_error * self.strength * inverse_timestep * reduced_mass;
        let velocity_impulse = velocity_error * self.damping * reduced_mass;

        let impulse = -(distance_impulse + velocity_impulse);
        impulse
    }
}
