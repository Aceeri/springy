use bevy::{
    prelude::*,
    reflect::{FromReflect, Reflect},
};
use bevy_inspector_egui::prelude::*;

pub mod prelude {
    #[cfg(any(feature = "rapier2d", feature = "rapier3d"))]
    pub use crate::rapier::RapierParticleQuery;
    pub use crate::Spring;
}

#[cfg(any(feature = "rapier2d", feature = "rapier3d"))]
pub mod rapier;

#[cfg(any(feature = "rapier2d", feature = "rapier3d"))]
pub use rapier::RapierParticleQuery;

pub mod kinematic;
use kinematic::*;

#[derive(Default, Debug, Copy, Clone, Component, Reflect, FromReflect, InspectorOptions)]
#[reflect(Component, InspectorOptions)]
pub struct Spring {
    /// Strength of the spring-like impulse. This is a range between 0 and 1
    /// where 1 will bring the spring to equilibrium in 1 timestep.
    #[inspector(min = 0.0, max = 1.0, speed = 0.01)]
    pub strength: f32,
    /// Damping of the spring-like impulse. <1 will be treated as under-dampened
    /// and will overshoot the target, >=1 will be treated as critically dampened
    /// and shouldn't overshoot the target.
    ///
    /// Note this will not be completely respected to avoid instability in the spring.
    /// So overshooting *may* happen if you have a really high strength value.
    #[inspector(min = 0.0, max = 4.0, speed = 0.05)]
    pub damp_ratio: f32,
}

/// One dimensional spring particle
#[derive(Default, Debug)]
pub struct Particle1 {
    /// Resistance the particle has to changes in motion.
    pub inertia: f32,
    /// Current translation of the particle.
    pub position: f32,
    /// Current velocity of the particle.
    pub velocity: f32,
}

impl Particle1 {
    pub fn reduced_inertia(&self, other: &Self) -> f32 {
        (self.inertia.inverse() + other.inertia.inverse()).inverse()
    }

    pub fn instant(&self, other: &Self) -> SpringInstant<f32> {
        SpringInstant {
            reduced_inertia: self.reduced_inertia(other),
            displacement: self.position - other.position,
            velocity: self.velocity - other.velocity,
        }
    }
}

#[derive(Default, Debug)]
pub struct TranslationParticle2 {
    /// Resistance the particle has to changes in motion.
    pub mass: f32,
    /// Current translation of the particle.
    pub translation: Vec2,
    /// Current velocity of the particle.
    pub velocity: Vec2,
}

#[derive(Default, Debug)]
pub struct AngularParticle2 {
    /// Resistance the particle has to changes in angular motion.
    pub inertia: f32,
    /// Current rotation of the particle.
    pub rotation: f32,
    /// Current angular velocity of the particle.
    pub velocity: f32,
}

#[derive(Default, Debug)]
pub struct TranslationParticle3 {
    /// Resistance the particle has to changes in motion.
    pub mass: f32,
    /// Current translation of the particle.
    pub translation: Vec3,
    /// Current velocity of the particle.
    pub velocity: Vec3,
}

#[derive(Default, Debug)]
pub struct AngularParticle3 {
    /// Resistance the particle has to changes in angular motion.
    pub inertia: Vec3,
    /// Current rotation of the particle.
    pub rotation: Quat,
    /// Current angular velocity of the particle.
    pub velocity: Vec3,
}

pub struct SpringInstant<K: Kinematic> {
    pub reduced_inertia: K,
    /// Displacement of the spring, which is the relative positions between particles.
    pub displacement: K,
    /// Velocity of the spring, which is the relative velocity between particles.
    pub velocity: K,
}

impl TranslationParticle2 {
    pub fn reduced_mass(&self, other: &Self) -> f32 {
        (self.mass.inverse() + other.mass.inverse()).inverse()
    }

    pub fn instant(&self, other: &Self) -> SpringInstant<Vec2> {
        SpringInstant {
            reduced_inertia: Vec2::splat(self.reduced_mass(other)),
            displacement: self.translation - other.translation,
            velocity: self.velocity - other.velocity,
        }
    }
}

impl AngularParticle2 {
    pub fn reduced_inertia(&self, other: &Self) -> f32 {
        (self.inertia.inverse() + other.inertia.inverse()).inverse()
    }

    pub fn instant(&self, other: &Self) -> SpringInstant<f32> {
        SpringInstant {
            reduced_inertia: self.reduced_inertia(other),
            displacement: self.rotation - other.rotation,
            velocity: self.velocity - other.velocity,
        }
    }
}

impl TranslationParticle3 {
    pub fn reduced_mass(&self, other: &Self) -> f32 {
        (self.mass.inverse() + other.mass.inverse()).inverse()
    }

    pub fn instant(&self, other: &Self) -> SpringInstant<Vec3> {
        SpringInstant {
            reduced_inertia: Vec3::splat(self.reduced_mass(other)),
            displacement: self.translation - other.translation,
            velocity: self.velocity - other.velocity,
        }
    }
}

impl AngularParticle3 {
    pub fn reduced_inertia(&self, other: &Self) -> Vec3 {
        (self.inertia.inverse() + other.inertia.inverse()).inverse()
    }

    pub fn instant(&self, other: &Self) -> SpringInstant<Vec3> {
        let q1 = self.rotation * Vec3::X;
        let q2 = other.rotation * Vec3::X;
        let diff = q1.cross(q2);
        SpringInstant {
            reduced_inertia: self.reduced_inertia(other),
            displacement: diff,
            velocity: other.velocity - self.velocity,
        }
    }
}

impl Spring {
    pub fn strength(&self) -> f32 {
        self.strength.clamp(0.0, 1.0)
    }

    pub fn damp_ratio(&self) -> f32 {
        self.damp_ratio.clamp(0.0, 20.0)
    }

    pub fn damping(&self) -> f32 {
        (self.damp_ratio() * 2.0 * self.strength().sqrt()).clamp(0.0, 1.0)
    }

    pub fn impulse<K: Kinematic>(&self, timestep: f32, instant: SpringInstant<K>) -> K {
        let inverse_timestep = 1.0 / timestep;

        let unit_vector = instant.displacement.normalize_or_zero();
        let distance_error = unit_vector * instant.displacement.length();
        let velocity_error = instant.velocity;//.dot(unit_vector);

        let distance_impulse =
            distance_error * instant.reduced_inertia * self.strength() * inverse_timestep;
        let velocity_impulse = velocity_error * instant.reduced_inertia * self.damping();

        let impulse = -(distance_impulse + velocity_impulse);
        impulse
    }
}
