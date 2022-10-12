use bevy::prelude::*;
#[cfg(feature = "rapier2d")]
use bevy_rapier2d::prelude::*;
#[cfg(feature = "rapier3d")]
use bevy_rapier3d::prelude::*;

use crate::Particle;

use bevy::ecs::query::WorldQuery;
use bevy::math::Vec3Swizzles;

#[derive(WorldQuery)]
pub struct RapierParticleQuery<'a> {
    pub global_transform: &'a GlobalTransform,
    pub rigid_body: Option<&'a RigidBody>,
    pub velocity: Option<&'a Velocity>,
    pub mass: Option<&'a ReadMassProperties>,
}

#[cfg(feature = "rapier2d")]
pub type Unit = Vec2;
#[cfg(feature = "rapier3d")]
pub type Unit = Vec3;

impl<'w, 's> Into<Particle<Unit>> for RapierParticleQueryItem<'w, 's> {
    fn into(self) -> Particle<Unit> {
        self.impulse_particle()
    }
}

impl<'w, 's> RapierParticleQueryItem<'w, 's> {
    pub fn velocity(&self) -> Velocity {
        match self.velocity {
            Some(velocity) => *velocity,
            None => match self.rigid_body {
                Some(
                    rigid_body @ RigidBody::Dynamic
                    | rigid_body @ RigidBody::KinematicVelocityBased,
                ) => {
                    warn!(
                        "{:?} rigidbody needs a `Velocity` component for spring damping",
                        rigid_body
                    );
                    Velocity::default()
                }
                _ => Velocity::default(),
            },
        }
    }

    pub fn mass(&self) -> f32 {
        match self.rigid_body {
            Some(rigid_body @ RigidBody::Dynamic) => match self.mass {
                Some(mass) => mass.0.mass,
                _ => {
                    warn!(
                        "{:?} rigidbody needs a `ReadMassProperties` component for accurate spring calculations",
                        rigid_body
                    );
                    1.0
                }
            },
            Some(_) => f32::INFINITY,
            _ => 1.0,
        }
    }

    pub fn local_center_of_mass(&self) -> Unit {
        match self.mass {
            Some(mass) => mass.0.local_center_of_mass,
            _ => {
                warn!(
                    "Rigidbody needs a `ReadMassProperties` component for accurate spring calculations",
                );
                Unit::ZERO
            }
        }
    }
    /*

    pub fn angular_particle(&self) -> Particle<Unit> {
        let velocity = self.velocity();
        Particle {
            #[cfg(feature = "rapier3d")]
            position: self.global_transform.rotation(),
            #[cfg(feature = "rapier2d")]
            position: self.global_transform.rotation().xy(),

            velocity: linvel,
            mass: self.mass(),
        }
    } */

    pub fn impulse_particle(&self) -> Particle<Unit> {
        let velocity = self.velocity();
        #[cfg(feature = "rapier2d")]
        let linvel = velocity.linvel;
        #[cfg(feature = "rapier3d")]
        let linvel = velocity.linvel
            + velocity
                .angvel
                .cross(Unit::ZERO - self.local_center_of_mass());
        Particle {
            #[cfg(feature = "rapier3d")]
            position: self.global_transform.translation(),
            #[cfg(feature = "rapier2d")]
            position: self.global_transform.translation().xy(),

            velocity: linvel,
            mass: self.mass(),
        }
    }
}
