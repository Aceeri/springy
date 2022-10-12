#[cfg(feature = "rapier2d")]
use bevy_rapier2d::prelude::*;
#[cfg(feature = "rapier3d")]
use bevy_rapier3d::prelude::*;

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

impl<'a> RapierParticleQueryItem<'a> {
    pub fn into_particle(&self) -> crate::Particle<Unit> {
        let velocity = match self.velocity {
            Some(velocity) => velocity,
            None => match self.rigid_body {
                Some(rigid_body @ RigidBody::Dynamic | RigidBody::KinematicVelocityBased) => {
                    warn!(
                        "{:?} rigidbody needs a `Velocity` component for spring damping",
                        rigid_body
                    );
                    Velocity::default()
                }
                _ => Velocity::default(),
            },
        };

        let mass = match self.rigid_body {
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
        };

        Particle {
            #[cfg(feature = "rapier3d")]
            position: self.global_transform.translation(),
            #[cfg(feature = "rapier2d")]
            position: self.global_transform.translation().xy(),

            velocity: velocity.linvel,
            mass: mass,
        }
    }
}
