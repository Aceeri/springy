use bevy::prelude::*;
#[cfg(feature = "rapier2d")]
use bevy_rapier2d::prelude::*;
#[cfg(feature = "rapier3d")]
use bevy_rapier3d::prelude::*;

use bevy::ecs::query::{QueryData, WorldQuery};
use bevy::math::Vec3Swizzles;

use crate::*;

#[derive(QueryData)]
pub struct RapierParticleQuery<'a> {
    pub entity: Entity,
    pub global_transform: &'a GlobalTransform,
    pub rigid_body: Option<&'a RigidBody>,
    pub velocity: Option<&'a Velocity>,
    pub mass: Option<&'a ReadMassProperties>,
    pub name: Option<&'a Name>,
}

#[cfg(feature = "rapier2d")]
pub type Unit = Vec2;
#[cfg(feature = "rapier3d")]
pub type Unit = Vec3;

impl<'w, 's> RapierParticleQueryItem<'w, 's> {
    pub fn name<'a>(&'a self) -> Box<dyn std::fmt::Debug + 'a> {
        match self.name {
            Some(name) => Box::new(name),
            None => Box::new(self.entity),
        }
    }

    pub fn velocity(&self) -> Velocity {
        match self.velocity {
            Some(velocity) => *velocity,
            None => match self.rigid_body {
                Some(
                    rigid_body @ RigidBody::Dynamic
                    | rigid_body @ RigidBody::KinematicVelocityBased,
                ) => {
                    warn!(
                        "{:?} rigidbody for {:?} needs a `Velocity` component for spring damping",
                        rigid_body,
                        self.name()
                    );
                    Velocity::default()
                }
                _ => Velocity::default(),
            },
        }
    }

    pub fn mass(&self) -> MassProperties {
        let mut prop = match self.mass {
            Some(mass) => mass.get().clone(),
            None => {
                match self.rigid_body {
                    Some(RigidBody::KinematicVelocityBased | RigidBody::Dynamic) => {
                        warn!(
                            "{:?} rigidbody for {:?} needs a `ReadMassProperties` component for spring damping",
                            self.rigid_body,
                            self.name()
                        );
                    }
                    _ => {}
                }
                MassProperties::default()
            }
        };

        match self.rigid_body {
            Some(
                RigidBody::KinematicVelocityBased
                | RigidBody::KinematicPositionBased
                | RigidBody::Fixed,
            ) => {
                prop.mass = 0.0;
                #[cfg(feature = "rapier2d")]
                {
                    prop.principal_inertia = 0.0;
                }
                #[cfg(feature = "rapier3d")]
                {
                    prop.principal_inertia = Unit::splat(0.0);
                }
            }
            _ => {}
        }

        prop
    }

    #[cfg(feature = "rapier2d")]
    pub fn translation(&self) -> TranslationParticle2 {
        let velocity = self.velocity();
        let mass = self.mass();
        let linvel = velocity.linvel;
        TranslationParticle2 {
            translation: self.global_transform.translation().xy(),
            velocity: linvel,
            mass: mass.mass,
        }
    }

    #[cfg(feature = "rapier3d")]
    pub fn translation(&self) -> TranslationParticle3 {
        let velocity = self.velocity();
        let mass = self.mass();
        let linvel = velocity.linvel
            + velocity
                .angvel
                .cross(Unit::ZERO - mass.local_center_of_mass);
        TranslationParticle3 {
            translation: self.global_transform.translation(),
            velocity: linvel,
            mass: mass.mass,
        }
    }

    #[cfg(feature = "rapier2d")]
    pub fn angular(&self) -> AngularParticle2 {
        let velocity = self.velocity();
        let mass = self.mass();
        let rotation = self.global_transform.compute_transform().rotation;
        let vector = rotation * Vec3::X;
        let angle = vector.y.atan2(vector.x);
        AngularParticle2 {
            rotation: angle,
            velocity: velocity.angvel,
            inertia: mass.principal_inertia,
        }
    }

    #[cfg(feature = "rapier3d")]
    pub fn angular(&self, axis: Vec3) -> AngularParticle3 {
        let velocity = self.velocity();
        let mass = self.mass();
        let global = self.global_transform.compute_transform();
        AngularParticle3 {
            direction: global.rotation * axis,
            velocity: velocity.angvel,
            inertia: mass.principal_inertia,
        }
    }

    #[cfg(feature = "rapier3d")]
    pub fn angular_x(&self) -> AngularParticle3 {
        self.angular(Vec3::X)
    }

    #[cfg(feature = "rapier3d")]
    pub fn angular_y(&self) -> AngularParticle3 {
        self.angular(Vec3::Y)
    }

    #[cfg(feature = "rapier3d")]
    pub fn angular_z(&self) -> AngularParticle3 {
        self.angular(Vec3::Z)
    }
}
