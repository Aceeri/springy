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
            Some(mass) => mass.0,
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
                prop.mass = f32::INFINITY;
                #[cfg(feature = "rapier2d")]
                {
                    prop.principal_inertia = f32::INFINITY;
                }
                #[cfg(feature = "rapier3d")]
                {
                    prop.principal_inertia = Unit::splat(f32::INFINITY);
                }
            }
            _ => {}
        }

        prop
    }

    pub fn translation_particle(&self) -> Particle<Unit> {
        let velocity = self.velocity();
        let mass = self.mass();

        #[cfg(feature = "rapier2d")]
        let linvel = velocity.linvel;
        /*
               let linvel =
                   velocity.linvel + Unit::ZERO - self.local_center_of_mass() + velocity.angvel.perp_dot();
        */
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
            inertia: Unit::splat(mass.mass),
        }
    }

    #[cfg(feature = "rapier2d")]
    pub fn angular_particle(&self) -> Particle<Unit> {
        let velocity = self.velocity();
        let mass = self.mass();
        let up = self.global_transform.compute_transform().up();

        let angvel = velocity.angvel;
        Particle {
            position: up.xy(),
            velocity: Vec2::splat(angvel),
            inertia: Vec2::splat(mass.principal_inertia),
        }
    }

    #[cfg(feature = "rapier3d")]
    pub fn angular_particle(&self) -> Particle<Unit> {
        let velocity = self.velocity();
        let mass = self.mass();
        let up = self.global_transform.compute_transform().up();

        let angvel = velocity.angvel;
        Particle {
            position: up,
            velocity: angvel,
            inertia: mass.principal_inertia,
        }
    }
}
