use bevy::math::Vec3Swizzles;
use bevy::{prelude::*, window::PresentMode};
use bevy_rapier2d::prelude::*;

const TICK_RATE: f32 = 1.0 / 100.0;

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::rgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .insert_resource(Msaa::default())
        .add_plugins(DefaultPlugins)
        .add_plugin(bevy_editor_pls::EditorPlugin::new())
        .insert_resource(bevy_framepace::FramepaceSettings {
            limiter: bevy_framepace::Limiter::Manual(std::time::Duration::from_secs_f64(TICK_RATE)),
            ..default()
        })
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(10.0))
        .add_plugin(RapierDebugRenderPlugin::default())
        .add_startup_system(setup_graphics)
        //.add_startup_system(setup_translation)
        .add_startup_system(setup_rotation)
        .add_system(spring_impulse)
        .register_type::<SpringSettings>()
        .run();
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn(Camera2dBundle {
        transform: Transform::from_xyz(-100.0, 100.0, 0.0),
        ..default()
    });
}

#[derive(Debug, Copy, Clone, Component)]
pub struct Spring {
    pub containing: Entity,
}

#[derive(Default, Debug, Copy, Clone, Component, Reflect)]
#[reflect(Component)]
pub struct SpringSettings(springy::Spring);

/*
pub fn spring_impulse(
    time: Res<Time>,
    mut impulses: Query<&mut Impulse>,
    mut springs: Query<(
        Entity,
        &GlobalTransform,
        &Velocity,
        &Inertia,
        &SpringSettings,
        &Spring,
        &mut PreviousUnitVector,
    )>,
    particle: Query<(&GlobalTransform, &Velocity, &Inertia)>,
) {
    if time.delta_seconds() == 0.0 {
        return;
    }

    let timestep = TICK_RATE as f32;

    for (
        spring_entity,
        spring_transform,
        spring_velocity,
        spring_mass,
        spring_settings,
        spring,
        mut previous_unit_vector,
    ) in &mut springs
    {
        let particle_entity = spring.containing;
        let (particle_transform, particle_velocity, particle_mass) =
            particle.get(particle_entity).unwrap();

        if particle_entity == spring_entity {
            continue;
        }

        let (_, spring_rotation, spring_translation) =
            spring_transform.to_scale_rotation_translation();
        let particle_a = springy::TranslationParticle2 {
            mass: spring_mass.linear,
            translation: spring_translation.xy(),
            velocity: spring_velocity.linear,
        };

        let unit_vector = spring_rotation * Vec3::X;
        let angular_particle_a = springy::AngularParticle2 {
            inertia: spring_mass.angular,
            rotation: unit_vector.y.atan2(unit_vector.x),
            velocity: spring_velocity.angular,
        };

        let (_, particle_rotation, particle_translation) =
            particle_transform.to_scale_rotation_translation();
        let particle_b = springy::TranslationParticle2 {
            mass: particle_mass.linear,
            translation: particle_translation.xy(),
            velocity: particle_velocity.linear,
        };

        let unit_vector_b = particle_rotation * Vec3::X;
        let angular_particle_b = springy::AngularParticle2 {
            inertia: particle_mass.angular,
            rotation: unit_vector_b.y.atan2(unit_vector_b.x),
            velocity: particle_velocity.angular,
        };

        let instant = particle_a.instant(&particle_b);
        let impulse = spring_settings.0.impulse(timestep, instant);

        let angular_instant = angular_particle_a.instant(&angular_particle_b);
        let angular_impulse = spring_settings.0.impulse(timestep, angular_instant);

        let [mut spring_impulse, mut particle_impulse] = impulses
            .get_many_mut([spring_entity, particle_entity])
            .unwrap();

        spring_impulse.linear += impulse;
        spring_impulse.angular += angular_impulse;
        particle_impulse.linear -= impulse;
        particle_impulse.angular -= angular_impulse;
    }
}
*/

pub fn spring_impulse(
    time: Res<Time>,
    mut impulses: Query<&mut ExternalImpulse>,
    mut springs: Query<(Entity, &mut SpringSettings, &Spring)>,
    particle: Query<springy::RapierParticleQuery>,
) {
    if time.delta_seconds() == 0.0 {
        return;
    }

    let timestep = TICK_RATE;

    for (spring_entity, mut spring_settings, spring) in &mut springs {
        let particle_entity = spring.containing;
        let particle_a = particle.get(spring_entity).unwrap();
        let particle_b = particle.get(particle_entity).unwrap();

        if particle_entity == spring_entity {
            continue;
        }

        let translate_particle_a = particle_a.translation();
        let angular_particle_a = particle_a.angular();
        let translate_particle_b = particle_b.translation();
        let angular_particle_b = particle_b.angular();

        let instant = translate_particle_a.instant(&translate_particle_b);
        let impulse = spring_settings.0.impulse(timestep, instant);

        let angular_instant = angular_particle_a.instant(&angular_particle_b);
        let angular_impulse = spring_settings.0.impulse(timestep, angular_instant);
        //let angular_impulse = angular_impulse * 0.001;
        let [mut spring_impulse, mut particle_impulse] = impulses
            .get_many_mut([spring_entity, particle_entity])
            .unwrap();

        spring_impulse.impulse += impulse;
        spring_impulse.torque_impulse += angular_impulse;
        particle_impulse.impulse -= impulse;
        particle_impulse.torque_impulse -= angular_impulse;
    }
}

/*
pub fn setup_physics(mut commands: Commands) {
    /*
     * Ground
     */
    let ground_size = 5000.0;
    let ground_height = 10.0;

    commands
        .spawn(TransformBundle::from(Transform::from_xyz(
            0.0,
            -ground_height - 100.0,
            0.0,
        )))
        .insert(Collider::cuboid(ground_size, ground_height));

    /*
     * Create the cubes
     */
    let size = 20.0;
    let sprite = Sprite {
        color: Color::BLUE,
        flip_x: false,
        flip_y: false,
        custom_size: Some(Vec2::new(size, size)),
        rect: None,
        anchor: Default::default(),
    };

    let slot = Sprite {
        color: Color::RED,
        flip_x: false,
        flip_y: false,
        custom_size: Some(Vec2::new(size / 4.0, size / 4.0)),
        rect: None,
        anchor: Default::default(),
    };

    let slotted_cube = commands
        .spawn(SpriteBundle {
            sprite: sprite,
            ..default()
        })
        .insert(TransformBundle::from(Transform::from_xyz(50.0, 50.0, 0.0)))
        .insert((
            RigidBody::Dynamic,
            Velocity::default(),
            ExternalImpulse::default(),
            ReadMassProperties::default(),
            Collider::cuboid(size / 2.0, size / 2.0),
        ))
        .insert(Name::new("Slotted Cube"))
        .id();

    let cube_slot = commands
        .spawn(SpriteBundle {
            sprite: slot,
            ..default()
        })
        .insert(TransformBundle::from(Transform::from_xyz(50.0, 50.0, 0.0)))
        .insert(Spring { containing: cube_1 })
        .insert(SpringSettings(springy::Spring {
            strength: 1.0,
            damp_ratio: 1.0,
        }))
        .insert((
            //RigidBody::Dynamic,
            Velocity::default(),
            ExternalImpulse::default(),
            ReadMassProperties::default(),
            //Collider::cuboid(size, size),
        ))
        .insert(Name::new("Cube Slot"));

    commands
        .spawn(TransformBundle::from(Transform::from_xyz(0.0, 0.0, 0.0)))
        .insert((
            RigidBody::Dynamic,
            Velocity::default(),
            ExternalImpulse::default(),
            ReadMassProperties::default(),
            Collider::cuboid(size / 2.0, size / 2.0),
        ))
        .insert(Name::new("Random cube"));
}
 */

pub fn setup_translation(mut commands: Commands) {
    let size = 20.0;
    let sprite = Sprite {
        color: Color::BLUE,
        flip_x: false,
        flip_y: false,
        custom_size: Some(Vec2::new(size, size)),
        rect: None,
        anchor: Default::default(),
    };

    let slot = Sprite {
        color: Color::RED,
        flip_x: false,
        flip_y: false,
        custom_size: Some(Vec2::new(size / 4.0, size / 4.0)),
        rect: None,
        anchor: Default::default(),
    };

    let iterations = 100;
    let height = 500.0;
    let size = height / iterations as f32;
    let margin = size * 0.5;

    for iteration in 0..iterations {
        let damped_sprite = Sprite {
            color: Color::YELLOW,
            flip_x: false,
            flip_y: false,
            custom_size: Some(Vec2::new(size, size)),
            rect: None,
            anchor: Default::default(),
        };

        let height = iteration as f32 * (size + margin);
        let damped_cube = commands
            .spawn(SpriteBundle {
                sprite: damped_sprite.clone(),
                ..default()
            })
            .insert(TransformBundle::from(Transform::from_xyz(
                300.0, height, 0.0,
            )))
            .insert((
                RigidBody::Dynamic,
                Velocity::default(),
                ExternalImpulse::default(),
                ReadMassProperties::default(),
                Collider::cuboid(size / 2.0, size / 2.0),
            ))
            .insert(Name::new(format!("Translational {}", iteration)))
            .id();

        let damp_ratio = iteration as f32 / iterations as f32;
        let critical_slot = commands
            .spawn(SpriteBundle {
                sprite: slot.clone(),
                ..default()
            })
            .insert(TransformBundle::from(Transform::from_xyz(
                100.0, height, 0.0,
            )))
            .insert(Spring {
                containing: damped_cube,
            })
            .insert(SpringSettings(springy::Spring {
                strength: 0.05,
                damp_ratio: damp_ratio,
            }))
            .insert((
                //RigidBody::Dynamic,
                Velocity::default(),
                ExternalImpulse::default(),
                ReadMassProperties::default(),
                //Collider::cuboid(size, size),
            ))
            .insert(Name::new(format!(
                "Slot {} (ratio {})",
                iteration, damp_ratio
            )));
    }
}

pub fn setup_rotation(mut commands: Commands) {
    let size = 20.0;
    let sprite = Sprite {
        color: Color::BLUE,
        flip_x: false,
        flip_y: false,
        custom_size: Some(Vec2::new(size, size)),
        rect: None,
        anchor: Default::default(),
    };

    let slot = Sprite {
        color: Color::RED,
        flip_x: false,
        flip_y: false,
        custom_size: Some(Vec2::new(size / 4.0, size / 4.0)),
        rect: None,
        anchor: Default::default(),
    };

    let iterations = 100;
    let height = 500.0;
    let size = height / iterations as f32;
    let margin = size * 1.0;

    for iteration in 0..iterations {
        let damped_sprite = Sprite {
            color: Color::YELLOW,
            flip_x: false,
            flip_y: false,
            custom_size: Some(Vec2::new(size, size)),
            rect: None,
            anchor: Default::default(),
        };

        let height = iteration as f32 * (size + margin);
        let damped_cube = commands
            .spawn(SpriteBundle {
                sprite: damped_sprite.clone(),
                ..default()
            })
            .insert(TransformBundle::from(Transform {
                translation: Vec3::new(-200.0, height, 0.0),
                rotation: Quat::from_rotation_z(1.0),
                scale: Vec3::splat(1.0),
            }))
            .insert((
                RigidBody::Dynamic,
                Velocity::default(),
                ExternalImpulse::default(),
                ReadMassProperties::default(),
                Collider::cuboid(size / 2.0, size / 2.0),
            ))
            .insert(Name::new(format!("Rotation {}", iteration)))
            .id();

        let damp_ratio = iteration as f32 / iterations as f32;
        let critical_slot = commands
            .spawn(SpriteBundle {
                sprite: slot.clone(),
                ..default()
            })
            .insert(TransformBundle::from(Transform::from_xyz(
                -200.0, height, 0.0,
            )))
            .insert(Spring {
                containing: damped_cube,
            })
            .insert(SpringSettings(springy::Spring {
                strength: 0.7,
                damp_ratio: damp_ratio,
            }))
            .insert((
                //RigidBody::Dynamic,
                Velocity::default(),
                ExternalImpulse::default(),
                ReadMassProperties::default(),
                //Collider::cuboid(size, size),
            ))
            .insert(Name::new(format!(
                "Rot Slot {} (ratio {})",
                iteration, damp_ratio
            )));
    }
}
