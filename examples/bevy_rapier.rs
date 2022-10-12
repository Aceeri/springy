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
        .insert_resource(WindowDescriptor {
            present_mode: PresentMode::AutoVsync,
            ..default()
        })
        .insert_resource(Msaa::default())
        .add_plugins(DefaultPlugins)
        .add_plugin(bevy_editor_pls::EditorPlugin)
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(10.0))
        .add_plugin(RapierDebugRenderPlugin::default())
        .add_startup_system(setup_graphics)
        .add_startup_system(setup_physics)
        .add_system(spring_impulse)
        .register_type::<SpringSettings>()
        .run();
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn_bundle(Camera2dBundle {
        transform: Transform::from_xyz(0.0, 20.0, 0.0),
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

pub fn spring_impulse(
    time: Res<Time>,
    mut impulses: Query<&mut ExternalImpulse>,
    springs: Query<(Entity, &SpringSettings, &Spring)>,
    particle: Query<springy::RapierParticleQuery>,
) {
    if time.delta_seconds() == 0.0 {
        return;
    }

    let timestep = TICK_RATE;

    for (spring_entity, spring_settings, spring) in &springs {
        let particle_entity = spring.containing;
        let particle_a = particle.get(spring_entity).unwrap();
        let particle_b = particle.get(particle_entity).unwrap();

        if particle_entity == spring_entity {
            continue;
        }

        let impulse = spring_settings.0.impulse(timestep, particle_a, particle_b);

        let [mut spring_impulse, mut particle_impulse] = impulses
            .get_many_mut([spring_entity, particle_entity])
            .unwrap();

        spring_impulse.impulse = -impulse;
        particle_impulse.impulse = impulse;
    }
}

pub fn setup_physics(mut commands: Commands) {
    /*
     * Ground
     */
    let ground_size = 5000.0;
    let ground_height = 10.0;

    commands
        .spawn_bundle(TransformBundle::from(Transform::from_xyz(
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
        anchor: Default::default(),
    };

    let slot = Sprite {
        color: Color::RED,
        flip_x: false,
        flip_y: false,
        custom_size: Some(Vec2::new(size / 4.0, size / 4.0)),
        anchor: Default::default(),
    };

    let cube_1 = commands
        .spawn()
        .insert_bundle(SpriteBundle {
            sprite: sprite,
            ..default()
        })
        .insert_bundle(TransformBundle::from(Transform::from_xyz(50.0, 50.0, 0.0)))
        .insert_bundle((
            RigidBody::Dynamic,
            Velocity::default(),
            ExternalImpulse::default(),
            ReadMassProperties::default(),
            Collider::cuboid(size / 2.0, size / 2.0),
        ))
        .insert(Name::new("Cube 1"))
        .id();

    let cube_slot = commands
        .spawn()
        .insert_bundle(SpriteBundle {
            sprite: slot,
            ..default()
        })
        .insert_bundle(TransformBundle::from(Transform::from_xyz(50.0, 50.0, 0.0)))
        .insert(Spring { containing: cube_1 })
        .insert(SpringSettings(springy::Spring {
            rest_distance: 5.0,
            limp_distance: 5.0,
            strength: 1.0,
            damping: 1.0,
        }))
        .insert_bundle((
            //RigidBody::Dynamic,
            Velocity::default(),
            ExternalImpulse::default(),
            ReadMassProperties::default(),
            //Collider::cuboid(size, size),
        ))
        .insert(Name::new("Cube Slot"));

    commands
        .spawn_bundle(TransformBundle::from(Transform::from_xyz(0.0, 0.0, 0.0)))
        .insert_bundle((
            RigidBody::Dynamic,
            Velocity::default(),
            ExternalImpulse::default(),
            ReadMassProperties::default(),
            Collider::cuboid(size / 2.0, size / 2.0),
        ))
        .insert(Name::new("Cube 2"));
}
