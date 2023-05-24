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
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(10.0))
        .add_plugin(RapierDebugRenderPlugin::default())
        .add_startup_system(setup_graphics)
        .add_startup_system(setup_physics)
        .add_system(spring_impulse)
        .register_type::<SpringSettings>()
        .run();
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn(Camera2dBundle {
        transform: Transform::from_xyz(0.0, 20.0, 0.0),
        ..default()
    });
}

#[derive(Debug, Copy, Clone, Component)]
pub struct Spring {
    pub containing: Entity,
}

#[derive(Default, Debug, Clone, Component, Reflect)]
#[reflect(Component)]
pub struct SpringSettings(springy::SpringState<Vec2>);

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

        match spring_settings.0.impulse(timestep, particle_a, particle_b) {
            springy::SpringResult::Impulse(impulse) => {
                let [mut spring_impulse, mut particle_impulse] = impulses
                    .get_many_mut([spring_entity, particle_entity])
                    .unwrap();

                spring_impulse.impulse = -impulse;
                particle_impulse.impulse = impulse;
            }
            _ => {}
        }
    }
}

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

    let cube_1 = commands
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
        .insert(Name::new("Cube 1"))
        .id();

    let cube_slot = commands
        .spawn(SpriteBundle {
            sprite: slot,
            ..default()
        })
        .insert(TransformBundle::from(Transform::from_xyz(50.0, 50.0, 0.0)))
        .insert(Spring { containing: cube_1 })
        .insert(SpringSettings(springy::SpringState::new(springy::Spring {
            rest_distance: 5.0,
            limp_distance: 5.0,
            strength: 1.0,
            damp_ratio: 1.0,
        })))
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
        .insert(Name::new("Cube 2"));
}
