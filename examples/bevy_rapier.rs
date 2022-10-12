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
        .insert_resource(RapierConfiguration {
            timestep_mode: TimestepMode,
            ..default()
        })
        .insert_resource(Msaa::default())
        .add_plugins(DefaultPlugins)
        .add_plugin(bevy_editor_pls::EditorPlugin)
        .add_plugin(RapierPhysicsPlugin)
        .add_plugin(RapierDebugRenderPlugin)
        .add_startup_system(setup_graphics)
        .add_startup_system(setup_physics)
        .add_system_to_stage(CoreStage::PostUpdate, symplectic_euler)
        .add_system(spring_impulse)
        .register_type::<Spring>()
        .run();
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn_bundle(Camera2dBundle {
        transform: Transform::from_xyz(0.0, 20.0, 0.0),
        ..default()
    });
}

#[derive(Debug, Copy, Clone, Component)]
pub struct Attached {
    pub attached: Entity,
    pub spring: Spring,
}

pub fn spring_impulse(
    time: Res<Time>,
    mut impulses: Query<Option<&mut ExternalImpulse>>,
    attached: Query<(Entity, &Attached)>,
    particle: Query<(&GlobalTransform, &Velocity, &ReadMassProperties)>,
) {
    if time.delta_seconds() == 0.0 {
        return;
    }

    let timestep = TICK_RATE;
    let inverse_timestep = 1.0 / timestep;

    for (spring_entity, spring_transform, spring_velocity, spring_mass, spring_settings, spring) in
        &springs
    {
        let particle_entity = spring.containing;
        let (particle_transform, particle_velocity, particle_mass) =
            particle.get(particle_entity).unwrap();

        if particle_entity == spring_entity {
            continue;
        }

        let strength = spring_settings.strength;
        let damping = spring_settings.damping;
        let rest_distance = spring_settings.rest_distance;
        let limp_distance = spring_settings.limp_distance;

        let distance = particle_transform.translation() - spring_transform.translation();
        let distance = Vec2::new(distance.x, distance.y);
        let velocity = particle_velocity.linvel - spring_velocity.linvel;

        let unit_vector = distance.normalize_or_zero();
        let distance_error = if limp_distance > distance.length() {
            0.0
        } else {
            unit_vector.dot(distance) - rest_distance
        };
        let distance_error = distance_error * unit_vector;
        let velocity_error = velocity;

        let reduced_mass = 1.0 / (spring_mass.inverse_mass() + particle_mass.inverse_mass());

        let distance_impulse = strength * distance_error * inverse_timestep * reduced_mass;
        let velocity_impulse = damping * velocity_error * reduced_mass;

        let impulse = -(distance_impulse + velocity_impulse);

        let [mut spring_impulse, mut particle_impulse] = impulses
            .get_many_mut([spring_entity, particle_entity])
            .unwrap();
        spring_impulse.impulse -= impulse;
        particle_impulse.impulse += impulse;
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
        .insert_bundle((Velocity::default(), Impulse::default(), Mass::default()))
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
        .insert(SpringSettings {
            rest_distance: 5.0,
            limp_distance: 5.0,
            strength: 1.0,
            damping: 1.0,
        })
        .insert_bundle((Velocity::default(), Impulse::default(), Mass::new(0.0)))
        .insert(Name::new("Cube Slot"));

    commands
        .spawn_bundle(TransformBundle::from(Transform::from_xyz(0.0, 0.0, 0.0)))
        .insert_bundle((Mass::default(), Velocity::default(), Impulse::default()))
        .insert(Name::new("Cube 2"));
}
