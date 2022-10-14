use bevy::math::Vec3Swizzles;
use bevy::{prelude::*, window::PresentMode};

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
        .add_startup_system(setup_graphics)
        .add_startup_system(setup_physics)
        .add_system_to_stage(CoreStage::PostUpdate, symplectic_euler)
        .add_system(spring_impulse)
        .add_system(gravity)
        .register_type::<Impulse>()
        .register_type::<Velocity>()
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

#[derive(Default, Debug, Copy, Clone, Component, Reflect)]
#[reflect(Component)]
pub struct Velocity(Vec2);

#[derive(Default, Debug, Copy, Clone, Component, Reflect)]
#[reflect(Component)]
pub struct Impulse(Vec2);

#[derive(Debug, Copy, Clone, Component, Reflect)]
#[reflect(Component)]
pub struct Mass(f32);

impl Default for Mass {
    fn default() -> Self {
        Self(1.0)
    }
}

impl Mass {
    pub fn inverse_mass(&self) -> f32 {
        if self.0 != 0.0 {
            1.0 / self.0
        } else {
            0.0
        }
    }
}

/// Basic symplectic euler integration of the impulse/velocity/position.
pub fn symplectic_euler(
    time: Res<Time>,
    mut to_integrate: Query<(&mut Transform, &mut Velocity, &mut Impulse, &Mass)>,
) {
    if time.delta_seconds() == 0.0 {
        return;
    }

    for (mut position, mut velocity, mut impulse, mass) in &mut to_integrate {
        velocity.0 += impulse.0 * mass.inverse_mass();
        position.translation += velocity.0.extend(0.0) * TICK_RATE;
        impulse.0 = Vec2::ZERO;
    }
}

pub fn gravity(time: Res<Time>, mut to_apply: Query<(&mut Impulse)>) {
    if time.delta_seconds() == 0.0 {
        return;
    }

    for (mut impulse) in &mut to_apply {
        impulse.0 += Vec2::new(0.0, -9.817);
    }
}

pub fn spring_impulse(
    time: Res<Time>,
    mut impulses: Query<&mut Impulse>,
    springs: Query<(
        Entity,
        &GlobalTransform,
        &Velocity,
        &Mass,
        &SpringSettings,
        &Spring,
    )>,
    particle: Query<(&GlobalTransform, &Velocity, &Mass)>,
) {
    if time.delta_seconds() == 0.0 {
        return;
    }

    let timestep = TICK_RATE;

    for (spring_entity, spring_transform, spring_velocity, spring_mass, spring_settings, spring) in
        &springs
    {
        let particle_entity = spring.containing;
        let (particle_transform, particle_velocity, particle_mass) =
            particle.get(particle_entity).unwrap();

        if particle_entity == spring_entity {
            continue;
        }

        let impulse = spring_settings.0.impulse(
            timestep,
            springy::Particle {
                mass: spring_mass.0,
                position: spring_transform.translation().xy(),
                velocity: spring_velocity.0,
            },
            springy::Particle {
                mass: particle_mass.0,
                position: particle_transform.translation().xy(),
                velocity: particle_velocity.0,
            },
        );

        let [mut spring_impulse, mut particle_impulse] = impulses
            .get_many_mut([spring_entity, particle_entity])
            .unwrap();

        spring_impulse.0 -= impulse;
        particle_impulse.0 += impulse;
    }
}

pub fn setup_physics(mut commands: Commands) {
    /*
     * Ground
     */
    let ground_size = 5000.0;
    let ground_height = 10.0;

    commands.spawn_bundle(TransformBundle::from(Transform::from_xyz(
        0.0,
        -ground_height - 100.0,
        0.0,
    )));

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

    let cube_3 = commands
        .spawn()
        .insert_bundle(SpriteBundle {
            sprite: sprite.clone(),
            ..default()
        })
        .insert_bundle((Mass::default(), Velocity::default(), Impulse::default()))
        .insert(Name::new("Cube 3"))
        .id();

    let cube_2 = commands
        .spawn()
        .insert_bundle(SpriteBundle {
            sprite: sprite.clone(),
            ..default()
        })
        .insert_bundle((Velocity::default(), Impulse::default(), Mass::default()))
        .insert(Name::new("Cube 2"))
        .insert(Spring { containing: cube_3 })
        .insert(SpringSettings(springy::Spring {
            rest_distance: 50.0,
            limp_distance: 0.0,
            strength: 0.01,
            damp_ratio: 1.0,
        }))
        .id();

    let cube_1 = commands
        .spawn()
        .insert_bundle(SpriteBundle {
            sprite: sprite.clone(),
            ..default()
        })
        .insert_bundle(TransformBundle::from(Transform::from_xyz(50.0, 50.0, 0.0)))
        .insert_bundle((Velocity::default(), Impulse::default(), Mass::default()))
        .insert(Spring { containing: cube_2 })
        .insert(SpringSettings(springy::Spring {
            rest_distance: 50.0,
            limp_distance: 0.0,
            strength: 0.33,
            damp_ratio: 1.0,
        }))
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
            rest_distance: 50.0,
            limp_distance: 0.0,
            strength: 0.01,
            damp_ratio: 1.0,
        }))
        .insert_bundle((Velocity::default(), Impulse::default(), Mass(f32::INFINITY)))
        .insert(Name::new("Cube Slot"));
}
