use bevy::math::Vec3Swizzles;
use bevy::time::FixedTimestep;
use bevy::{prelude::*, window::PresentMode};

const TICK_RATE: f64 = 1.0 / 60.0;
const VISUAL_SLOWDOWN: f64 = 1.0;

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::DARK_GRAY))
        .insert_resource(WindowDescriptor {
            present_mode: PresentMode::AutoVsync,
            ..default()
        })
        .insert_resource(Msaa::default())
        .add_plugins(DefaultPlugins)
        .add_plugin(bevy_editor_pls::EditorPlugin)
        .add_startup_system(setup_graphics)
        .add_startup_system(setup)
        .add_system_set(
            SystemSet::new()
                //.with_run_criteria(FixedTimestep::step(TICK_RATE * VISUAL_SLOWDOWN))
                .with_system(symplectic_euler)
                .with_system(spring_impulse.before(symplectic_euler))
                .with_system(gravity.before(symplectic_euler)),
        )
        .register_type::<Impulse>()
        .register_type::<Gravity>()
        .register_type::<Mass>()
        .register_type::<Velocity>()
        .register_type::<SpringSettings>()
        .register_type::<springy::SpringBreak>()
        .register_type::<Option<springy::SpringBreak>>()
        .run();
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn(Camera2dBundle {
        transform: Transform::from_xyz(0.0, 300.0, 0.0),
        ..default()
    });
}

#[derive(Debug, Clone, Component)]
pub struct Spring {
    pub containing: Entity,
}

#[derive(Default, Debug, Clone, Component, Reflect)]
#[reflect(Component)]
pub struct SpringSettings(springy::SpringState<Vec2>);

#[derive(Default, Debug, Copy, Clone, Component, Reflect)]
#[reflect(Component)]
pub struct Velocity(Vec2);

#[derive(Default, Debug, Copy, Clone, Component, Reflect)]
#[reflect(Component)]
pub struct Impulse(Vec2);

#[derive(Debug, Copy, Clone, Component, Reflect)]
#[reflect(Component)]
pub struct Mass(pub f32);

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

#[derive(Debug, Copy, Clone, Component, Reflect)]
#[reflect(Component)]
pub struct Gravity(pub Vec2);

impl Default for Gravity {
    fn default() -> Self {
        Self(Vec2::new(0.0, -9.817))
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
        position.translation += velocity.0.extend(0.0) * TICK_RATE as f32;
        impulse.0 = Vec2::ZERO;
    }
}

pub fn gravity(time: Res<Time>, mut to_apply: Query<(&mut Impulse, &Gravity)>) {
    if time.delta_seconds() == 0.0 {
        return;
    }

    for (mut impulse, gravity) in &mut to_apply {
        impulse.0 += gravity.0;
    }
}

pub fn spring_impulse(
    mut commands: Commands,
    time: Res<Time>,
    mut impulses: Query<&mut Impulse>,
    mut springs: Query<(
        Entity,
        &GlobalTransform,
        &Velocity,
        &Mass,
        &mut SpringSettings,
        &Spring,
    )>,
    particle: Query<(&GlobalTransform, &Velocity, &Mass)>,
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
        mut spring_settings,
        spring,
    ) in &mut springs
    {
        let particle_entity = spring.containing;
        let (particle_transform, particle_velocity, particle_mass) =
            particle.get(particle_entity).unwrap();

        if particle_entity == spring_entity {
            continue;
        }

        match spring_settings.0.impulse(
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
        ) {
            springy::SpringResult::Broke => {
                commands.entity(spring_entity).remove::<Spring>();
            }
            springy::SpringResult::Impulse(impulse) => {
                let [mut spring_impulse, mut particle_impulse] = impulses
                    .get_many_mut([spring_entity, particle_entity])
                    .unwrap();

                spring_impulse.0 -= impulse;
                particle_impulse.0 += impulse;
            }
        }
    }
}

pub fn setup(mut commands: Commands) {
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
        .insert(SpriteBundle {
            sprite: sprite.clone(),
            ..default()
        })
        .insert((
            Velocity::default(),
            Impulse::default(),
            Mass::default(),
            Gravity::default(),
        ))
        .insert(Name::new("Cube 3"))
        .id();

    let cube_2 = commands
        .spawn()
        .insert(SpriteBundle {
            sprite: sprite.clone(),
            ..default()
        })
        .insert((
            Velocity::default(),
            Impulse::default(),
            Mass::default(),
            Gravity::default(),
        ))
        .insert(Name::new("Cube 2"))
        .insert(Spring { containing: cube_3 })
        .insert(SpringSettings(springy::SpringState {
            spring: springy::Spring {
                rest_distance: 50.0,
                limp_distance: 0.0,
                strength: 0.3,
                damp_ratio: 1.0,
            },
            breaking: Some(springy::SpringBreak {
                tear_force: 50.00,
                tear_step: 0.01,
                ..default()
            }),
            ..default()
        }))
        .id();

    let cube_1 = commands
        .spawn()
        .insert(SpriteBundle {
            sprite: sprite.clone(),
            ..default()
        })
        .insert(TransformBundle::from(Transform::from_xyz(50.0, 50.0, 0.0)))
        .insert((
            Velocity::default(),
            Impulse::default(),
            Mass::default(),
            Gravity::default(),
        ))
        .insert(Spring { containing: cube_2 })
        .insert(SpringSettings(springy::SpringState::new(springy::Spring {
            rest_distance: 50.0,
            limp_distance: 0.0,
            strength: 0.33,
            damp_ratio: 1.0,
        })))
        .insert(Name::new("Cube 1"))
        .id();

    let cube_slot = commands
        .spawn()
        .insert(SpriteBundle {
            sprite: slot.clone(),
            ..default()
        })
        .insert(TransformBundle::from(Transform::from_xyz(0.0, 300.0, 0.0)))
        .insert(Spring { containing: cube_1 })
        .insert(SpringSettings(springy::SpringState::new(springy::Spring {
            rest_distance: 50.0,
            limp_distance: 0.0,
            strength: 0.01,
            damp_ratio: 1.0,
        })))
        .insert((Velocity::default(), Impulse::default(), Mass(f32::INFINITY)))
        .insert(Name::new("Cube Slot"));

    let iterations = 100;
    for damped in 0..iterations {
        let size = 5.0;
        let damped_sprite = Sprite {
            color: Color::YELLOW,
            flip_x: false,
            flip_y: false,
            custom_size: Some(Vec2::new(size, size)),
            anchor: Default::default(),
        };

        let height = damped as f32 * (size + 1.0);
        let damped_cube = commands
            .spawn()
            .insert(SpriteBundle {
                sprite: damped_sprite.clone(),
                ..default()
            })
            .insert(TransformBundle::from(Transform::from_xyz(
                300.0, height, 0.0,
            )))
            .insert((Velocity::default(), Impulse::default(), Mass::default()))
            .insert(Name::new("Critical"))
            .id();

        let critical_slot = commands
            .spawn()
            .insert(SpriteBundle {
                sprite: slot.clone(),
                ..default()
            })
            .insert(TransformBundle::from(Transform::from_xyz(
                100.0, height, 0.0,
            )))
            .insert(Spring {
                containing: damped_cube,
            })
            .insert(SpringSettings(springy::SpringState::new(springy::Spring {
                rest_distance: 0.0,
                limp_distance: 0.0,
                strength: 0.05,
                damp_ratio: damped as f32 / 100.0 as f32,
            })))
            .insert((Velocity::default(), Impulse::default(), Mass(f32::INFINITY)))
            .insert(Name::new("Critical Slot"));
    }
}
