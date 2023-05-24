use bevy::math::Vec3Swizzles;
//use bevy::time::FixedTimestep;
use bevy::{prelude::*, window::PresentMode};

const TICK_RATE: f64 = 1.0 / 20.0;
const VISUAL_SLOWDOWN: f64 = 1.0;

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::DARK_GRAY))
        .insert_resource(Msaa::default())
        .add_plugins(DefaultPlugins)
        .add_plugin(bevy_editor_pls::EditorPlugin::new())
        .add_startup_system(setup_graphics)
        .add_startup_system(setup_rope)
        .add_startup_system(setup_translation)
        .add_startup_system(setup_rotational)
        .add_systems(
            (
                symplectic_euler,
                spring_impulse.before(symplectic_euler),
                gravity.before(symplectic_euler),
            ),
        )
        .register_type::<Impulse>()
        .register_type::<Gravity>()
        .register_type::<Mass>()
        .register_type::<Velocity>()
        .register_type::<SpringSettings>()
        .run();
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn(Camera2dBundle {
        transform: Transform::from_xyz(0.0, 300.0, 0.0),
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

#[derive(Default, Debug, Copy, Clone, Component, Reflect)]
#[reflect(Component)]
pub struct PreviousUnitVector(Option<Vec2>);

pub fn spring_impulse(
    time: Res<Time>,
    mut impulses: Query<&mut Impulse>,
    mut springs: Query<(
        Entity,
        &GlobalTransform,
        &Velocity,
        &Mass,
        &SpringSettings,
        &Spring,
        &mut PreviousUnitVector,
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
        let particle_a = 
            springy::TranslationParticle2 {
                mass: spring_mass.0,
                translation: spring_transform.translation().xy(),
                velocity: spring_velocity.0,
            };
        let particle_b = springy::TranslationParticle2 {
                mass: particle_mass.0,
                translation: particle_transform.translation().xy(),
                velocity: particle_velocity.0,
            };

        let instant = particle_a.instant(&particle_b);
        let impulse = spring_settings.0.impulse(
            timestep,
            instant,
        );

        let [mut spring_impulse, mut particle_impulse] = impulses
            .get_many_mut([spring_entity, particle_entity])
            .unwrap();

        spring_impulse.0 += impulse;
        particle_impulse.0 -= impulse;
    }
}

pub fn setup_rope(mut commands: Commands) {
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

    let cube_3 = commands
        .spawn(SpriteBundle {
            sprite: sprite.clone(),
            ..default()
        })
        .insert((
            Velocity::default(),
            Impulse::default(),
            Mass::default(),
            Gravity::default(),
            PreviousUnitVector::default(),
        ))
        .insert(Name::new("Cube 3"))
        .id();

    let cube_2 = commands
        .spawn(SpriteBundle {
            sprite: sprite.clone(),
            ..default()
        })
        .insert((
            Velocity::default(),
            Impulse::default(),
            Mass::default(),
            Gravity::default(),
            PreviousUnitVector::default(),
        ))
        .insert(Name::new("Cube 2"))
        .insert(Spring { containing: cube_3 })
        .insert(SpringSettings(springy::Spring {
            strength: 0.05,
            damp_ratio: 1.0,
        }))
        .id();

    let cube_1 = commands
        .spawn(SpriteBundle {
            sprite: sprite.clone(),
            ..default()
        })
        .insert(TransformBundle::from(Transform::from_xyz(50.0, 50.0, 0.0)))
        .insert((
            Velocity::default(),
            Impulse::default(),
            Mass::default(),
            Gravity::default(),
            PreviousUnitVector::default(),
        ))
        .insert(Spring { containing: cube_2 })
        .insert(SpringSettings(springy::Spring {
            strength: 0.05,
            damp_ratio: 1.0,
        }))
        .insert(Name::new("Cube 1"))
        .id();

    let cube_slot = commands
        .spawn(SpriteBundle {
            sprite: slot.clone(),
            ..default()
        })
        .insert(TransformBundle::from(Transform::from_xyz(0.0, 300.0, 0.0)))
        .insert(Spring { containing: cube_1 })
        .insert(SpringSettings(springy::Spring {
            strength: 0.05,
            damp_ratio: 1.0,
        }))
        .insert((
            Velocity::default(),
            Impulse::default(),
            Mass(f32::INFINITY),
            PreviousUnitVector::default(),
        ))
        .insert(Name::new("Cube Slot"));
    }


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

    let iterations = 500;
    let height = 500.0;
    for damped in 0..iterations {
        let size = height / iterations as f32;
        let damped_sprite = Sprite {
            color: Color::YELLOW,
            flip_x: false,
            flip_y: false,
            custom_size: Some(Vec2::new(5.0, size)),
            rect: None,
            anchor: Default::default(),
        };

        let height = damped as f32 * size;
        let damped_cube = commands
            .spawn(SpriteBundle {
                sprite: damped_sprite.clone(),
                ..default()
            })
            .insert(TransformBundle::from(Transform::from_xyz(
                300.0, height, 0.0,
            )))
            .insert((
                Velocity::default(),
                Impulse::default(),
                Mass::default(),
                PreviousUnitVector::default(),
            ))
            .insert(Name::new("Critical"))
            .id();

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
                damp_ratio: damped as f32 / iterations as f32,
            }))
            .insert((
                Velocity::default(),
                Impulse::default(),
                Mass(f32::INFINITY),
                PreviousUnitVector::default(),
            ))
            .insert(Name::new("Critical Slot"));
    }
}

pub fn setup_rotational(mut commands: Commands) {
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

    let iterations = 500;
    let height = 500.0;
    for damped in 0..iterations {
        let size = height / iterations as f32;
        let damped_sprite = Sprite {
            color: Color::YELLOW,
            flip_x: false,
            flip_y: false,
            custom_size: Some(Vec2::new(5.0, size)),
            rect: None,
            anchor: Default::default(),
        };

        let height = damped as f32 * size;
        let damped_cube = commands
            .spawn(SpriteBundle {
                sprite: damped_sprite.clone(),
                ..default()
            })
            .insert(TransformBundle::from(Transform::from_xyz(
                -300.0, height, 0.0,
            )))
            .insert((
                Velocity::default(),
                Impulse::default(),
                Mass::default(),
                PreviousUnitVector::default(),
            ))
            .insert(Name::new("Critical"))
            .id();

        let critical_slot = commands
            .spawn(SpriteBundle {
                sprite: slot.clone(),
                ..default()
            })
            .insert(TransformBundle::from(Transform::from_xyz(
                -300.0, height, 0.0,
            )))
            .insert(Spring {
                containing: damped_cube,
            })
            .insert(SpringSettings(springy::Spring {
                strength: 0.05,
                damp_ratio: damped as f32 / iterations as f32,
            }))
            .insert((
                Velocity::default(),
                Impulse::default(),
                Mass(f32::INFINITY),
                PreviousUnitVector::default(),
            ))
            .insert(Name::new("Critical Slot"));
    }
}


