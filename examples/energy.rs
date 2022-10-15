use bevy::math::Vec3Swizzles;
use bevy::time::FixedTimestep;
use bevy::{prelude::*, window::PresentMode};

const TICK_RATE: f64 = 1.0 / 60.0;
const VISUAL_SLOWDOWN: f64 = 1.0;
const MIN_DAMP_RATIO: f32 = 0.00055;

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
        .run();
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn_bundle(Camera2dBundle {
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
    if time.delta_seconds() == 0.0 || time.delta_seconds() >= 1.0 {
        return;
    }

    for (mut position, mut velocity, mut impulse, mass) in &mut to_integrate {
        velocity.0 += impulse.0 * mass.inverse_mass();
        position.translation += velocity.0.extend(0.0) * TICK_RATE as f32;
        impulse.0 = Vec2::ZERO;
    }
}

pub fn gravity(time: Res<Time>, mut to_apply: Query<(&mut Impulse, &Gravity)>) {
    if time.delta_seconds() == 0.0 || time.delta_seconds() >= 1.0 {
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
    if time.delta_seconds() == 0.0 || time.delta_seconds() >= 1.0 {
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

        let (impulse, unit_vec) = spring_settings.0.impulse(
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
            previous_unit_vector.0,
        );

        let [mut spring_impulse, mut particle_impulse] = impulses
            .get_many_mut([spring_entity, particle_entity])
            .unwrap();

        spring_impulse.0 -= impulse;
        particle_impulse.0 += impulse;
        previous_unit_vector.0 = Some(unit_vec);
    }
}

pub fn setup(mut commands: Commands) {
    let size = 10.0;
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
        custom_size: Some(Vec2::new(size, size)),
        anchor: Default::default(),
    };

    let slot_location = Vec3::new(50.0, 50.0, 0.0);
    let original_location = Vec3::new(0.0, 0.0, 0.0);
    let location_step = Vec3::new(50.0, 0.0, 0.0);

    let from = 0.0002;
    let to = 0.0004;
    let step = 0.00001;

    let mut current = from;
    let mut index = 0;
    while (current < to) {
        let ratio = current;
        let slot_location = slot_location + (location_step * index as f32);
        let original_location = original_location + (location_step * index as f32);

        let starting = commands
            .spawn()
            .insert_bundle(SpriteBundle {
                sprite: slot.clone(),
                ..default()
            })
            .insert_bundle(TransformBundle::from(Transform::from_translation(
                original_location,
            )))
            .insert(Name::new(format!("{:?} Starting", ratio)));

        let cube_1 = commands
            .spawn()
            .insert_bundle(SpriteBundle {
                sprite: sprite.clone(),
                ..default()
            })
            .insert_bundle(TransformBundle::from(Transform::from_translation(
                original_location,
            )))
            .insert_bundle((
                Velocity::default(),
                Impulse::default(),
                Mass::default(),
                //Gravity::default(),
                PreviousUnitVector::default(),
            ))
            .insert(Name::new(format!("{:?} Cube", ratio)))
            .id();

        info!("trying min damping ratio of {:?}", ratio,);
        let cube_slot = commands
            .spawn()
            .insert_bundle(SpriteBundle {
                sprite: slot.clone(),
                ..default()
            })
            .insert_bundle(TransformBundle::from(Transform::from_translation(
                slot_location,
            )))
            .insert(Spring { containing: cube_1 })
            .insert(SpringSettings(springy::Spring {
                rest_distance: 0.0,
                limp_distance: 0.0,
                strength: 0.05,
                damp_ratio: ratio,
            }))
            .insert_bundle((
                Velocity::default(),
                Impulse::default(),
                Mass(f32::INFINITY),
                PreviousUnitVector::default(),
            ))
            .insert(Name::new(format!("{:?} Slot", ratio)));

        current += step;
        index += 1;
    }
}
