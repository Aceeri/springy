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
        .add_startup_system(setup_graphics)
        .add_startup_system(setup_physics)
        .add_system_to_stage(CoreStage::PostUpdate, symplectic_euler)
        .add_system(spring_impulse)
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
pub struct SpringSettings {
    pub strength: f32,
    pub damping: f32,
    pub rest_distance: f32,
    pub limp_distance: f32,
}

#[derive(Default, Debug, Copy, Clone, Component, Reflect)]
#[reflect(Component)]
pub struct Velocity {
    pub linvel: Vec2,
}

#[derive(Default, Debug, Copy, Clone, Component, Reflect)]
#[reflect(Component)]
pub struct Impulse {
    pub impulse: Vec2,
}

#[derive(Debug, Copy, Clone, Component, Reflect)]
#[reflect(Component)]
pub struct Mass {
    mass: f32,
    inverse_mass: f32,
}

impl Mass {
    pub fn new(mass: f32) -> Mass {
        let inverse = if mass == 0.0 { 0.0 } else { 1.0 / mass };
        Mass {
            mass: mass,
            inverse_mass: inverse,
        }
    }

    pub fn mass(&self) -> f32 {
        self.mass
    }

    pub fn inverse_mass(&self) -> f32 {
        self.inverse_mass
    }
}

impl Default for Mass {
    fn default() -> Self {
        Self::new(1.0)
    }
}

pub fn symplectic_euler(
    time: Res<Time>,
    mut to_integrate: Query<(&mut Transform, &mut Velocity, &mut Impulse, &Mass)>,
) {
    if time.delta_seconds() == 0.0 {
        return;
    }

    for (mut position, mut velocity, mut impulse, mass) in &mut to_integrate {
        velocity.linvel += impulse.impulse * mass.inverse_mass();
        position.translation += Vec3::new(velocity.linvel.x, velocity.linvel.y, 0.0) * TICK_RATE;
        impulse.impulse = Vec2::ZERO;
    }
}

/*
// First we calculate the distance and velocity vectors.
vec2 distance = S->particle_b->position - S->particle_a->position;
vec2 velocity = S->particle_b->velocity - S->particle_a->velocity;
// We normalize the distance vector to get the unit vector.
S->unit_vector = distance.normalize();
// Now we calculate the distance and velocity errors.
float distance_error = S->unit_vector.dot( distance ) - S->rest_distance;
float velocity_error = S->unit_vector.dot( velocity );
// Now we use the spring equation to calculate the impulse.
float distance_impulse = C_S * distance_error * INVERSE_TIMESTEP;
float velocity_impulse = C_D * velocity_error;
float impulse = -( distance_impulse + velocity_impulse ) * S->reduced_mass;
// Finally, we apply opposite equal impulses to the endpoint particles.
S->particle_a->impulse -= impulse * S->unit_vector;
S->particle_b->impulse += impulse * S->unit_vector
*/
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
        let strength_max = reduced_mass / timestep;
        let damping_max = reduced_mass;

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
