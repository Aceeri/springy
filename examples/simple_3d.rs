use bevy::{ecs::schedule::ScheduleLabel, prelude::*};
use springy::kinematic::Kinematic;

const TICK_RATE: f64 = 1.0 / 60.0;
const VISUAL_SLOWDOWN: f64 = 1.0;

#[derive(ScheduleLabel, PartialEq, Eq, PartialOrd, Ord, Hash, Debug, Clone, Copy)]
pub struct PhysicsSchedule;


#[derive(Resource, PartialEq, Eq, PartialOrd, Ord, Hash, Debug, Clone, Copy)]
pub struct Running(pub bool);

pub fn physics_step(world: &mut World) {
    let running = world.resource::<Running>();
    let input = world.resource::<Input<KeyCode>>();

    if running.0 || input.just_pressed(KeyCode::I) {
        world.run_schedule(PhysicsSchedule);
    }
}

pub fn toggle_running(mut running: ResMut<Running>, input: Res<Input<KeyCode>>) {
    if input.just_pressed(KeyCode::P) {
        running.0 = !running.0;
    }
}

fn main() {
    let mut app = App::new();

    app.insert_resource(ClearColor(Color::DARK_GRAY))
        .insert_resource(Msaa::default())
        .add_plugins(DefaultPlugins)
        .add_plugin(bevy_editor_pls::EditorPlugin::new())
        .insert_resource(bevy_framepace::FramepaceSettings {
            limiter: bevy_framepace::Limiter::Manual(std::time::Duration::from_secs_f64(TICK_RATE)),
            ..default()
        })
        .insert_resource(Running(false))
        .add_startup_system(setup_graphics)
        //.add_startup_system(setup_rope)
        .add_startup_system(setup_translation)
        .add_startup_system(setup_rotational)
        .add_startup_system(setup_rotation_test)
        .add_system(physics_step)
        .add_system(toggle_running)
        .register_type::<Impulse>()
        .register_type::<Gravity>()
        .register_type::<Inertia>()
        .register_type::<Velocity>()
        .register_type::<SpringSettings>();

    app.init_schedule(PhysicsSchedule);
    let physics_schedule = app.get_schedule_mut(PhysicsSchedule).unwrap();
    physics_schedule.add_systems((
        symplectic_euler,
        spring_impulse.before(symplectic_euler),
        gravity.before(symplectic_euler),
    ));

    app.run();
}

fn setup_graphics(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn(PbrBundle {
        mesh: meshes.add(shape::Plane::from_size(5.0).into()),
        material: materials.add(Color::rgb(0.3, 0.5, 0.3).into()),
        ..default()
    });
    commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(0.0, 3.0, 5.0).looking_at(Vec3::new(0.0, 2.0, 0.0), Vec3::Y),
        ..default()
    });
    commands.spawn(PointLightBundle {
        point_light: PointLight {
            intensity: 1500.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(4.0, 8.0, 4.0),
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
pub struct Velocity {
    pub linear: Vec3,
    pub angular: Vec3,
}

#[derive(Default, Debug, Copy, Clone, Component, Reflect)]
#[reflect(Component)]
pub struct Impulse {
    pub linear: Vec3,
    pub angular: Vec3,
}

#[derive(Debug, Copy, Clone, Component, Reflect)]
#[reflect(Component)]
pub struct Inertia {
    pub linear: f32,
    pub angular: Vec3,
}

impl Default for Inertia {
    fn default() -> Self {
        Self {
            linear: 1.0,
            angular: Vec3::splat(0.05),
        }
    }
}

impl Inertia {
    pub const INFINITY: Self = Inertia {
        linear: f32::INFINITY,
        angular: Vec3::splat(f32::INFINITY),
    };
}

#[derive(Debug, Copy, Clone, Component, Reflect)]
#[reflect(Component)]
pub struct Gravity(pub Vec3);

impl Default for Gravity {
    fn default() -> Self {
        Self(Vec3::new(0.0, -9.817, 0.0))
    }
}

/// Basic symplectic euler integration of the impulse/velocity/position.
pub fn symplectic_euler(
    time: Res<Time>,
    mut to_integrate: Query<(&mut Transform, &mut Velocity, &mut Impulse, &Inertia)>,
) {
    if time.delta_seconds() == 0.0 {
        return;
    }

    for (mut position, mut velocity, mut impulse, inertia) in &mut to_integrate {
        velocity.linear += impulse.linear * inertia.linear.inverse();
        velocity.angular += impulse.angular * inertia.angular.inverse();

        position.translation += velocity.linear * TICK_RATE as f32;

        // 𝑞𝑛𝑒𝑤=𝑞0+𝑡/2∗𝑤∗𝑞0
        //let q0 = position.rotation;
        //let t = TICK_RATE;
        //let new_rot = q0 * velocity.angular * TICK_RATE as f32;
        let ang = Quat::from_xyzw(
            0.0,
            velocity.angular.x,
            velocity.angular.y,
            velocity.angular.z,
        );
        /*
        * float sql = angVel.SqL(); // squared magnitude

        if (sql > FP_EPSILON2)
        {
            float invOmegaMag = 1.0f / sqrt(sql);
            sVec3 omegaAxis (angVel * invOmegaMag);
            float omegaAngle = invOmegaMag * sql * timestep;
            sQuat rotation; rotation.FromAxisAndAngle (omegaAxis, omegaAngle);
            sQuat newOrn = rotation * orn;
            newOrn.Normalize ();
            return newOrn;
        }
        */

        let sql = velocity.angular.length_squared();

        if sql > std::f32::EPSILON {
            let inv_omega_mag = 1.0 / sql.sqrt();
            let omega_axis = velocity.angular * inv_omega_mag;
            let omega_angle = inv_omega_mag * sql * TICK_RATE as f32;
            let rotation = Quat::from_axis_angle(omega_axis, omega_angle);
            let new_orn = rotation * position.rotation;
            position.rotation = new_orn;
        }

        impulse.linear = Vec3::ZERO;
        impulse.angular = Vec3::ZERO;
    }
}

pub fn gravity(time: Res<Time>, mut to_apply: Query<(&mut Impulse, &Gravity)>) {
    if time.delta_seconds() == 0.0 {
        return;
    }

    for (mut impulse, gravity) in &mut to_apply {
        impulse.linear += gravity.0;
    }
}

#[derive(Default, Debug, Copy, Clone, Component, Reflect)]
#[reflect(Component)]
pub struct PreviousUnitVector(Option<Vec3>);

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
        let particle_a = springy::TranslationParticle3 {
            mass: spring_mass.linear,
            translation: spring_translation,
            velocity: spring_velocity.linear,
        };

        let angular_particle_a = springy::AngularParticle3 {
            inertia: spring_mass.angular,
            direction: spring_rotation * Vec3::X,
            velocity: spring_velocity.angular,
        };

        let (_, particle_rotation, particle_translation) =
            particle_transform.to_scale_rotation_translation();
        let particle_b = springy::TranslationParticle3 {
            mass: particle_mass.linear,
            translation: particle_translation,
            velocity: particle_velocity.linear,
        };

        let angular_particle_b = springy::AngularParticle3 {
            inertia: particle_mass.angular,
            direction: particle_rotation * Vec3::X,
            velocity: particle_velocity.angular,
        };
        let instant = particle_a.instant(&particle_b);
        let impulse = spring_settings.0.impulse(timestep, instant);

        let angular_instant = angular_particle_a.instant(&angular_particle_b);
        let angular_impulse = -spring_settings.0.impulse(timestep, angular_instant);
        let [mut spring_impulse, mut particle_impulse] = impulses
            .get_many_mut([spring_entity, particle_entity])
            .unwrap();

        spring_impulse.linear += impulse;
        spring_impulse.angular += angular_impulse;
        particle_impulse.linear -= impulse;
        particle_impulse.angular -= angular_impulse;
    }
}

fn setup_rope(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let cube_3 = commands
        .spawn(PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Cube { size: 0.5 })),
            material: materials.add(Color::BLUE.into()),
            ..default()
        })
        .insert((
            Velocity::default(),
            Impulse::default(),
            Inertia::default(),
            Gravity::default(),
            PreviousUnitVector::default(),
        ))
        .insert(Name::new("Cube 3"))
        .id();

    let cube_2 = commands
        .spawn(PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Cube { size: 0.5 })),
            material: materials.add(Color::BLUE.into()),
            ..default()
        })
        .insert((
            Velocity::default(),
            Impulse::default(),
            Inertia::default(),
            Gravity::default(),
            PreviousUnitVector::default(),
        ))
        .insert(Name::new("Cube 2"))
        .insert(Spring { containing: cube_3 })
        .insert(SpringSettings(springy::Spring {
            strength: 0.5,
            damp_ratio: 1.0,
        }))
        .id();

    let cube_1 = commands
        .spawn(PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Cube { size: 0.5 })),
            material: materials.add(Color::BLUE.into()),
            ..default()
        })
        .insert(TransformBundle::from(Transform::from_xyz(50.0, 50.0, 0.0)))
        .insert((
            Velocity::default(),
            Impulse::default(),
            Inertia::default(),
            Gravity::default(),
            PreviousUnitVector::default(),
        ))
        .insert(Spring { containing: cube_2 })
        .insert(SpringSettings(springy::Spring {
            strength: 0.5,
            damp_ratio: 1.0,
        }))
        .insert(Name::new("Cube 1"))
        .id();

    let cube_slot = commands
        .spawn(PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Cube { size: 0.5 })),
            material: materials.add(Color::RED.into()),
            ..default()
        })
        .insert(TransformBundle::from(Transform::from_xyz(-3.0, 5.0, -3.0)))
        .insert(Spring { containing: cube_1 })
        .insert(SpringSettings(springy::Spring {
            strength: 0.5,
            damp_ratio: 1.0,
        }))
        .insert((
            Velocity::default(),
            Impulse::default(),
            Inertia::INFINITY,
            PreviousUnitVector::default(),
        ))
        .insert(Name::new("Cube Slot"));
}

fn setup_rotation_test(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
        let damped_cube = commands
            .spawn(PbrBundle {
                mesh: meshes.add(Mesh::from(shape::Cube { size: 0.5 })),
                material: materials.add(Color::YELLOW.into()),
                ..default()
            })
            .insert(TransformBundle::from(Transform::from_xyz(
                0.0, 1.0, 0.0,
            )))
            .insert((
                Velocity::default(),
                Impulse::default(),
                Inertia::default(),
                PreviousUnitVector::default(),
            ))
            .insert(Name::new("Test"))
            .id();

        let critical_slot = commands
            .spawn(PbrBundle {
                mesh: meshes.add(Mesh::from(shape::Cube { size: 0.01 })),
                material: materials.add(Color::RED.into()),
                ..default()
            })
            .insert(TransformBundle::from(Transform::from_xyz(
                0.0, 1.0, 0.0,
            )))
            .insert(Spring {
                containing: damped_cube,
            })
            .insert(SpringSettings(springy::Spring {
                strength: 1.0,
                damp_ratio: 0.0,
            }))
            .insert((
                Velocity::default(),
                Impulse::default(),
                Inertia::INFINITY,
                PreviousUnitVector::default(),
            ))
            .insert(Name::new("Test Slot"));
}

pub fn setup_translation(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let iterations = 50;
    let height = 10.0;
    for damped in 0..iterations {
        let size = height / iterations as f32;

        let height = damped as f32 * size;
        let damped_cube = commands
            .spawn(PbrBundle {
                mesh: meshes.add(Mesh::from(shape::Cube { size: size })),
                material: materials.add(Color::YELLOW.into()),
                ..default()
            })
            .insert(TransformBundle::from(Transform::from_xyz(
                10.0, height, 10.0,
            )))
            .insert((
                Velocity::default(),
                Impulse::default(),
                Inertia::default(),
                PreviousUnitVector::default(),
            ))
            .insert(Name::new(format!("Translational {}", height)))
            .id();

        let critical_slot = commands
            .spawn(PbrBundle {
                mesh: meshes.add(Mesh::from(shape::Cube { size: 0.01 })),
                material: materials.add(Color::RED.into()),
                ..default()
            })
            .insert(TransformBundle::from(Transform::from_xyz(0.0, height, 0.0)))
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
                Inertia::INFINITY,
                PreviousUnitVector::default(),
            ))
            .insert(Name::new(format!("Translational Slot {}", height)));
    }
}

pub fn setup_rotational(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let iterations = 500;
    let height = 10.0;
    for damped in 0..iterations {
        let size = height / iterations as f32;

        let height = damped as f32 * size;
        let damped_cube = commands
            .spawn(PbrBundle {
                mesh: meshes.add(Mesh::from(shape::Cube { size: size })),
                material: materials.add(Color::YELLOW.into()),
                ..default()
            })
            .insert(TransformBundle::from(Transform {
                translation: Vec3::new(10.0, height, 10.0),
                rotation: Quat::from_euler(EulerRot::XYZ, damped as f32 / 10.0, damped as f32 / 20.0, damped as f32 / 30.0),
                ..default()
            }))
            .insert((
                Velocity::default(),
                Impulse::default(),
                Inertia::default(),
                PreviousUnitVector::default(),
            ))
            .insert(Name::new(format!("Rotational {}", height)))
            .id();

        let critical_slot = commands
            .spawn(PbrBundle {
                mesh: meshes.add(Mesh::from(shape::Cube { size: 0.01 })),
                material: materials.add(Color::RED.into()),
                ..default()
            })
            .insert(TransformBundle::from(Transform::from_xyz(
                -1.0, height, -1.0,
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
                Inertia::INFINITY,
                PreviousUnitVector::default(),
            ))
            .insert(Name::new("Rotational Slot"));
    }
}