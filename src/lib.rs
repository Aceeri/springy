use bevy::prelude::*;

#[derive(Default, Debug, Copy, Clone, Component, Reflect)]
#[reflect(Component)]
pub struct Spring {
    /// Strength of the spring-like impulse. This is a range between 0 and 1
    /// where 1 will bring the spring to equilibrium in 1 timestep.
    pub strength: f32,
    /// Damping of the spring-like impulse. This is a range between 0 and 1
    /// where 1 will bring the spring to equilibrium in 1 timestep.
    pub damping: f32,
    /// Rest distance around the particle, it will try to push the particle out
    /// when too close.
    pub rest_distance: f32,
    /// Similar to rest distance except it will not push outwards if it is too close.
    pub limp_distance: f32,
}

#[derive(Default, Debug, Copy, Clone, Reflect)]
pub struct Particle {
    /// Mass of the particle, set to 0.0 or f32::INFINITY to set to `static`.
    pub mass: f32,
    /// Translation of the particle.
    pub position: Vec3,
    /// Current velocity of the particle.
    pub velocity: Vec3,
}

impl Particle {
    pub fn inverse_mass(&self) -> f32 {
        if mass != 0.0 {
            1.0 / mass
        } else {
            0.0
        }
    }

    pub fn position(&self) -> Vec3 {
        self.position
    }

    pub fn velocity(&self) -> Vec3 {
        self.velocity
    }
}

impl Spring {
    /// Impulse required to satisfy the spring constraint.
    ///
    /// This makes assumptions that the integrator for your physics is symplectic Euler.
    /// This allows us to make the spring stable with any provided user inputs by constraining
    /// the spring strength to the `reduced mass / timestep` and the damping to `reduced_mass`.
    pub fn impulse(&self, particle_a: Particle, particle_b: Particle) -> Vec3 {
        let distance = particle_b.position() - particle_a.position();
        let velocity = particle_b.velocity() - particle_a.velocity();

        let unit_vector = distance.normalize_or_zero();
        let distance_error = if self.limp_distance > distance.length() {
            0.0
        } else {
            unit_vector.dot(distance) - self.rest_distance
        };

        let distance_error = distance_error * unit_vector;
        let velocity_error = velocity;

        let reduced_mass = 1.0 / (particle_a.inverse_mass() + particle_b.inverse_mass());

        let distance_impulse = strength * distance_error * inverse_timestep * reduced_mass;
        let velocity_impulse = damping * velocity_error * reduced_mass;

        let impulse = -(distance_impulse + velocity_impulse);
        impulse
    }
}
