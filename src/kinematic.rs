use bevy::{
    prelude::*,
    reflect::Reflect,
};
//use bevy_inspector_egui::prelude::*;

pub trait Kinematic:
    std::ops::Sub<Self, Output = Self>
    + std::ops::Add<Self, Output = Self>
    + std::ops::Mul<Self, Output = Self>
    + std::ops::Mul<f32, Output = Self>
    + std::ops::Neg<Output = Self>
    + Sized
    + Copy
    + Send
    + Sync
    + std::fmt::Debug
    + Reflect
    + 'static
{
    fn length(self) -> f32;
    fn normalize_or_zero(self) -> Self;
    fn dot(self, other: Self) -> f32;
    fn inverse(self) -> Self;
}

impl Kinematic for f32 {
    fn length(self) -> f32 {
        self
    }
    fn normalize_or_zero(self) -> Self {
        1.0
    }
    fn dot(self, other: Self) -> f32 {
        self * other
    }
    fn inverse(self) -> Self {
        if self.is_normal() {
            1.0 / self
        } else {
            0.0
        }
    }
}

impl Kinematic for Vec2 {
    fn length(self) -> f32 {
        self.length()
    }
    fn normalize_or_zero(self) -> Self {
        self.normalize_or_zero()
    }
    fn dot(self, other: Self) -> f32 {
        self.dot(other)
    }
    fn inverse(self) -> Self {
        Vec2::new(self.x.inverse(), self.y.inverse())
    }
}

impl Kinematic for Vec3 {
    fn length(self) -> f32 {
        self.length()
    }
    fn normalize_or_zero(self) -> Self {
        self.normalize_or_zero()
    }
    fn dot(self, other: Self) -> f32 {
        self.dot(other)
    }
    fn inverse(self) -> Self {
        Vec3::new(self.x.inverse(), self.y.inverse(), self.z.inverse())
    }
}
