#ifndef _PARTICLE_H_
#define _PARTICLE_H_

#include <iostream>

#include "Eigen/Eigen/Dense"
#define normalized_Z Vector3f(0, 0, 1)
#define MIN_DELTA_ANGLE 0.001f

using namespace std;
using namespace Eigen;

class Particle {
   private:
    float mass;

    Vector3f location;

    Quaternionf quat;

    Vector3f linear_velocity;

    Vector3f angular_velocity;

    Vector3f force_accum;

    Vector3f* hit_point;

    void set_quat(float angle, Vector3f axis, Quaternionf* quat_) {
        const float a = angle * 0.5f;
        const float s = (float)sin(a);
        const float c = (float)cos(a);
        quat_->w() = c;
        quat_->x() = axis.x() * s;
        quat_->y() = axis.y() * s;
        quat_->z() = axis.z() * s;
    }

   public:
    Particle(float mass_,
             Vector3f location_,
             Quaternionf quat_,
             Vector3f linear_velocity_,
             Vector3f angular_velocity_)
        : mass(mass_),
          location(location_),
          quat(quat_),
          linear_velocity(linear_velocity_),
          angular_velocity(angular_velocity_),
          hit_point(nullptr) {
        force_accum.setZero();
    }

    void set_linear_velocity(float idx, float vel) {
        linear_velocity(idx) = vel;
    }

    Vector3f get_location() { return location; }

    Quaternionf get_quat() { return quat; }

    float get_mass() { return mass; }

    void update_hit_point(Vector3f* hit_point_) { hit_point = hit_point_; }

    Vector3f* get_hit_point() { return hit_point; }

    Vector3f get_linear_velocity() { return linear_velocity; }

    Vector3f get_angular_velocity() { return angular_velocity; }

    void update_force(const Vector3f& force) { force_accum += force; }

    void apply_force(float delta_time) {
        // linear
        linear_velocity += delta_time * force_accum / mass;
        location += delta_time * linear_velocity;

        // reset
        force_accum.setZero();
    }

    void move(bool forward) {
        static char flag[2] = {-1, 1};
        linear_velocity(0) += flag[forward] * 20.f;
        linear_velocity(1) += flag[forward] * 20.f;
    }

    void turn(bool left, Vector3f torque, float delta_time, Matrix3f inertia) {
        static char flag[2] = {-1, 1};
        angular_velocity += delta_time * inertia.inverse() * torque;

        float angular_vel_norm, angular_delta_angle;
        angular_vel_norm = angular_delta_angle = 0.f;
        Vector3f angular_vel_axis;
        angular_vel_axis.setZero();
        if (!angular_velocity.isZero()) {
            angular_vel_norm = angular_velocity.norm();
            angular_delta_angle = angular_vel_norm * delta_time;
            angular_vel_axis = angular_velocity / angular_vel_norm;
        }

        Quaternionf delta_quat;
        set_quat(angular_delta_angle, angular_vel_axis, &delta_quat);
        quat = delta_quat * quat;
    }
};

#endif
