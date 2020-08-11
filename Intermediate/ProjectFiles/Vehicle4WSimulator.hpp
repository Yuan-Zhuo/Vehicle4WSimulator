#ifndef _VEHICLE4WSIMULATOR_HPP_
#define _VEHICLE4WSIMULATOR_HPP_

#include "Eigen/Eigen/Dense"
#include "Particle.hpp"
#include "ParticleForce.hpp"
#include "ParticleForceRegistry.hpp"
using namespace Eigen;

class Vehicle4WSimulator {
    Vector3f location;
    Vector3f body_box_extent;
    float wheel_radius;

    Particle* body;
    Particle* wheel[4];
    ParticleForceRegistry* permanent_registry;
    ParticleForceRegistry* temporary_registry;

    float sphere_inertia(float mass, float radius) {
        return 2.f / 5 * mass * radius * radius;
    }

   public:
    Vehicle4WSimulator(float body_mass_,
                       float wheel_mass_,
                       Vector3f location_,
                       Quaternionf quat_,
                       Vector3f body_box_extent_,
                       float wheel_radius_,
                       Vector3f linear_velocity_,
                       Vector3f angular_velocity_,
                       Vector3f body_relative_location_,
                       Vector3f* wheel_relative_location_arr_)
        : location(location_),
          body_box_extent(body_box_extent_),
          wheel_radius(wheel_radius_) {
        body = new Particle(body_mass_, body_relative_location_, quat_,
                            linear_velocity_, angular_velocity_);

        for (int i = 0; i < 4; i++) {
            wheel[i] =
                new Particle(wheel_mass_, wheel_relative_location_arr_[i],
                             quat_, linear_velocity_, angular_velocity_);
        }

        permanent_registry = new ParticleForceRegistry();
        temporary_registry = new ParticleForceRegistry();

        // // gravity
        float gravity_acc = 10.f;
        Vector3f gravity_acc_vec = -gravity_acc * normalized_Z;
        Gravity* fg_gravity_body = new Gravity(gravity_acc_vec);
        permanent_registry->add(body, fg_gravity_body);
        for (int i = 0; i < 4; i++) {
            Gravity* fg_gravity_wheel = new Gravity(gravity_acc_vec);
            permanent_registry->add(wheel[i], fg_gravity_wheel);
        }

        // spring
        float spring_constant = 100.f;
        float* normal_length_body = new float[4]();
        float* normal_length_wheel = new float[4]();
        for (int i = 0; i < 4; i++) {
            normal_length_body[i] =
                (body_relative_location_(2) -
                 (wheel_relative_location_arr_[i](2))) +
                ((body_mass_ / 4 * gravity_acc) / spring_constant);
            normal_length_wheel[i] = -normal_length_body[i];
        }

        Spring* fg_spring_body =
            new Spring(wheel, 4, spring_constant, normal_length_body);
        permanent_registry->add(body, fg_spring_body);

        for (int i = 0; i < 4; i++) {
            Spring* fg_spring_wheel =
                new Spring(&body, 1, spring_constant, normal_length_wheel + i);
            permanent_registry->add(wheel[i], fg_spring_wheel);
        }

        // contact
        float balance = (body_mass_ + 4 * wheel_mass_) * gravity_acc / 4;
        float loss_coeff = 0.2f;
        for (int i = 0; i < 4; i++) {
            Contact* fg_contact = new Contact(balance, loss_coeff);
            permanent_registry->add(wheel[i], fg_contact);
        }

        // friction
        float damping = 1.f;
        for (int i = 0; i < 4; i++) {
            Friction* fg_friction = new Friction(damping, gravity_acc);
            permanent_registry->add(wheel[i], fg_friction);
        }

        // constraint
        FrameConstraint* fg_frameconstraint = new FrameConstraint(wheel, 4);
        permanent_registry->add(body, fg_frameconstraint);
    }

    void apply(Vector3f** hit_point_arr, float delta_time) {
        // pre-set
        for (int i = 0; i < 4; i++) {
            if (!hit_point_arr[i]) {
                wheel[i]->update_hit_point(nullptr);
            } else {
                Vector3f hit_point = *(hit_point_arr[i]) - location;
                wheel[i]->update_hit_point(&hit_point);
            }
        }

        // update
        permanent_registry->update_forces(delta_time);

        // apply
        body->apply_force(delta_time);
        for (int i = 0; i < 4; i++) {
            wheel[i]->apply_force(delta_time);
        }
    }

    void move(bool forward) {
        body->move(forward);
        for (int i = 0; i < 4; i++) {
            wheel[i]->move(forward);
        }
    }

    void turn(bool left) {
        float turn_radius = 10.f;
        float force = 1000.f;
        bool direction = true;
        for (int i = 0; i < 4; i++) {
            Matrix3f inertia;
            float mass = wheel[i]->get_mass();
            float inertia_1d = sphere_inertia(mass, wheel_radius);
            cout << "inertia_1d: " << inertia_1d << endl;
            inertia << inertia_1d, 0, 0, 0, inertia_1d, 0, 0, 0, inertia_1d;
            Vector3f torque;
            torque.setZero();
            torque(2) = force * turn_radius;
            wheel[i]->turn(left, torque, 0.02f, inertia);
        }
    }

    Vector3f get_location() { return location; }

    Vector3f get_body_box_extent() { return body_box_extent; }

    Vector3f get_body_relative_location() { return body->get_location(); }

    Vector3f get_body_location() { return location + body->get_location(); }

    Quaternionf get_body_relative_quat() { return body->get_quat(); }

    Vector3f get_wheel_relative_location(int i) {
        return wheel[i]->get_location();
    }

    Vector3f get_wheel_location(int i) {
        return location + wheel[i]->get_location();
    }

    Quaternionf get_wheel_relative_quat(int i) { return wheel[i]->get_quat(); }

    Vector3f get_wheel_linear_velocity(int i) {
        return wheel[i]->get_linear_velocity();
    }

    float get_wheel_radius() { return wheel_radius; }
};

#endif
