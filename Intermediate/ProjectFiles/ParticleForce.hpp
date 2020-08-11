#ifndef _PARTICLEFORCE_HPP_
#define _PARTICLEFORCE_HPP_

#include <iostream>

#include "Eigen/Eigen/Dense"
#include "Particle.hpp"
using namespace std;
using namespace Eigen;

#define normalized_X Vector3f(1, 0, 0)
#define normalized_Y Vector3f(0, 1, 0)
#define normalized_Z Vector3f(0, 0, 1)
#define MAX_DEVIATION 1e-5f

class ForceGenerator {
   public:
    virtual void update_force(Particle* particle, float delta_time) = 0;
};

class Gravity : public ForceGenerator {
    Vector3f gravity;

   public:
    Gravity(const Vector3f& gravity) : gravity(gravity) {}

    virtual void update_force(Particle* particle, float delta_time) {
        particle->update_force(particle->get_mass() * gravity);
    }
};

class Spring : public ForceGenerator {
    Particle** other;
    int size;
    float spring_constant;
    float const* normal_length;

   public:
    Spring(Particle** other_,
           int size_,
           float spring_constant_,
           const float* normal_length_)
        : other(other_),
          size(size_),
          spring_constant(spring_constant_),
          normal_length(normal_length_) {}

    virtual void update_force(Particle* particle, float delta_time) {
        Vector3f force_sum;
        force_sum.setZero();

        for (int i = 0; i < size; i++) {
            float val =
                particle->get_location()(2) - other[i]->get_location()(2);
            val = val - normal_length[i];
            val *= spring_constant;
            Vector3f force = -val * normalized_Z;
            force_sum += force;
        }
        particle->update_force(force_sum);
    }
};

class Friction : public ForceGenerator {
    float damping;
    float gravity_acc;

   public:
    Friction(float damping_, float gravity_acc_)
        : damping(damping_), gravity_acc(gravity_acc_) {}

    virtual void update_force(Particle* particle, float delta_time) {
        if (!particle->get_hit_point())
            return;

        float force = damping * particle->get_mass() * gravity_acc;
        for (int i = 0; i < 2; i++) {
            float vel_proj = particle->get_linear_velocity()(i);
            if (abs(vel_proj) < (force / particle->get_mass()) * delta_time) {
                particle->set_linear_velocity(i, 0.f);

            } else {
                force *= (vel_proj > 0 ? -1 : 1);
                particle->update_force(force * Vector3f((i + 1) % 2, i % 2, 0));
            }
        }
    }
};

class FrameConstraint : public ForceGenerator {
    Particle** other;
    int size;

   public:
    FrameConstraint(Particle** other_, int size_)
        : other(other_), size(size_) {}

    virtual void update_force(Particle* particle, float delta_time) {
        for (int i = 0; i < 2; i++) {
            float vel_proj = 0.f;
            for (int j = 0; j < size; j++) {
                vel_proj += other[j]->get_linear_velocity()(i);
            }
            vel_proj /= size;
            particle->set_linear_velocity(i, vel_proj);
        }
    }
};

class Contact : public ForceGenerator {
    float balance;
    float loss_coeff;

   public:
    Contact(float balance_, float loss_coeff_)
        : balance(balance_), loss_coeff(loss_coeff_) {}

    virtual void update_force(Particle* particle, float delta_time) {
        Vector3f* hit_point = particle->get_hit_point();
        if (!hit_point)
            return;
        float force = 0.f;
        Vector3f linear_velocity = particle->get_linear_velocity();
        if (abs(linear_velocity(2)) < MAX_DEVIATION) {
            force = balance;
        } else {
            force = (1 + loss_coeff) * (-linear_velocity(2)) *
                        particle->get_mass() / delta_time +
                    balance;
        }
        particle->update_force(force * normalized_Z);
    }
};

#endif
