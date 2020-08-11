#ifndef _PARTICLEFORCEREGISTRY_HPP_
#define _PARTICLEFORCEREGISTRY_HPP_

#include <vector>

#include "Eigen/Eigen/Dense"
#include "Particle.hpp"
#include "ParticleForce.hpp"

using namespace Eigen;

class ParticleForceRegistry {
   private:
    struct ParticleForceRegistration {
        Particle* particle;
        ForceGenerator* fg;

        ParticleForceRegistration(Particle* particle_, ForceGenerator* fg_)
            : particle(particle_), fg(fg_) {}
    };

    typedef std::vector<ParticleForceRegistration> Registry;
    Registry registrations;

   public:
    void add(Particle* particle, ForceGenerator* fg) {
        ParticleForceRegistration registration(particle, fg);
        registrations.push_back(registration);
    }

    void update_forces(float delta_time) {
        for (auto registry : registrations) {
            registry.fg->update_force(registry.particle, delta_time);
        }
    }
};

#endif
