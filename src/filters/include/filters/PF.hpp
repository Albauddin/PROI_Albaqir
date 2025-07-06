#pragma once
#include <vector>
#include <random>
#include <Eigen/Dense>

struct Particle {
    double x, y, theta, weight;
};

class ParticleFilter {
public:
    ParticleFilter(int num_particles = 100);
    void init(double x, double y, double theta, double std[]);
    void predict(double v, double w, double dt, double std_pos[]);
    void update(const Eigen::Vector3d& measurement, double std_landmark[]);
    Eigen::Vector3d getBestEstimate() const;
    const std::vector<Particle>& getParticles() const;

private:
    int num_particles_;
    std::vector<Particle> particles_;
    std::default_random_engine gen_;
    void normalizeWeights();
    void resample();
};
