#include "filters/PF.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>

double angle_diff(double a, double b) {
    double d = a - b;
    while (d > M_PI) d -= 2*M_PI;
    while (d < -M_PI) d += 2*M_PI;
    return d;
}

ParticleFilter::ParticleFilter(int num_particles) : num_particles_(num_particles) {}

void ParticleFilter::init(double x, double y, double theta, double std[]) {
    std::normal_distribution<double> dist_x(x, std[0]);
    std::normal_distribution<double> dist_y(y, std[1]);
    std::normal_distribution<double> dist_theta(theta, std[2]);

    particles_.clear();
    for (int i = 0; i < num_particles_; ++i) {
        Particle p;
        p.x = dist_x(gen_);
        p.y = dist_y(gen_);
        p.theta = dist_theta(gen_);
        p.weight = 1.0;
        particles_.push_back(p);
    }
}

void ParticleFilter::predict(double v, double w, double dt, double std_pos[]) {
    for (auto& p : particles_) {
        double theta_new = p.theta + w * dt;
        if (fabs(w) > 1e-5) {
            p.x += v / w * (sin(theta_new) - sin(p.theta));
            p.y += v / w * (-cos(theta_new) + cos(p.theta));
        } else {
            p.x += v * dt * cos(p.theta);
            p.y += v * dt * sin(p.theta);
        }
        p.theta = theta_new;

        // Add noise
        std::normal_distribution<double> dist_x(0, std_pos[0]);
        std::normal_distribution<double> dist_y(0, std_pos[1]);
        std::normal_distribution<double> dist_theta(0, std_pos[2]);
        p.x += dist_x(gen_);
        p.y += dist_y(gen_);
        p.theta += dist_theta(gen_);
    }
}

// Accepts measurement as (x, y, theta)
void ParticleFilter::update(const Eigen::Vector3d& measurement, double std_landmark[]) {
    for (auto& p : particles_) {
        double dx = measurement.x() - p.x;
        double dy = measurement.y() - p.y;
        double dtheta = angle_diff(measurement.z(), p.theta);

        double weight = exp(-0.5 * (
            dx*dx / (std_landmark[0]*std_landmark[0]) +
            dy*dy / (std_landmark[1]*std_landmark[1]) +
            dtheta*dtheta / (std_landmark[2]*std_landmark[2])
        ));
        weight /= (2.0 * M_PI * std_landmark[0] * std_landmark[1] * std_landmark[2]);
        p.weight = weight;
    }
    normalizeWeights();
    resample();
}

void ParticleFilter::normalizeWeights() {
    double sum = 0.0;
    for (const auto& p : particles_) sum += p.weight;
    for (auto& p : particles_) p.weight /= (sum + 1e-9);
}

void ParticleFilter::resample() {
    std::vector<Particle> new_particles;
    std::vector<double> weights;
    for (const auto& p : particles_) weights.push_back(p.weight);
    std::discrete_distribution<> dist(weights.begin(), weights.end());

    for (int i = 0; i < num_particles_; ++i) {
        new_particles.push_back(particles_[dist(gen_)]);
    }
    particles_ = new_particles;
}

Eigen::Vector3d ParticleFilter::getBestEstimate() const {
    // Weighted mean (could also use max weight)
    double x = 0, y = 0, sin_sum = 0, cos_sum = 0;
    for (const auto& p : particles_) {
        x += p.x * p.weight;
        y += p.y * p.weight;
        sin_sum += sin(p.theta) * p.weight;
        cos_sum += cos(p.theta) * p.weight;
    }
    double theta = atan2(sin_sum, cos_sum);
    return Eigen::Vector3d(x, y, theta);
}

const std::vector<Particle>& ParticleFilter::getParticles() const {
    return particles_;
}
