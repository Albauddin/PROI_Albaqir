#include "filters/EKF.hpp"
#include <cmath>

EKF::EKF() {}

void EKF::init(const Eigen::Vector3d& x0, const Eigen::Matrix3d& P0) {
    x_ = x0;
    P_ = P0;
}

void EKF::predict(double v, double omega, double dt, const Eigen::Matrix3d& Q) {
    double theta = x_(2);
    double dx = v * dt * std::cos(theta);
    double dy = v * dt * std::sin(theta);
    double dtheta = omega * dt;

    x_(0) += dx;
    x_(1) += dy;
    x_(2) += dtheta;

    Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
    F(0,2) = -v * dt * std::sin(theta);
    F(1,2) =  v * dt * std::cos(theta);

    P_ = F * P_ * F.transpose() + Q;
}

void EKF::update(const Eigen::Vector3d& z, const Eigen::Matrix3d& R) {
    Eigen::Matrix3d H = Eigen::Matrix3d::Identity();
    Eigen::Vector3d y = z - x_;
    Eigen::Matrix3d S = H * P_ * H.transpose() + R;
    Eigen::Matrix3d K = P_ * H.transpose() * S.inverse();
    x_ = x_ + K * y;
    P_ = (Eigen::Matrix3d::Identity() - K * H) * P_;
}

Eigen::Vector3d EKF::getState() const { return x_; }
Eigen::Matrix3d EKF::getCovariance() const { return P_; }
