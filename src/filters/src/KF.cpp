#include "filters/KF.hpp"

KalmanFilter::KalmanFilter() : dt_(0.033), initialized_(false) {}

void KalmanFilter::configure(double dt) {
    dt_ = dt;

    // State transition matrix (constant velocity model)
    A_ = Eigen::Matrix<double, 6, 6>::Identity();
    A_(0,3) = dt_; // x += vx*dt
    A_(1,4) = dt_; // y += vy*dt
    A_(2,5) = dt_; // theta += omega*dt

    // Control matrix (direct addition of input velocities)
    B_ = Eigen::Matrix<double, 6, 3>::Zero();
    B_(3,0) = 1.0; // vx
    B_(4,1) = 1.0; // vy
    B_(5,2) = 1.0; // omega

    // Measurement model: observe x, y, theta
    C_ = Eigen::Matrix<double, 3, 6>::Zero();
    C_(0,0) = 1;
    C_(1,1) = 1;
    C_(2,2) = 1;

    // Process noise
    Q_ = 2.0 * Eigen::Matrix<double, 6, 6>::Identity();

    // Measurement noise
    R_ = Eigen::Matrix3d::Zero();
    R_(0,0) = 1.5 * 1.5;
    R_(1,1) = 1.5 * 1.5;
    R_(2,2) = 0.5 * 0.5;
}

void KalmanFilter::init(const Eigen::VectorXd &x0, const Eigen::MatrixXd &P0) {
    x_hat_ = x0;
    P_ = P0;
    initialized_ = true;
}

void KalmanFilter::predict(const Eigen::Vector3d& u) {
    if (!initialized_) return;
    // Prediction step
    x_hat_ = A_ * x_hat_ + B_ * u;
    P_ = A_ * P_ * A_.transpose() + Q_;
}

void KalmanFilter::update(const Eigen::Vector3d &z) {
    if (!initialized_) return;
    // Innovation
    Eigen::Vector3d y = z - C_ * x_hat_;
    y(2) = atan2(sin(y(2)), cos(y(2))); // Normalize angle

    Eigen::Matrix3d S = C_ * P_ * C_.transpose() + R_;
    Eigen::Matrix<double, 6, 3> K = P_ * C_.transpose() * S.inverse();

    // Update step
    x_hat_ = x_hat_ + K * y;
    P_ = (Eigen::Matrix<double, 6, 6>::Identity() - K * C_) * P_;
}
