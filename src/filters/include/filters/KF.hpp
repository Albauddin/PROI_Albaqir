#pragma once
#include <Eigen/Dense>

class KalmanFilter {
public:
    KalmanFilter();
    void configure(double dt);

    void init(const Eigen::VectorXd &x0, const Eigen::MatrixXd &P0);
    void predict(const Eigen::Vector3d& u);           // u = [vx, vy, omega]
    void update(const Eigen::Vector3d& z);            // z = [x, y, theta]

    const Eigen::VectorXd& state() const { return x_hat_; }
    const Eigen::MatrixXd& covariance() const { return P_; }
private:
    double dt_;
    bool initialized_;

    Eigen::Matrix<double, 6, 6> A_; // State transition
    Eigen::Matrix<double, 6, 3> B_; // Control input
    Eigen::Matrix<double, 3, 6> C_; // Measurement

    Eigen::Matrix<double, 6, 6> Q_; // Process noise
    Eigen::Matrix3d R_;             // Measurement noise

    Eigen::VectorXd x_hat_;         // State estimate (6x1)
    Eigen::MatrixXd P_;             // Covariance estimate (6x6)
};
