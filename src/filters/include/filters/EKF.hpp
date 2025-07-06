#pragma once
#include <Eigen/Dense>

class EKF {
public:
    EKF();
    void init(const Eigen::Vector3d& x0, const Eigen::Matrix3d& P0);
    void predict(double v, double omega, double dt, const Eigen::Matrix3d& Q);
    void update(const Eigen::Vector3d& z, const Eigen::Matrix3d& R);

    Eigen::Vector3d getState() const;
    Eigen::Matrix3d getCovariance() const;
private:
    Eigen::Vector3d x_; // state
    Eigen::Matrix3d P_; // covariance
};
