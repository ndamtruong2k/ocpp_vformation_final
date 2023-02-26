#pragma once
#include <Eigen/Dense>
#include <webots/Robot.hpp>
#include <chrono>


class kalman_filter {
private:
    std::chrono::time_point<std::chrono::steady_clock> start, end;
    std::chrono::duration<double> duration;
    double delta_t;
    Eigen::Matrix<double,6, 6> A;
    Eigen::Matrix<double,6, 3> B;
    Eigen::Matrix<double,6, 6> Q;
    Eigen::Matrix<double,6, 1> xk;
    Eigen::Matrix<double,6, 6> P;
    Eigen::Matrix<double,6, 6> prev_P;
    Eigen::Matrix<double,6, 6> prev_state;
    Eigen::Matrix3d R;
    Eigen::Matrix<double,6, 6> Accel_cov_mat;
    Eigen::Matrix<double,3, 6> H;
    Eigen::Matrix<int,6, 6> I;
public:
	kalman_filter(Eigen::Vector3d init_state);
	Eigen::Vector3d predict(Eigen::Vector3d controlVec);
	void update(Eigen::Vector3d pos);
	void update_P();
    ~kalman_filter();
};