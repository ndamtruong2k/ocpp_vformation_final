#include "kalman_filter.h"
#include <Eigen/Dense>
#include <math.h>
#include <chrono>


kalman_filter::kalman_filter(Eigen::Vector3d init_state) {

    start = std::chrono::high_resolution_clock::now();
	R << 5.11358507823672e-21, 0, 0,
         0, 0.0640413894873817, 0,
         0, 0, 0.0285362939024232;
    Accel_cov_mat << 0.00167538361225821, 0, 0, 0, 0, 0,
                     0, 0.00104809823951315, 0, 0, 0, 0,
                     0, 0, 0.00582366007389699, 0, 0, 0,
                     0, 0, 0, 0.00167538361225821, 0, 0,
                     0, 0, 0, 0, 0.00104809823951315, 0,
                     0, 0, 0, 0, 0, 0.00582366007389699;
    prev_state = init_state;
    prev_P << 1,0,0,1,0,0,
              0,1,0,0,1,0,
              0,0,1,0,0,1,
              1,0,0,1,0,0,
              0,1,0,0,1,0,
              0,0,1,0,0,1;
    H << 1,0,0,0,0,0,
         0,1,0,0,0,0,
         0,0,1,0,0,0;
    I << 1,0,0,0,0,0,
         0,1,0,0,0,0,
         0,0,1,0,0,0,
         0,0,0,1,0,0,
         0,0,0,0,1,0,
         0,0,0,0,0,1;
    end = std::chrono::high_resolution_clock::now();
    duration = end - start;
    delta_t = duration.count();
}
Eigen::Vector3d kalman_filter::predict(Eigen::Vector3d controlVec) {

    start = std::chrono::high_resolution_clock::now();
    A << 1,0,0,delta_t,0,0,
         0,1,0,0,delta_t,0,
         0,0,1,0,0,delta_t,
         0,0,0,1,0,0,
         0,0,0,0,1,0,
         0,0,0,0,0,1;
    B << (1 / 2) * std::pow(delta_t,2),0,0,
         0,(1 / 2) * std::pow(delta_t,2),0,
         0,0,(1 / 2) * std::pow(delta_t,2),
         delta_t,0,0,
         0,delta_t,0,
         0,0,delta_t;

    Q << (1 / 4) * std::pow(delta_t,4),0,0,(1 / 2) * std::pow(delta_t,3),0,0,
         0,(1 / 4) * std::pow(delta_t,4),0,0,(1 / 2) * std::pow(delta_t,3),0,
         0,0,(1 / 4) * std::pow(delta_t,4),0,0,(1 / 2) * std::pow(delta_t,3),
         (1 / 2) * std::pow(delta_t,3),0,0,delta_t,0,0,
         0,(1 / 2) * std::pow(delta_t,3),0,0,delta_t,0,
         0,0,(1 / 2) * std::pow(delta_t,3),0,0,delta_t;
    Q *= Accel_cov_mat;

    xk = A * prev_state + B * controlVec;
    P = A * prev_P * (A.transpose()) + Q;
    P(0, 1) = 0;
    P(0, 2) = 0;
    P(0, 4) = 0;
    P(0, 5) = 0;
    P(1, 0) = 0;
    P(1, 2) = 0;
    P(1, 3) = 0;
    P(1, 5) = 0;
    P(2, 0) = 0;
    P(2, 1) = 0;
    P(2, 3) = 0;
    P(2, 4) = 0;
    P(3, 1) = 0;
    P(3, 2) = 0;
    P(3, 4) = 0;
    P(3, 5) = 0;
    P(4, 0) = 0;
    P(4, 2) = 0;
    P(4, 3) = 0;
    P(4, 5) = 0;
    P(5, 0) = 0;
    P(5, 1) = 0;
    P(5, 3) = 0;
    P(5, 4) = 0;
    return xk;

}
void kalman_filter::update(Eigen::Vector3d pos) {

    Eigen::Matrix<double,3, 1> y = pos - H * xk;
    Eigen::Matrix<double,3, 3> S = H * P * (H.transpose()) + R;
    Eigen::Matrix<double,6, 3> K = P * (H.transpose()) * (S.inverse());
    prev_state = xk + K * y;
    prev_P = (I - K * H) * P;

    end = std::chrono::high_resolution_clock::now();
    duration = end - start;
    delta_t = duration.count();

}
void kalman_filter::update_P() {
    Eigen::Matrix<double, 3, 3> S = H * P * (H.transpose()) + R;
    Eigen::Matrix<double, 6, 3> K = P * (H.transpose()) * (S.inverse());
    prev_state = xk;
    prev_P = (I - K * H) * P;

    end = std::chrono::high_resolution_clock::now();
    duration = end - start;
    delta_t = duration.count();
}

kalman_filter::~kalman_filter() {

}