#pragma once

#include <Eigen/Dense>
#include <iostream>

class KalmanFilter
{
private:
    int stateSize; //state variable's dimenssion
    int measSize; //measurement variable's dimession
    int uSize; //control variables's dimenssion
    Eigen::VectorXd x;
    Eigen::VectorXd z;
    Eigen::MatrixXd F;
    Eigen::MatrixXd P;//coveriance
    Eigen::MatrixXd H;
    Eigen::MatrixXd R;//measurement noise covariance
    Eigen::MatrixXd Q;//process noise covariance
public:
    KalmanFilter(int stateSize_, int measSize_);
    ~KalmanFilter(){}
    void init(Eigen::VectorXd x_, Eigen::MatrixXd F_);
    Eigen::VectorXd predict();
    Eigen::VectorXd update(Eigen::VectorXd z_meas);
};