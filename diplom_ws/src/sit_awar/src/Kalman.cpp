#include <Kalman.h>

KalmanFilter::KalmanFilter(int stateSize_, int measSize_) :stateSize(stateSize_), measSize(measSize_)
{
    if (stateSize == 0 || measSize == 0)
    {
        throw "Error, State size and measurement size must bigger than 0\n";
    }

    x.resize(stateSize);
    x.setZero();

    F.resize(stateSize, stateSize);
    F.setIdentity();
    // F.topRightCorner(stateSize-measSize, stateSize-measSize) = Eigen::MatrixXd::Identity(stateSize-measSize, stateSize-measSize);

    P.resize(stateSize, stateSize);
    P.setIdentity();
    P.bottomRightCorner(stateSize-measSize, stateSize-measSize) = P.bottomRightCorner(stateSize-measSize, stateSize-measSize)*1000;
    P = P*10;

    H.resize(measSize, stateSize);
    H.setIdentity();
    
    z.resize(measSize);
    z.setZero();

    Q.resize(stateSize, stateSize);
    Q.setIdentity();
    Q.bottomRightCorner(stateSize-measSize, stateSize-measSize) = Q.bottomRightCorner(stateSize-measSize, stateSize-measSize)*0.1;

    R.resize(measSize, measSize);
    R.setIdentity();
}

void KalmanFilter::init(Eigen::VectorXd x_, Eigen::MatrixXd F_)
{
    x = x_;
    F = F_;
}

Eigen::VectorXd KalmanFilter::predict()
{
    x = F*x;
    Eigen::MatrixXd Ft = F.transpose();
    P = F*P*Ft + Q; 
    return x;
}

Eigen::VectorXd KalmanFilter::update(Eigen::VectorXd z_meas)
{
    Eigen::MatrixXd temp1, temp2,Ht;
    Ht = H.transpose();
    temp1 = H*P*Ht + R;
    temp2 = temp1.inverse();//(H*P*H'+R)^(-1)
    Eigen::MatrixXd K = P*Ht*temp2;
    z = H*x;
    x = x + K*(z_meas-z);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(stateSize, stateSize);
    P = (I - K*H)*P;
    return x;
}