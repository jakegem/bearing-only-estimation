#ifndef BEARINGEKF_HPP
#define BEARINGEKF_HPP
#include <Eigen/Dense>

class BearingEKF {
public:
    BearingEKF();
    ~BearingEKF();
    
    void setSensorPosition(const Eigen::Vector2d& pos);
    void setBearing(const double bearing);
    Eigen::Vector2d getTargetPosition();

    Eigen::Matrix<double, 1, 4> computeJacobian();
    double bearingMeasurement();
    
    void predict();
    void update();

private:
    Eigen::Vector4d X;
    Eigen::Matrix4d P;
    Eigen::Matrix4d F;
    Eigen::Matrix4d Q;
    Eigen::Matrix<double, 1, 1> R;
    Eigen::Vector2d sensorPosition;
    double currentBearing;
};

#endif  // BEARINGEKF_HPP