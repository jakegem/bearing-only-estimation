#include "BearingEKF.hpp"
#include <iostream>
#include <cmath>

// Normalize angle to be within the range [-pi, pi]
double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

BearingEKF::BearingEKF() {
    // Initialize the state vector
    this->X << 50.0, 50.0, 0.0, 0.0;

    double dt = 0.1;

    this->F << 1.0, 0.0, dt, 0.0,
         0.0, 1.0, 0.0, dt,
         0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 1.0;
    
    this->Q << 0.01, 0.0, 0.0, 0.0,
         0.0, 0.01, 0.0, 0.0,
         0.0, 0.0, 0.01, 0.0,
         0.0, 0.0, 0.0, 0.01;
        
    this->R << 0.1;

    // Initialize the covariance matrix
    this->P << 10.0, 0.0, 0.0, 0.0,
         0.0, 10.0, 0.0, 0.0,
         0.0, 0.0, 100.0, 0.0,
         0.0, 0.0, 0.0, 100.0;

    this->sensorPosition << 0.0, 0.0;
}

BearingEKF::~BearingEKF() {std::cout << "Cleaning up BearingEKF..." << std::endl;}

Eigen::Matrix<double, 1, 4> BearingEKF::computeJacobian() {
    Eigen::Matrix<double, 1, 4> H;
    H << 0.0, 0.0, 0.0, 0.0;
    double x_s = this->sensorPosition(0);
    double y_s = this->sensorPosition(1);
    double x_t = this->X(0);
    double y_t = this->X(1);

    double dx = x_t - x_s;
    double dy = y_t - y_s;

    double denom = dx*dx + dy*dy;
    if (abs(denom) > 0.01) {
        H << -dy/denom, dx/denom, 0.0, 0.0;
    }
    
    return H;
}

double BearingEKF::bearingMeasurement() {
    double x_s = this->sensorPosition(0);
    double y_s = this->sensorPosition(1);
    double x_t = X(0);
    double y_t = X(1);

    double dx = x_t - x_s;
    double dy = y_t - y_s;

    return atan2(dy, dx);
}

void BearingEKF::predict() {
    // Prediction step
    X = F*X;
    P = F*P*F.transpose() + Q;
    std::cout << "prior: " << X.transpose() << std::endl;
}

void BearingEKF::update() {
    // Update step
    Eigen::Matrix<double, 1, 4> H = computeJacobian();
    std::cout << "H: " << H << std::endl; 
    std::cout << "bearing: " << this->currentBearing * 180.0 / M_PI << std::endl;
    std::cout << "sen_pos: " << this->sensorPosition << std::endl;
    double y = normalizeAngle(currentBearing - bearingMeasurement());
    std::cout << "innovation: " << y << std::endl;
    Eigen::Matrix<double, 1, 1> S = H*P*H.transpose() + R;
    Eigen::Matrix<double, 4, 1> K = P*H.transpose()*S.inverse();
    X += K*y;
    P = (Eigen::Matrix4d::Identity() - K*H)*P;
    std::cout << "posterior: " << X.transpose() << std::endl;
}



void BearingEKF::setBearing(const double bearing) {
    this->currentBearing = bearing;
}

void BearingEKF::setSensorPosition(const Eigen::Vector2d& pos) {
    this->sensorPosition = pos;
}


Eigen::Vector2d BearingEKF::getTargetPosition() {
    Eigen::Vector2d targetPosition;
    targetPosition << X(0), X(1);
    return targetPosition;
}