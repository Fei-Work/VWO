#ifndef _WHEELENCODER_H_
#define _WHEELENCODER_H_

#include<Eigen/Dense>
#include<opencv2/core/core.hpp>
#include<cmath>
#include<vector>
// #include<sophus/se3.hpp>

#include<Converter.h>


namespace ORB_SLAM2{

namespace WHEEL{

Eigen::Matrix3d RightJacobianSO3(const Eigen::Vector3d &v);
Eigen::Matrix3d RightJacobianSO3(const double x, const double y, const double z);

    
// Calibration Data
class Calibration{
public:
    Calibration(float _eResolution, float _eLeftWheelDiameter, float _eRightWheelDiameter, float _eWheelBase);

    float eResolution;
    float eLeftWheelDiameter;
    float eRightWheelDiameter;
    float eWheelBase;
};

class Vehicle2StereoInfo{
public:
    Vehicle2StereoInfo(cv::Mat T);
    cv::Mat GetVehicle2StereoP();
    Eigen::Matrix3d GetVehicle2StereoR();
    Eigen::Vector3d GetVehicle2Stereot();

private:
    cv::Mat P;
    Eigen::Matrix3d  R;
    Eigen::Vector3d  t;
};

// WheelEncoder measure 
class PulseCount{
public:
    PulseCount();
    PulseCount(double _time, double _WheelLeft, double _WheelRight);

    double time;
    double WheelLeft;
    double WheelRight;
};

class Preintegrated{
public:
    Preintegrated();
    void IntegrateNewMeasurement(const Eigen::Vector3d &velocity, const double &base_w, const float &dt);
    cv::Mat GetRecentPose(const cv::Mat LastTwc, Vehicle2StereoInfo* mpV2S);

    float dT;
    Eigen::Matrix<double,6,6> C;
    Eigen::Matrix<double,6,6> Info;
    Eigen::DiagonalMatrix<double,6> Nga, NgaWalk;

    // Values
    Eigen::Matrix3d dR;
    Eigen::Vector3d dP;
    Eigen::Vector3d avgA, avgW;

    Eigen::Matrix3d RVC;
    Eigen::Vector3d PVC;
};


class WheelEncoderDatas{
public:
    WheelEncoderDatas(const PulseCount mLastPulseCount, std::vector<PulseCount> vPc, WHEEL::Calibration* WheelCalib);

    cv::Mat GetNewPose(const cv::Mat LastTwc);
    
    void clear();

    double during_time;
    double distance;
    
    Eigen::Vector3d WheelBaseTranst;
    Eigen::Matrix3d WheelBaseTransR;
};


} // end of WHEEL
} // end of ORB_SLAM2
#endif