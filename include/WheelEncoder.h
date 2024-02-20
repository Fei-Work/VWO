#ifndef _WHEELENCODER_H_
#define _WHEELENCODER_H_

#include<Eigen/Dense>
#include<opencv2/core/core.hpp>
#include<cmath>
#include<vector>

#include<Converter.h>


namespace ORB_SLAM2{

namespace WHEEL{
    
// Calibration Data
class Calibration{
public:
    Calibration(float _eResolution, float _eLeftWheelDiameter, float _eRightWheelDiameter, float _eWheelBase);

    float eResolution;
    float eLeftWheelDiameter;
    float eRightWheelDiameter;
    float eWheelBase;
};

// WheelEncoder measure 
class PulseCount{
public:
    PulseCount(double _time, double _WheelLeft, double _WheelRight);

    double time;
    double WheelLeft;
    double WheelRight;
};

class WheelEncoderDatas{
public:
    WheelEncoderDatas(const PulseCount mLastPulseCount, const std::vector<PulseCount> vPc, WHEEL::Calibration* WheelCalib);

    cv::Mat GetNewPose(const cv::Mat LastTwc);

    double during_time;
    double left_ditance;
    double right_distance;
    double left_velocity;
    double distance;
    double right_velocity;
    double base_velocity;
    double base_w;

    Eigen::Vector3d WheelBaseTranst;
    Eigen::Matrix3d WheelBaseTransR;
};

class WheelTransferInfo{
    double WheelCamTransfer;
};




} // end of WHEEL
} // end of ORB_SLAM2
#endif