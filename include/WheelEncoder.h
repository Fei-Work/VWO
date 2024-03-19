#ifndef _WHEELENCODER_H_
#define _WHEELENCODER_H_

#include<Eigen/Dense>
#include<opencv2/core/core.hpp>
#include<cmath>
#include<vector>

#include<Converter.h>


namespace ORB_SLAM2{

namespace WHEEL{

Eigen::Matrix3f RightJacobianSO3(const Eigen::Vector3f &v);
Eigen::Matrix3f RightJacobianSO3(const float x, const float y, const float z);

    
// Calibration Data
class Calibration{
public:
    Calibration(){}
    Calibration(const Sophus::SE3f &Tbc);
    Calibration(const Sophus::SE3f &Tbc, const float &_eResolution, const float &_eLeftWheelDiameter, const float &_eRightWheelDiameter, const float & _eWheelBase)
    {
        Set(Tbc,_eResolution,_eLeftWheelDiameter, _eRightWheelDiameter, _eWheelBase);
    }

    Calibration(const Calibration &calib);

    //void Set(const cv::Mat &cvTbc, const float &ng, const float &na, const float &ngw, const float &naw);
    void Set(const Sophus::SE3<float> &sophTbc, const float &_eResolution, const float &_eLeftWheelDiameter, const float &_eRightWheelDiameter, const float & _eWheelBase);


    float eResolution;
    float eLeftWheelDiameter;
    float eRightWheelDiameter;
    float eWheelBase;

    Sophus::SE3<float> mTcb;
    Sophus::SE3<float> mTbc;

    Eigen::DiagonalMatrix<float,6> Cov;
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
public:    Preintegrated(const Calibration _mCalib);
    void IntegrateNewMeasurement(const Eigen::Vector3f &velocity, const float &base_w, const float &dt);
    cv::Mat GetRecentPose(const cv::Mat LastTwc);

    float dT;
    Eigen::Matrix<float,6,6> C;
    Eigen::Matrix<float,6,6> Info;
    Eigen::DiagonalMatrix<float,6> Nga, NgaWalk;

    // Values
    Eigen::Matrix3f dR;
    Eigen::Vector3f dP;
    Eigen::Vector3f avgA, avgW;

    Eigen::Matrix3f RVC;
    Eigen::Vector3f PVC;

    // Trans Tcb Info
    Calibration mCalib;
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