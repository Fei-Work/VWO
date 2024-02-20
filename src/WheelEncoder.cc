#include "WheelEncoder.h"

namespace ORB_SLAM2{

namespace WHEEL{


PulseCount::PulseCount(double _time, double _WheelLeft, double _WheelRight):
    time(_time), 
    WheelLeft(_WheelLeft), WheelRight(_WheelRight)
{}

WheelEncoderDatas::WheelEncoderDatas(const PulseCount mLastPulseCount, const std::vector<PulseCount> vPc, WHEEL::Calibration* WheelCalib)
{
    int vPcSize = vPc.size();
    double Resolution = WheelCalib->eResolution;
    double LeftWheelDiameter = WheelCalib->eLeftWheelDiameter;
    double RightWheelDiameter = WheelCalib->eRightWheelDiameter;
    double WheelBase = WheelCalib->eWheelBase;
    
    double pi = M_PI;

    if(vPcSize>=1){
        // 此处设置为0的主要原因为可能视频数据到位前wheel还没开启
        if(mLastPulseCount.time == 0){
            during_time = (vPc[vPcSize-1].time -  vPc[0].time)/pow(10,9);
            left_ditance = (vPc[vPcSize-1].WheelLeft - vPc[0].WheelLeft)/Resolution * pi * LeftWheelDiameter;
            right_distance = (vPc[vPcSize-1].WheelRight - vPc[0].WheelRight)/Resolution * pi * RightWheelDiameter;
        }
        else{
            during_time = (vPc[vPcSize-1].time - mLastPulseCount.time)/pow(10,9);
            left_ditance = (vPc[vPcSize-1].WheelLeft - mLastPulseCount.WheelLeft)/Resolution * pi * LeftWheelDiameter;
            right_distance = (vPc[vPcSize-1].WheelRight - mLastPulseCount.WheelRight)/Resolution * pi * RightWheelDiameter;
        }
        distance = (left_ditance + right_distance)/2;

        left_velocity = left_ditance/during_time;
        right_velocity = right_distance/during_time;

        base_velocity = (left_velocity + right_velocity)/2;
        base_w = (left_velocity - right_velocity)/WheelBase;
    }
    else{
        during_time = 0;
        left_ditance = 0;
        right_distance = 0;
        left_velocity = 0;
        base_velocity = 0;
        base_w = 0;
    }
    

}

cv::Mat WheelEncoderDatas::GetNewPose(const cv::Mat LastTwc)
{
    Eigen::Matrix<double,3,3> LastR = Converter::toMatrix3d(LastTwc.rowRange(0,3).colRange(0,3));
    Eigen::Matrix<double,3,1> Lastt = Converter::toVector3d(LastTwc.rowRange(0,3).col(3));
    
    WheelBaseTranst(0) = 0;
    WheelBaseTranst(1) = 0;
    WheelBaseTranst(2) = -base_velocity * during_time;


    Eigen::Matrix<double,3,3> NewPoseR;

    Eigen::Vector3d axis(0, 1.0, 0);
    double base_theta = -base_w * during_time;
    Eigen::AngleAxis rotation(base_theta, axis);
    WheelBaseTransR = rotation.toRotationMatrix();
    NewPoseR = WheelBaseTransR*LastR ;

    Eigen::Matrix<double,3,1> NewPoset = WheelBaseTransR * Lastt + WheelBaseTranst;

    cv::Mat cvMat(4,4,CV_32F);
    for(int i=0;i<3;i++){
        for(int j=0; j<3; j++)
            cvMat.at<float>(i,j)=NewPoseR(i,j);
        cvMat.at<float>(i,3)=NewPoset(i);
    }
    cvMat.at<float>(3, 0) = 0;
    cvMat.at<float>(3, 1) = 0;
    cvMat.at<float>(3, 2) = 0;
    cvMat.at<float>(3, 3) = 1;

    return cvMat.clone();

}

void WheelEncoderDatas::clear()
{
    
}


Calibration::Calibration(float _eResolution, float _eLeftWheelDiameter, float _eRightWheelDiameter, float _eWheelBase):
    eResolution(_eResolution), eLeftWheelDiameter(_eLeftWheelDiameter),
    eRightWheelDiameter(_eRightWheelDiameter), eWheelBase(_eWheelBase)
{}

}
}