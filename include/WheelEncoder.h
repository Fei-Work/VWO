#ifndef _WHEELENCODER_H_
#define _WHEELENCODER_H_

namespace ORB_SLAM2{

namespace WHEEL{

// WheelEncoder measure 
class PulseCount{
public:
    PulseCount(double _WheelLeft, double _WheelRight);
    double WheelLeft;
    double WheelRight;
};

// Calibration Data
class Calib{
public:
    Calib(float _eResolution, float _eLeftWheelDiameter, float _eRightWheelDiameter, float _eWheelBase);

    float eResolution;
    float eLeftWheelDiameter;
    float eRightWheelDiameter;
    float eWheelBase;
};


} // end of WHEEL
} // end of ORB_SLAM2
#endif