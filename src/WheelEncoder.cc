#include "WheelEncoder.h"

namespace ORB_SLAM2{

namespace WHEEL{


PulseCount::PulseCount(double _WheelLeft, double _WheelRight):
    WheelLeft(_WheelLeft), WheelRight(_WheelRight)
{}

Calib::Calib(float _eResolution, float _eLeftWheelDiameter, float _eRightWheelDiameter, float _eWheelBase):
    eResolution(_eResolution), eLeftWheelDiameter(_eLeftWheelDiameter),
    eRightWheelDiameter(_eRightWheelDiameter), eWheelBase(_eWheelBase)
{}

}
}