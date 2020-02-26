#ifndef _COUPLED_CONTROL_HPP_
#define _COUPLED_CONTROL_HPP_

#include "MotionCommand.h"
#include "Waypoint.hpp"
#include <cstdlib>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

#define PI 3.1415

using namespace base;
using namespace proxy_library;
using namespace std;

namespace coupled_control
{
class coupledControl
{
public:
    int findMaxValue(std::vector<float> vect);

    void modifyMotionCommand(const double mMaxSpeed,
                             const std::vector<double> &vd_arm_abs_speed,
                             MotionCommand &rover_command);

    bool selectNextManipulatorPosition(
        unsigned int current_waypoint,
        std::vector<std::vector<double>> *armConfig,
        std::vector<double> *nextConfig,
        int negative);

    void manipulatorMotionControl(double gain,
                                  int &saturation,
                                  double mMaxSpeed,
                                  std::vector<double> nextConfig,
                                  std::vector<double> lastConfig,
                                  std::vector<double> &vd_arm_abs_speed);

    double constrainAngle(double angle, int negative);
};

} // end namespace coupled_control

#endif
