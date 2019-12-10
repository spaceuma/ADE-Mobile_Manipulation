#ifndef _COUPLED_CONTROL_HPP_
#define _COUPLED_CONTROL_HPP_

#include <stdio.h> 
#include <stdlib.h> 
#include <iostream>
#include <vector>
#include <math.h>
#include <cstdlib>
#include "Waypoint.hpp"
#include "MotionCommand.h"

#define PI 3.1415

using namespace base;
using namespace proxy_library;
using namespace std;

namespace coupled_control
{
    class coupledControl
    {
        public: 
			int  findMaxValue(std::vector<float> vect);
            void modifyMotionCommand(double mMaxSpeed, double maxJW, std::vector<float>& jW, MotionCommand rover_command, MotionCommand& modified_rover_command);
			void selectNextManipulatorPosition(int current_waypoint, std::vector<int> assign, std::vector<double> armConfig, std::vector<double>& nextConfig, int negative);
			void manipulatorMotionControl(double gain, int& saturation, double mMaxSpeed, std::vector<double> nextConfig, std::vector<double> lastConfig, std::vector<float>& jW);
			double constrainAngle(double angle, int negative);

			
    };

} // end namespace coupled_control

#endif 
