#include "coupledControl.hpp"

using namespace coupled_control;

void coupledControl::modifyMotionCommand(const double mMaxSpeed,
                                         const std::vector<double> &vd_arm_abs_speed,
                                         MotionCommand &rover_command)
{
    // Speed adaptation ratio

    double d_max_abs_speed = *max_element(vd_arm_abs_speed.begin(), vd_arm_abs_speed.end());
    double R = mMaxSpeed / d_max_abs_speed;

    // Addapt the rover global speed to the speed of the arm
    /*double vA = rover_command.m_speed_ms;
    double vR = rover_command.m_turnRate_rads;
    modified_rover_command.m_speed_ms = vA * R;
    modified_rover_command.m_turnRate_rads = vR * R;*/
    rover_command.m_speed_ms *= R;
    rover_command.m_turnRate_rads *= R;
}

bool coupledControl::selectNextManipulatorPosition(
    unsigned int current_waypoint,
    std::vector<std::vector<double>> *armConfig,
    std::vector<double> *nextConfig,
    int negative)
{
    // Selection of the next manipulator configuration depending on the current
    // waypoint (current_segment)
    uint pointer = current_waypoint;

    for (unsigned int i = 0; i < nextConfig->size(); i++)
    {
        nextConfig->at(i) = constrainAngle((*armConfig)[pointer][i], negative);
    }

    // Returns true if it is the last position
    return pointer == (*armConfig).size() - 1;
}

void coupledControl::manipulatorMotionControl(double gain,
                                              int &saturation,
                                              double mMaxSpeed,
                                              std::vector<double> nextConfig,
                                              std::vector<double> lastConfig,
                                              std::vector<double> &vd_abs_speed)
{
    double e;

    std::cout << "Position error: ";
    for (unsigned int i = 0; i < nextConfig.size(); i++)
    {
        e = nextConfig.at(i) - lastConfig.at(i);

        if (e > PI)
            e = e - 2 * PI;
        else if (e < -PI)
            e = e + 2 * PI;
        vd_abs_speed.at(i) = abs(gain * e);
        if (abs(vd_abs_speed.at(i)) > mMaxSpeed)
	{
	    saturation = 1;
	}
    }
}

int coupledControl::findMaxValue(std::vector<float> vect)
{
    unsigned int max = 0;
    for (unsigned int i = 1; i < vect.size(); i++)
    {
        if (abs(vect.at(i)) > abs(vect.at(max))) max = i;
    }
    return max;
}

double coupledControl::constrainAngle(double angle, int negative)
{
    double c = cos(angle);
    double s = sin(angle);

    double na = atan2(s, c);
    if (negative)
    {
        if (na < -PI) na = na + 2 * PI;
        if (na > PI) na = na - 2 * PI;
    }
    else
    {
        if (na < 0) na = na + 2 * PI;
        if (na > 2 * PI) na = na - 2 * PI;
    }
    return na;
}
