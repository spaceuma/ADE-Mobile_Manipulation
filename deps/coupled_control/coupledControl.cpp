#include "coupledControl.hpp"

using namespace coupled_control;

void coupledControl::modifyMotionCommand(double mMaxSpeed,
                                         double maxJW,
                                         std::vector<float> &jW,
                                         MotionCommand rover_command,
                                         MotionCommand &modified_rover_command)
{
    // Speed adaption relation
    double R = mMaxSpeed / maxJW;

    std::cout << "Conversion relation: " << R << std::endl;
    // Addapt all the arm motion commands to the maximum speed of the real
    // motors
    for (unsigned int i = 0; i < jW.size(); i++)
        jW.at(i) = jW.at(i) * R;

    // Addapt the rover global speed to the speed of the arm
    double vA = rover_command.m_speed_ms;
    double vR = rover_command.m_turnRate_rads;
    modified_rover_command.m_speed_ms = vA * R;
    modified_rover_command.m_turnRate_rads = vR * R;
}

void coupledControl::selectNextManipulatorPosition(
    int current_waypoint,
    std::vector<std::vector<double>> *armConfig,
    std::vector<double> *nextConfig,
    int negative)
{
    // Selection of the next manipulator configuration depending on the current
    // waypoint (current_segment)
    int pointer = current_waypoint;

    for (unsigned int i = 0; i < nextConfig->size(); i++)
    {
        nextConfig->at(i) = constrainAngle((*armConfig)[pointer][i], negative);
    }
}

void coupledControl::manipulatorMotionControl(double gain,
                                              int &saturation,
                                              double mMaxSpeed,
                                              std::vector<double> nextConfig,
                                              std::vector<double> lastConfig,
                                              std::vector<float> &jW)
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

        std::cout << e << "  ";
        jW.at(i) = gain * e;
        if (abs(jW.at(i)) > mMaxSpeed) saturation = 1;
    }
    jW.at(0) = -jW.at(0);
    jW.at(1) = -jW.at(1);
    jW.at(3) = -jW.at(3);
    std::cout << std::endl;
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
