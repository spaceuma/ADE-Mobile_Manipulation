#include "Waypoint.hpp"
#include <vector>

namespace KinematicModel_lib
{

std::vector<std::vector<double>> dot(std::vector<std::vector<double>> A, std::vector<std::vector<double>> B);

std::vector<std::vector<double>> getTraslation(std::vector<double> position);

std::vector<std::vector<double>> getXrot(double angle);

std::vector<std::vector<double>> getYrot(double angle);

std::vector<std::vector<double>> getZrot(double angle);

double getDeterminant(std::vector<std::vector<double>> A);

std::vector<std::vector<double>> getCofactor(std::vector<std::vector<double>> A, int row, int col);

std::vector<std::vector<double>> getAdjoint(std::vector<std::vector<double>> A);

std::vector<std::vector<double>> getInverse(std::vector<std::vector<double>> A);

class Manipulator
{
private:
public:
    // -- PARAMETERS --
    double d0 = 0.500;
    double a1 = 0.225;
    double a2 = 0.735;
    double c2 = 0.030;
    double a3 = 0.030;
    double d4 = 0.695;
    double d6 = 0.300;

    // -- FUNCTIONS --
    std::vector<std::vector<double>> getEEtransform(std::vector<double> manipulatorJoints);

    std::vector<double>
    getManipJoints(std::vector<double> position, std::vector<double> orientation, int shoulder, int elbow);
};
}
