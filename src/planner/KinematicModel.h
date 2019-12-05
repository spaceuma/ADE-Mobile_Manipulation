#ifndef __KINEMATIC_MODEL__
#define __KINEMATIC_MODEL__

#include "Waypoint.hpp"
#include <vector>

namespace KinematicModel_lib
{

std::vector<std::vector<double>> dot(std::vector<std::vector<double>> A, std::vector<std::vector<double>> B);

std::vector<double> dot(std::vector<std::vector<double>> A, std::vector<double> b);

std::vector<double> dot(double n, std::vector<double> a);

std::vector<std::vector<double>> getTraslation(std::vector<double> position);

std::vector<std::vector<double>> getXrot(double angle);

std::vector<std::vector<double>> getYrot(double angle);

std::vector<std::vector<double>> getZrot(double angle);

double getDeterminant(const std::vector<std::vector<double>> *A);

std::vector<std::vector<double>> getCofactor(const std::vector<std::vector<double>> *A, int row, int col);

std::vector<std::vector<double>> getAdjoint(const std::vector<std::vector<double>> *A);

std::vector<std::vector<double>> getInverse(const std::vector<std::vector<double>> *A);

std::vector<double> getCrossProduct(std::vector<double> a, std::vector<double> b);

std::vector<double> getSum(std::vector<double> a, std::vector<double> b);

std::vector<double> getDifference(std::vector<double> a, std::vector<double> b);

double getNorm(std::vector<double> a);

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

    std::vector<double> getManipJoints(std::vector<double> position,
                                       std::vector<double> orientation,
                                       int shoulder,
                                       int elbow);

    std::vector<double> getManipJoints(std::vector<double> position,
                                       std::vector<double> orientation,
                                       std::vector<double> previousConfig);

    std::vector<std::vector<double>> getJacobianMatrix(std::vector<double> manipulatorJoints);
};
}
#endif
// namespace KinematicModel_lib
