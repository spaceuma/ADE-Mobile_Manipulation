#ifndef __KINEMATIC_MODEL__
#define __KINEMATIC_MODEL__

#include "CollisionDetector.h"
#include "Waypoint.hpp"
#include "mmFileManager.h"
#include <exception>
#include <vector>

#define pi 3.14159265359

namespace KinematicModel_lib
{

std::vector<std::vector<double>> dot(std::vector<std::vector<double>> A,
                                     std::vector<std::vector<double>> B);

std::vector<double> dot(std::vector<std::vector<double>> A,
                        std::vector<double> b);

std::vector<double> dot(double n, std::vector<double> a);

std::vector<std::vector<double>> getTraslation(std::vector<double> position);

std::vector<std::vector<double>> getXrot(double angle);

std::vector<std::vector<double>> getYrot(double angle);

std::vector<std::vector<double>> getZrot(double angle);

double getDeterminant(const std::vector<std::vector<double>> *A);

std::vector<std::vector<double>> getCofactor(
    const std::vector<std::vector<double>> *A,
    int row,
    int col);

std::vector<std::vector<double>> getAdjoint(
    const std::vector<std::vector<double>> *A);

std::vector<std::vector<double>> getInverse(
    const std::vector<std::vector<double>> *A);

std::vector<double> getCrossProduct(std::vector<double> a,
                                    std::vector<double> b);

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

    // Manipulator Workspace
    double maxArmDistance = a2 + d4 + d6;
    double maxZArm = maxArmDistance + d0;
    double maxXYArm = maxArmDistance + a1;
    double maxArmOptimalDistance = maxArmDistance - d6;
    double minArmOptimalDistance = d6;
    double minArmDistance = 0.0;
    double optimalArmRadius
        = (maxArmOptimalDistance + minArmOptimalDistance) / 2;

    std::vector<double> initialConfiguration
        = {0.785398, -1.62075, 2.3405, -3.14159, 0.719747, 3.14159};

    std::vector<double> iniEEorientation = {0, pi / 2, pi / 4};

    // -- VARIABLES --
    std::vector<std::vector<std::vector<double>>> *reachabilityMap;
    std::vector<std::vector<std::vector<double>>> *reachabilityDistances;
    std::vector<double> *resolutions;
    std::vector<double> *minValues;
    std::vector<double> *maxValues;

    // -- FUNCTIONS --
    Manipulator();

    ~Manipulator();

    std::vector<std::vector<double>> getEETransform(
        std::vector<double> manipulatorJoints);

    std::vector<std::vector<double>> getWristTransform(
        std::vector<double> manipulatorJoints);

    std::vector<double> getManipJoints(std::vector<double> position,
                                       std::vector<double> orientation,
                                       int shoulder,
                                       int elbow);

    std::vector<double> getPositionJoints(std::vector<double> position,
                                          int shoulder,
                                          int elbow);

    std::vector<double> getWristJoints(std::vector<double> positionJoints,
                                       std::vector<double> orientation);

    std::vector<double> getManipJoints(std::vector<double> position,
                                       std::vector<double> orientation,
                                       std::vector<double> previousConfig);

    std::vector<std::vector<double>> getJacobianMatrix(
        std::vector<double> manipulatorJoints);

    void computeReachabilityMap(const double resXY, const double resZ);

    bool isReachable(std::vector<double> position);

    double getDistanceToCollision(std::vector<double> position);
};
} // namespace KinematicModel_lib
#endif
// namespace KinematicModel_lib
