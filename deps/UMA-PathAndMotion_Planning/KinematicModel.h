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

    //std::vector<double> armJointsMaxSpeed = {0.541052068,
    //                                         0.122173048,
    //                                         0.122173048,
    //                                         2.26892803,
    //                                         2.26892803,
    //                                         2.26892803};
    std::vector<double> armJointsMaxSpeed = {3.1416*3.0 / 180.0,
                                             3.1416*6.0 / 180.0,
                                             3.1416*6.0 / 180.0,
                                             3.1416*20.0 / 180.0,
                                             3.1416*20.0 / 180.0,
                                             3.1416*20.0 / 180.0};

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
        //= {0.8, -0.7, 1.0, 1.0, -1.1, 2.4};
        = {1.51, -0.9, 1.48, 0.0, -1.48, -2.7};

    std::vector<double> iniEEorientation = {0, pi/2, 0};

    // -- VARIABLES --
    std::vector<std::vector<std::vector<double>>> *reachabilityMap;
    std::vector<std::vector<std::vector<double>>> *reachabilityDistances;
    std::vector<double> *resolutions;
    std::vector<double> *minValues;
    std::vector<double> *maxValues;

    std::string s_data_path_m;
    // -- FUNCTIONS --
    Manipulator(std::string _s_data_path_m);

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
                                          int elbow,
					  double d_error_margin = 0.0);

    std::vector<double> getWristJoints(std::vector<double> positionJoints,
                                       std::vector<double> orientation);

    std::vector<double> getManipJoints(std::vector<double> position,
                                       std::vector<double> orientation,
                                       std::vector<double> previousConfig);

    std::vector<std::vector<double>> getJacobianMatrix(
        std::vector<double> manipulatorJoints);

    void computeReachabilityMap(const double resXY, const double resZ);

    int isReachable(std::vector<double> position);

    std::vector<double> getRelativePosition(std::vector<double> position);

    std::vector<double> getAbsolutePosition(std::vector<double> position);

    double getDistanceToCollision(std::vector<double> position);

    std::vector<double> getReachabilityMapSize();
    bool isFarFromLeg(double joint0, double d_z);
    bool isFarFromMast(double joint0, double joint1, double joint2);
};
} // namespace KinematicModel_lib
#endif
// namespace KinematicModel_lib
