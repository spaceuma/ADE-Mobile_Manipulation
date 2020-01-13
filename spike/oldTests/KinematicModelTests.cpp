#include "KinematicModel.h"
#include "Waypoint.hpp"
#include "WaypointNavigation.hpp"
#include <ctime>
#include <fstream>
#include <gtest/gtest.h>
#include <math.h>

using namespace KinematicModel_lib;

TEST(KinematicModelTests, testingInverseComputation)
{
    std::vector<std::vector<double>> C{{1, 2, 0, 4}, {0, 6, 7, 8}, {9, 0, 11, 0}, {13, 14, 15, 16}};

    std::vector<std::vector<double>> inv{{22.0 / 219.0, -11.0 / 73.0, 2.0 / 73.0, 11.0 / 219.0},
                                         {-134.0 / 219.0, -6.0 / 73.0, -31.0 / 146.0, 85.0 / 438.0},
                                         {-6.0 / 73.0, 9.0 / 73.0, 5.0 / 73.0, -3.0 / 73.0},
                                         {155.0 / 292.0, 23.0 / 292.0, 29.0 / 292.0, -8.0 / 73.0}};
    EXPECT_EQ(inv, getInverse(&C));
}

TEST(KinematicModelTests, testingDKM)
{
    Manipulator dummyManipulator;

    std::vector<double> q{0, 2, 0, 3, 2.3, 1};
    std::vector<std::vector<double>> T{{-0.0484168, 0.312901, 0.948551, -0.0855248},
                                       {-0.883852, -0.455776, 0.105234, 0.0315702},
                                       {0.465255, -0.833283, 0.298626, -0.710708},
                                       {0.0, 0.0, 0.0, 1.0}};
    std::vector<std::vector<double>> Tres = dummyManipulator.getEEtransform(q);
    double sum = 0;
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            sum += abs(T[i][j] - Tres[i][j]);

    EXPECT_EQ(true, sum < 1e-4);
}

TEST(KinematicModelTests, testingIKM)
{
    Manipulator dummyManipulator;

    std::vector<double> pos{1, 1, 0.5};
    std::vector<double> orientation{0, 3.14159265359, 0};

    std::vector<double> q{0.785398, -0.812875, 1.16748, 0, 1.21619, 0.785398};
    std::vector<double> qres = dummyManipulator.getManipJoints(pos, orientation, 1, 1);
    std::vector<double> diff{abs(q[5] - qres[5]),
                             abs(q[5] - qres[5]),
                             abs(q[5] - qres[5]),
                             abs(q[5] - qres[5]),
                             abs(q[5] - qres[5]),
                             abs(q[5] - qres[5])};

    EXPECT_EQ(true, (diff[0] + diff[1] + diff[2] + diff[3] + diff[4] + diff[5]) < 1e-4);
}

TEST(KinematicModelTests, testingCLIK)
{
    Manipulator dummyManipulator;

    std::vector<double> pos{1, 1, 0.5};
    std::vector<double> orientation{0, 3.14159265359, 0};

    std::vector<double> q{0.785398, -0.812875, 1.16748, 0, 1.21619, 0.785398};
    std::vector<double> qi{0, -0.2, 0.2, 0, 0.2, 0};
    std::vector<double> qres = dummyManipulator.getManipJoints(pos, orientation, qi);
    std::vector<double> diff{abs(q[5] - qres[5]),
                             abs(q[5] - qres[5]),
                             abs(q[5] - qres[5]),
                             abs(q[5] - qres[5]),
                             abs(q[5] - qres[5]),
                             abs(q[5] - qres[5])};

    EXPECT_EQ(true, (diff[0] + diff[1] + diff[2] + diff[3] + diff[4] + diff[5]) < 1e-4);
}
