#include "MM_status.h"
#include "MobileManipMap.h"
#include "readMatrixFile.h"
#include <fstream>
#include <gtest/gtest.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

TEST(MMMapTest, NominalWorking)
{
    // Input Elevation Matrix is read
    std::vector<std::vector<double>> vvd_elevation_data;
    readMatrixFile("test/unit/data/input/ColmenarRocks_smaller_10cmDEM.csv",
                   vvd_elevation_data);
    double res = 0.1; // meters

    ASSERT_EQ(vvd_elevation_data.size(), 80);
    ASSERT_EQ(vvd_elevation_data[0].size(), 80);

    // A dummy Rover Guidance based DEM is created
    RoverGuidance_Dem *prgd_dummy_dem = new RoverGuidance_Dem;
    double dummyArray[vvd_elevation_data.size() * vvd_elevation_data[0].size()];
    prgd_dummy_dem->p_heightData_m = dummyArray;
    prgd_dummy_dem->cols = vvd_elevation_data[0].size();
    prgd_dummy_dem->rows = vvd_elevation_data.size();
    prgd_dummy_dem->nodeSize_m = res;
    for (uint j = 0; j < vvd_elevation_data.size(); j++)
    {
        for (uint i = 0; i < vvd_elevation_data[0].size(); i++)
        {
            prgd_dummy_dem->p_heightData_m[i + j * vvd_elevation_data[0].size()]
                = vvd_elevation_data[j][i];
        }
    }

    base::Waypoint samplePos;
    samplePos.position[0] = 5.3;
    samplePos.position[1] = 5.6;
    samplePos.heading = 0;

    MobileManipMap dummyMap((*prgd_dummy_dem), samplePos);
    double d_elevation_min = dummyMap.getMinElevation();
    ASSERT_LT(d_elevation_min, 1008.55);
    ASSERT_GT(d_elevation_min, 1008.53);

    std::vector<std::vector<double>> costMap;
    costMap.resize(prgd_dummy_dem->rows);
    for (uint i = 0; i < prgd_dummy_dem->rows; i++)
    {
        costMap[i].resize(prgd_dummy_dem->cols);
    }
    dummyMap.getCostMap(costMap);
    ASSERT_EQ(costMap.size(), 80);
    ASSERT_EQ(costMap[0].size(), 80);

    std::ofstream costMapFile;

    costMapFile.open("test/unit/data/results/costMap.txt");

    for (int j = 0; j < costMap.size(); j++)
    {
        for (int i = 0; i < costMap[0].size(); i++)
        {
            costMapFile << costMap[j][i] << " ";
        }
        costMapFile << "\n";
    }

    costMapFile.close();
}

TEST(MMMapTest, DEMformatError)
{
    // Input Elevation Matrix is read
    std::vector<std::vector<double>> vvd_elevation_data;
    readMatrixFile("test/unit/data/input/ColmenarRocks_smaller_10cmDEM.csv",
                   vvd_elevation_data);
    double res = 0.1; // meters

    // A dummy Rover Guidance based DEM is created
    RoverGuidance_Dem *prgd_dummy_dem = new RoverGuidance_Dem;
    double dummyArray[vvd_elevation_data.size() * vvd_elevation_data[0].size()];
    prgd_dummy_dem->p_heightData_m = dummyArray;
    prgd_dummy_dem->cols = vvd_elevation_data[0].size();
    prgd_dummy_dem->rows = vvd_elevation_data.size();
    for (uint j = 0; j < vvd_elevation_data.size(); j++)
    {
        for (uint i = 0; i < vvd_elevation_data[0].size(); i++)
        {
            prgd_dummy_dem->p_heightData_m[i + j * vvd_elevation_data[0].size()]
                = vvd_elevation_data[j][i];
        }
    }

    base::Waypoint samplePos;
    samplePos.position[0] = 5.3;
    samplePos.position[1] = 5.6;
    samplePos.heading = 0;

    // Error with resolution
    std::cout
        << "\033[32m[----------]\033[0m Testing exception with resolution value"
        << std::endl;
    prgd_dummy_dem->nodeSize_m = 0;
    ASSERT_THROW(MobileManipMap dummyMap((*prgd_dummy_dem), samplePos),
                 std::exception);
    prgd_dummy_dem->nodeSize_m = -0.1;
    ASSERT_THROW(MobileManipMap dummyMap((*prgd_dummy_dem), samplePos),
                 std::exception);
    prgd_dummy_dem->nodeSize_m = res;
    // Error with number of cols
    std::cout << "\033[32m[----------]\033[0m Testing exception with number of "
                 "columns"
              << std::endl;
    prgd_dummy_dem->cols = 0;
    ASSERT_THROW(MobileManipMap dummyMap((*prgd_dummy_dem), samplePos),
                 std::exception);
    prgd_dummy_dem->cols = vvd_elevation_data[0].size();
    // Error with number of rows
    std::cout
        << "\033[32m[----------]\033[0m Testing exception with number of rows"
        << std::endl;
    prgd_dummy_dem->rows = 0;
    ASSERT_THROW(MobileManipMap dummyMap((*prgd_dummy_dem), samplePos),
                 std::exception);
    prgd_dummy_dem->rows = vvd_elevation_data.size();
    // Error with the sample waypoint
    std::cout << "\033[32m[----------]\033[0m Testing exception with waypoint "
                 "out of range"
              << std::endl;
    samplePos.position[0] = 30000;
    ASSERT_THROW(MobileManipMap dummyMap((*prgd_dummy_dem), samplePos),
                 std::exception);
}
