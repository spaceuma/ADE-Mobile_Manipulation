#include "MMStatus.h"
#include "MobileManipMap.h"
#include "mmFileManager.h"
#include <fstream>
#include <gtest/gtest.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

TEST(MMMapTest, nominal_working_test)
{
    // Input Elevation Matrix is read
    std::vector<std::vector<double>> vvd_elevation_data;
    ASSERT_NO_THROW(readMatrixFile("test/unit/data/input/MMMapTest/ColmenarRocks_Nominal_10cmDEM.csv",
                   vvd_elevation_data)) << "Input DEM file is missing";
    double res = 0.1; // meters

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
    ASSERT_NO_THROW(samplePos = getWaypoint("test/unit/data/input/MMMapTest/sample_pos.txt")) << "Input Waypoint file is missing";

    MobileManipMap dummyMap((*prgd_dummy_dem));
    dummyMap.computeFACE(samplePos);

    double d_elevation_min = dummyMap.getMinElevation();
    //ASSERT_LT(d_elevation_min, 1008.55);
    //ASSERT_GT(d_elevation_min, 1008.53);

    std::vector<std::vector<double>> costMap;
    std::vector<std::vector<int>> traversabilityMap;
    costMap.resize(prgd_dummy_dem->rows);
    traversabilityMap.resize(prgd_dummy_dem->rows);
    for (uint i = 0; i < prgd_dummy_dem->rows; i++)
    {
        costMap[i].resize(prgd_dummy_dem->cols);
        traversabilityMap[i].resize(prgd_dummy_dem->cols);
    }
    dummyMap.getCostMap(costMap);
    ASSERT_EQ(costMap.size(), 80);
    ASSERT_EQ(costMap[0].size(), 80);

    std::ofstream costMapFile, traversabilityMapFile;

    costMapFile.open("test/unit/data/results/MMMapTest/costMap.txt");

    for (int j = 0; j < costMap.size(); j++)
    {
        for (int i = 0; i < costMap[0].size(); i++)
        {
            costMapFile << costMap[j][i] << " ";
        }
        costMapFile << "\n";
    }

    costMapFile.close();

    traversabilityMapFile.open("test/unit/data/results/MMMapTest/traversabilityMap.txt");
    dummyMap.getTraversabilityMap(traversabilityMap);
    for (int j = 0; j < traversabilityMap.size(); j++)
    {
        for (int i = 0; i < traversabilityMap[0].size(); i++)
        {
            traversabilityMapFile << traversabilityMap[j][i] << " ";
        }
        traversabilityMapFile << "\n";
    }

    traversabilityMapFile.close();

}

TEST(MMMapTest, nominal_working_test_2)
{
    // Input Elevation Matrix is read
    std::vector<std::vector<double>> vvd_elevation_data;
    ASSERT_NO_THROW(readMatrixFile("test/unit/data/input/MMMapTest/ColmenarRocks_splitted_10cmDEM.csv",
                   vvd_elevation_data)) << "Input DEM file is missing";
    double res = 0.1; // meters

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

    std::vector<std::vector<double>> costMap;
    costMap.resize(prgd_dummy_dem->rows);
    for (uint i = 0; i < prgd_dummy_dem->rows; i++)
    {
        costMap[i].resize(prgd_dummy_dem->cols);
    }
    std::ofstream costMapFile;
    MobileManipMap mmmap_no_shadowing((*prgd_dummy_dem));
    costMapFile.open("test/unit/data/results/MMMapTest/costMap_splittedMap.txt");
    mmmap_no_shadowing.getCostMap(costMap);
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

TEST(MMMapTest, dem_format_error_test)
{
    // Input Elevation Matrix is read
    std::vector<std::vector<double>> vvd_elevation_data;
    ASSERT_NO_THROW(readMatrixFile("test/unit/data/input/MMMapTest/ColmenarRocks_smaller_10cmDEM.csv",
                   vvd_elevation_data));
    double res = 0.1; // meters

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
    ASSERT_NO_THROW(samplePos = getWaypoint("test/unit/data/input/MMMapTest/sample_pos.txt")) << "Input Waypoint file is missing";

    // Error with resolution
    std::cout
        << "\033[32m[----------]\033[0m [INFO] Testing exception with resolution value"
        << std::endl;
    prgd_dummy_dem->nodeSize_m = 0;

    ASSERT_THROW(MobileManipMap dummyMap1((*prgd_dummy_dem)),
                 std::exception);
    prgd_dummy_dem->nodeSize_m = -0.1;
    ASSERT_THROW(MobileManipMap dummyMap2((*prgd_dummy_dem)),
                 std::exception);
    prgd_dummy_dem->nodeSize_m = res;
    // Error with number of cols
    std::cout << "\033[32m[----------]\033[0m [INFO] Testing exception with number of "
                 "columns"
              << std::endl;
    prgd_dummy_dem->cols = 0;
    ASSERT_THROW(MobileManipMap dummyMap3((*prgd_dummy_dem)),
                 std::exception);
    prgd_dummy_dem->cols = vvd_elevation_data[0].size();
    // Error with number of rows
    std::cout
        << "\033[32m[----------]\033[0m [INFO] Testing exception with number of rows"
        << std::endl;
    prgd_dummy_dem->rows = 0;
    ASSERT_THROW(MobileManipMap dummyMap4((*prgd_dummy_dem)),
                 std::exception);
    prgd_dummy_dem->rows = vvd_elevation_data.size();
}

TEST(MMMapTest, sample_pos_error_test)
{
    // Input Elevation Matrix is read
    std::vector<std::vector<double>> vvd_elevation_data;
    ASSERT_NO_THROW(readMatrixFile("test/unit/data/input/MMMapTest/ColmenarRocks_smaller_10cmDEM.csv",
                   vvd_elevation_data));
    double res = 0.1; // meters

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

    base::Waypoint w_sample_one, w_sample_two, w_sample_three, w_sample_four;
    w_sample_one.position[0] = -1.0;
    w_sample_one.position[1] = 5.6;
    w_sample_two.position[0] = 100;
    w_sample_two.position[1] = 5.6;
    w_sample_three.position[0] = 5.3;
    w_sample_three.position[1] = -1.0;
    w_sample_four.position[0] = 5.3;
    w_sample_four.position[1] = 100;
    
    MobileManipMap dummyMap((*prgd_dummy_dem));

    // Error with the sample waypoints
    std::cout << "\033[32m[----------]\033[0m [INFO] Testing exception with waypoint "
                 "out of range - First Sample"
              << std::endl;
    ASSERT_THROW(dummyMap.computeFACE(w_sample_one),
                 std::exception);
    std::cout << "\033[32m[----------]\033[0m [INFO] Testing exception with waypoint "
                 "out of range - Second Sample"
              << std::endl;
    ASSERT_THROW(dummyMap.computeFACE(w_sample_two),
                 std::exception);
    std::cout << "\033[32m[----------]\033[0m [INFO] Testing exception with waypoint "
                 "out of range - Third Sample"
              << std::endl;
    ASSERT_THROW(dummyMap.computeFACE(w_sample_three),
                 std::exception);
    std::cout << "\033[32m[----------]\033[0m [INFO] Testing exception with waypoint "
                 "out of range - Fourth Sample"
              << std::endl;
    ASSERT_THROW(dummyMap.computeFACE(w_sample_four),
                 std::exception);
}
