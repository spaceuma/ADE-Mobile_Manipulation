#include "MM_status.h"
#include "MobileManipMap.h"
#include "readMatrixFile.h"
#include <fstream>
#include <gtest/gtest.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

TEST(MMMapTest, constructorTest)
{
    // Input Elevation Matrix is read
    std::vector<std::vector<double>> vvd_elevation_data;
    readMatrixFile("test/unit/data/input/ColmenarRocks_smaller_10cmDEM.csv",
                   vvd_elevation_data);
    double res = 0.1; // meters

    ASSERT_EQ( vvd_elevation_data.size(), 80 );
    ASSERT_EQ( vvd_elevation_data[0].size(), 80 );

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

    MobileManipMap dummyMap;
    dummyMap.setRGDem((*prgd_dummy_dem));
    double d_elevation_min = dummyMap.getMinElevation();
    ASSERT_LT( d_elevation_min, 1008.55 );
    ASSERT_GT( d_elevation_min, 1008.53 );

    dummyMap.addSampleFacingObstacles(samplePos);
    /*dummyMap.showElevationMap();
    waitKey();
    dummyMap.showSlopeMap();
    waitKey();
    dummyMap.showObstacleMap();
    waitKey();*/
    std::vector<std::vector<double>> costMap;
    costMap.resize(prgd_dummy_dem->rows);
    for (uint i = 0; i < prgd_dummy_dem->rows; i++)
    {
        costMap[i].resize(prgd_dummy_dem->cols);
    }
    dummyMap.getCostMap(costMap);
    std::cout << " The size of the costMap is " << costMap.size() << " x "
              << costMap[0].size() << std::endl;
    // ASSERT_NO_THROW(dummyMap.setImageDem(decosDEM,0.028));
    // ASSERT_NO_THROW(dummyMap.showSlopeMap());

    // namedWindow("DECOS Map", WINDOW_AUTOSIZE);
    // imshow("DECOS Map", decosDEM);
    // waitKey();
    // destroyWindow("DECOS Map");
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
