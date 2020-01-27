#include "MobileManipMap.h"
#include "MobileManipMotionPlanner.h"
#include "mmFileManager.h"
#include <iostream>

using namespace std;

int main()
{
// Input Elevation Matrix is read
    std::vector<std::vector<double>> vvd_elevation_data;
    readMatrixFile("test/harnessExample/data/ColmenarRocks_smaller_10cmDEM.csv",
                   vvd_elevation_data);
    double res = 0.1; // meters
// A dummy Rover Guidance based DEM is created
    RoverGuidance_Dem rgd_dummy_dem;
    double dummyArray[vvd_elevation_data.size() * vvd_elevation_data[0].size()];
    rgd_dummy_dem.p_heightData_m = dummyArray;
    rgd_dummy_dem.cols = vvd_elevation_data[0].size();
    rgd_dummy_dem.rows = vvd_elevation_data.size();
    rgd_dummy_dem.nodeSize_m = res;
    for (uint j = 0; j < vvd_elevation_data.size(); j++)
    {
        for (uint i = 0; i < vvd_elevation_data[0].size(); i++)
        {
            rgd_dummy_dem.p_heightData_m[i + j * vvd_elevation_data[0].size()]
                = vvd_elevation_data[j][i];
        }
    }  
    MobileManipMotionPlanner mmmotion_planner(rgd_dummy_dem);
    mmmotion_planner.start();
    mmmotion_planner.printErrorCode();
  return 0;
}
