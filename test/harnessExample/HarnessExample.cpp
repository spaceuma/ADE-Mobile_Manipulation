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
    base::Waypoint w_rover_pos, w_sample_pos;

    w_rover_pos.position[0] = 3.0;
    w_rover_pos.position[1] = 2.0;
    w_rover_pos.heading = 0.0;
    w_sample_pos.position[0] = 5.3;
    w_sample_pos.position[1] = 5.6;

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
    mmmotion_planner.generateMotionPlan(w_rover_pos, w_sample_pos);
    mmmotion_planner.start();
    mmmotion_planner.printErrorCode();
    mmmotion_planner.printStatus();
  return 0;
}
