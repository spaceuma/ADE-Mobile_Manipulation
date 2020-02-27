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


    proxy_library::Pose plpose_rover;
    plpose_rover.m_position.m_x = 3.0;
    plpose_rover.m_position.m_y = 2.0;


    double d_sample_pos_x = 5.3;
    double d_sample_pos_y = 5.6;

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

    MotionCommand mc;
    std::vector<JointState> vj_current_jointstates;
    vj_current_jointstates.resize(6);
    for (uint i = 0; i < 6; i++)
    {
        vj_current_jointstates[i].m_position = 0.0;
        vj_current_jointstates[i].m_speed = 0.0;
    }
    Joints j_current_joints(0, vj_current_jointstates);


    double d_zres = 0.08;
    std::ifstream if_urdf_path("data/urdfmodel_path.txt", std::ios::in);
    std::string s_urdf_path;
    if (if_urdf_path.is_open())
    {
        std::getline(if_urdf_path, s_urdf_path);
        std::cout << "urdf path is read from " << s_urdf_path << std::endl;
    }
    else
    {
        std::cout << "Cannot open urdfmodel_path.txt" << std::endl;
	throw "Cannot open urdf model path "; 
    }
    MobileManipMotionPlanner mmmotion_planner(rgd_dummy_dem, j_current_joints, d_zres, s_urdf_path);
    mmmotion_planner.generateMotionPlan(plpose_rover, d_sample_pos_x, d_sample_pos_y);
    mmmotion_planner.start();
    mmmotion_planner.printErrorCode();
    mmmotion_planner.printStatus();
    mmmotion_planner.pause(j_current_joints, mc);
    mmmotion_planner.printStatus();
    mmmotion_planner.resumeOperation();
    mmmotion_planner.printStatus();
  return 0;
}
