#include "KinematicModel.h"

int main(int argc, char * argv[])
{
    std::ifstream if_urdf_path("../../../data/planner/urdfmodel_path.txt", std::ios::in);
    std::string s_urdf_path;
    if(if_urdf_path.is_open())
    {
        std::getline(if_urdf_path, s_urdf_path);
        std::cout << "urdf path is read from " << s_urdf_path << std::endl;
    }

    KinematicModel_lib::Manipulator * manip = new KinematicModel_lib::Manipulator(s_urdf_path);
    manip->computeReachabilityMap(0.05, 0.04);

    return 0;
}
