#include "ArmPlanner.h"
#include "CollisionDetector.h"
#include "KinematicModel.h"
#include "mmFileManager.h"

int main(int argc, char * argv[])
{
    std::ifstream if_urdf_path("../../../data/planner/urdfmodel_path.txt", std::ios::in);
    std::string s_urdf_path;
    if(if_urdf_path.is_open())
    {
        std::getline(if_urdf_path, s_urdf_path);
        std::cout << "urdf path is read from " << s_urdf_path << std::endl;
    }

    std::vector<std::vector<double>> armAuxProfile, armSweepingProfile;
    // armSweepingProfile.resize(7,std::vector<double>(6,0));

    readMatrixFile(s_urdf_path + "/../../test/unit/data/results/MMMotionPlanTest/"
                                 "nominal_working_profile_01.txt",
                   armAuxProfile);

    std::vector<double> nextConfiguration,
        lastConfiguration = armAuxProfile[armAuxProfile.size() - 1];

    // Point 1
    armSweepingProfile.push_back(lastConfiguration);

    KinematicModel_lib::Manipulator * manip = new KinematicModel_lib::Manipulator(s_urdf_path);
    ArmPlanner_lib::ArmPlanner * arm_planner = new ArmPlanner_lib::ArmPlanner(s_urdf_path);
    CollisionDetector * collision_detector = new CollisionDetector(s_urdf_path);

    std::vector<std::vector<double>> eeTransform = manip->getEETransform(lastConfiguration);

    // Point 2
    std::vector<double> position = {
        eeTransform[0][3] + 0.06, eeTransform[1][3] - 0.05, eeTransform[2][3]};
    std::vector<double> orientation = {pi, 0, 0};

    // nextConfiguration = manip->getManipJoints(position, orientation,1,1);
    nextConfiguration = manip->getManipJoints(position, orientation, lastConfiguration);

    if(collision_detector->isColliding(nextConfiguration) || std::isnan(nextConfiguration[0]) ||
       std::isnan(nextConfiguration[1]) || std::isnan(nextConfiguration[2]) ||
       std::isnan(nextConfiguration[3]) || std::isnan(nextConfiguration[4]) ||
       std::isnan(nextConfiguration[5]))
        std::cout << "Next config is colliding!\n";
    else
        armSweepingProfile.push_back(nextConfiguration);

    lastConfiguration = nextConfiguration;

    // Point 3
    position = {eeTransform[0][3] - 0.06, eeTransform[1][3] - 0.05, eeTransform[2][3]};

    // nextConfiguration = manip->getManipJoints(position, orientation,1,1);
    nextConfiguration = manip->getManipJoints(position, orientation, lastConfiguration);

    if(collision_detector->isColliding(nextConfiguration) || std::isnan(nextConfiguration[0]) ||
       std::isnan(nextConfiguration[1]) || std::isnan(nextConfiguration[2]) ||
       std::isnan(nextConfiguration[3]) || std::isnan(nextConfiguration[4]) ||
       std::isnan(nextConfiguration[5]))
        std::cout << "Next config is colliding!\n";
    else
        armSweepingProfile.push_back(nextConfiguration);

    lastConfiguration = nextConfiguration;

    // Point 4
    position = {eeTransform[0][3] - 0.06, eeTransform[1][3], eeTransform[2][3]};

    // nextConfiguration = manip->getManipJoints(position, orientation,1,1);
    nextConfiguration = manip->getManipJoints(position, orientation, lastConfiguration);

    if(collision_detector->isColliding(nextConfiguration) || std::isnan(nextConfiguration[0]) ||
       std::isnan(nextConfiguration[1]) || std::isnan(nextConfiguration[2]) ||
       std::isnan(nextConfiguration[3]) || std::isnan(nextConfiguration[4]) ||
       std::isnan(nextConfiguration[5]))
        std::cout << "Next config is colliding!\n";
    else
        armSweepingProfile.push_back(nextConfiguration);

    lastConfiguration = nextConfiguration;

    // Point 5
    position = {eeTransform[0][3] + 0.06, eeTransform[1][3], eeTransform[2][3]};

    // nextConfiguration = manip->getManipJoints(position, orientation,1,1);
    nextConfiguration = manip->getManipJoints(position, orientation, lastConfiguration);

    if(collision_detector->isColliding(nextConfiguration) || std::isnan(nextConfiguration[0]) ||
       std::isnan(nextConfiguration[1]) || std::isnan(nextConfiguration[2]) ||
       std::isnan(nextConfiguration[3]) || std::isnan(nextConfiguration[4]) ||
       std::isnan(nextConfiguration[5]))
        std::cout << "Next config is colliding!\n";
    else
        armSweepingProfile.push_back(nextConfiguration);

    lastConfiguration = nextConfiguration;

    // Point 6
    position = {eeTransform[0][3] + 0.06, eeTransform[1][3] + 0.05, eeTransform[2][3]};

    // nextConfiguration = manip->getManipJoints(position, orientation,1,1);
    nextConfiguration = manip->getManipJoints(position, orientation, lastConfiguration);

    if(collision_detector->isColliding(nextConfiguration) || std::isnan(nextConfiguration[0]) ||
       std::isnan(nextConfiguration[1]) || std::isnan(nextConfiguration[2]) ||
       std::isnan(nextConfiguration[3]) || std::isnan(nextConfiguration[4]) ||
       std::isnan(nextConfiguration[5]))
        std::cout << "Next config is colliding!\n";
    else
        armSweepingProfile.push_back(nextConfiguration);

    lastConfiguration = nextConfiguration;

    // Point 7
    position = {eeTransform[0][3] - 0.06, eeTransform[1][3] + 0.05, eeTransform[2][3]};

    // nextConfiguration = manip->getManipJoints(position, orientation,1,1);
    nextConfiguration = manip->getManipJoints(position, orientation, lastConfiguration);

    if(collision_detector->isColliding(nextConfiguration) || std::isnan(nextConfiguration[0]) ||
       std::isnan(nextConfiguration[1]) || std::isnan(nextConfiguration[2]) ||
       std::isnan(nextConfiguration[3]) || std::isnan(nextConfiguration[4]) ||
       std::isnan(nextConfiguration[5]))
        std::cout << "Next config is colliding!\n";
    else
        armSweepingProfile.push_back(nextConfiguration);

    // Obtaining times profile
    std::vector<double> timeProfile = arm_planner->getTimeProfile(&armSweepingProfile);

    // Saving sweeping profile
    saveProfile(&armSweepingProfile, s_urdf_path + "/sweepingProfile.txt");
    saveVector(&timeProfile, s_urdf_path + "/sweepingTimes.txt");

    return 0;
}
