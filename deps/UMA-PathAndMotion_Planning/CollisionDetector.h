#ifndef __COLLISION_DETECTOR__
#define __COLLISION_DETECTOR__

#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>
#include <iostream>

class CollisionDetector
{
private:
    dart::utils::DartLoader dl;
    // Create the world
    dart::dynamics::SkeletonPtr sherpatt;
    dart::dynamics::SkeletonPtr manipulator;

public:
    dart::simulation::WorldPtr mWorld;
    CollisionDetector(std::string s_urdf_path);

    ~CollisionDetector();

    bool isColliding(const std::vector<double> manip_joints);
};

#endif
