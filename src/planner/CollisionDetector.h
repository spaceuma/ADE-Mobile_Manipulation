#include <iostream>

#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>

class CollisionDetector{
private:
  dart::utils::DartLoader dl;
  // Create the world
  dart::simulation::WorldPtr mWorld; 
  dart::dynamics::SkeletonPtr sherpatt;
  dart::dynamics::SkeletonPtr manipulator;
public:
  CollisionDetector(std::string s_urdf_path);

  ~CollisionDetector();

  bool isColliding(const std::vector<double> manip_joints);
};
