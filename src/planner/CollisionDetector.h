#include <iostream>

#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>

class CollisionDetector{
public:
  CollisionDetector();

  ~CollisionDetector();

  bool isColliding(const std::vector<double> manip_joints, std::string s_urdf_path);
};
