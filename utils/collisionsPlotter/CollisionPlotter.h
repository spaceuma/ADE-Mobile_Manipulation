#include <iostream>

#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>

#include "MyWindow.hpp"

class CollisionPlotter{
public:
  CollisionPlotter();

  ~CollisionPlotter();

  bool isColliding(dart::simulation::WorldPtr mWorld,
                   const std::vector<double> manip_joints);
};
