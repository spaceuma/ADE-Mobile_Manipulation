#include "MyWindow.hpp"

#include "KinematicModel.h"
#include "CollisionDetector.h"
#include "mmFileManager.h"


int main(int argc, char* argv[]) 
{

  std::vector<double> position;
  readVectorFile("../position.txt", position);

  std::vector<double> orientation;
  readVectorFile("../orientation.txt", orientation);

  std::vector<double> manip_joints; 

  std::ifstream if_urdf_path("../../../data/planner/urdfmodel_path.txt", std::ios::in);
  std::string s_urdf_path;
  if (if_urdf_path.is_open())
  {
    std::getline(if_urdf_path, s_urdf_path);
    std::cout << "urdf path is read from " << s_urdf_path << std::endl;
  }

  KinematicModel_lib::Manipulator *manip = new KinematicModel_lib::Manipulator(s_urdf_path);
  //manip_joints = manip->getManipJoints(position, orientation,1,1);
  readVectorFile("../configuration.txt", manip_joints);

  CollisionDetector* p_collision_detector = new CollisionDetector(s_urdf_path, false);

  bool colliding = p_collision_detector->isColliding(manip_joints);

  if(colliding) std::cout<<"Collision detected!\n";

  // create a window and link it to the world
  MyWindow window;
  window.setWorld(p_collision_detector->mWorld);

  glutInit(&argc, argv);
  window.initWindow(640, 480, "Detect Collitions");
  glutMainLoop();
  return 0;
}
