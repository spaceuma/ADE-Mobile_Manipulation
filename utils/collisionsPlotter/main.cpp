#include "CollisionPlotter.h"


int main(int argc, char* argv[]) 
{
  std::vector<double> manip_joints = {20,0,90,0,30,0};

  // Create and initialize the world
  dart::simulation::WorldPtr mWorld
      = dart::utils::SkelParser::readWorld("dart://sample/skel/empty.skel");
  assert(mWorld != nullptr);
  Eigen::Vector3d gravity(0.0, -9.81, 0.0);
  mWorld->setGravity(gravity);

  CollisionPlotter plotter; 
  bool colliding = plotter.isColliding(mWorld, manip_joints);

  // create a window and link it to the world
  MyWindow window;
  window.setWorld(mWorld);

  glutInit(&argc, argv);
  window.initWindow(640, 480, "Detect Collitions");
  glutMainLoop();
  return 0;
}
