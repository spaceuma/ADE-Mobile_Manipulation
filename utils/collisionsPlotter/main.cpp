#include "CollisionPlotter.h"


int main(int argc, char* argv[]) 
{
  std::vector<double> manip_joints = {20,0,90,0,30,0};

  // Create the world
  dart::simulation::WorldPtr mWorld
      = dart::simulation::World::create("Empty");

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
