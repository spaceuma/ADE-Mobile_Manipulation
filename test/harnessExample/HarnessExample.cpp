#include "MobileManipMap.h"
#include "MobileManipMotionPlanner.h"
#include <iostream>

using namespace std;

int main()
{
  RoverGuidance_Dem exampleDEM;
  cout << "Built dummy DEM" << endl;
  MobileManipMotionPlanner dummyPlanner(exampleDEM);
  cout << "Reached this line" << endl;
  return 0;
}
