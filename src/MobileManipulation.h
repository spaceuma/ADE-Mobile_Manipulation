#ifndef MOBILEMANIPULATION_H
#define MOBILEMANIPULATION_H

// Map
class Map
{
public:
  Map();
  updateDem();
  calculateCostMap();
  calculateObstaclesMap();
private:
  //RoverGuidance_Dem currentDem
  //currentCostMap -> float? Maybe use std::vector...
  //currentObstacleMap -> bool?
};


// Mobile Manipulation Motion Planner
class MobileManipMotionPlanner
{
public:
  MobileManipMotionPlanner();
  ExecuteMotion();
  generateMotionPlan();
  updateMap();
  getStatus();
  setStatus();
private:
  Map currentMap;
  // executor
  // status
};


// Motion Plan
class MotionPlan
{
private:
  //Vector of roverPos roverPath
  //Vector of Joints manipulatorJoints
  float sampleTime;
};


// Mobile Manipulation Motion Plan Execution
class MobileManipMotionPlanExecution
{
public:
  MobileManipMotionPlanExecution();
  int start();
  int stop();
private:
  MotionPlan currentMotionPlan;
  MobileManipMotionPlanner currentMMMP;
};

#endif
