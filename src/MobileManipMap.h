#ifndef __MOBILE_MANIP_MAP__
#define __MOBILE_MANIP_MAP__

#include <types/RoverGuidance_Dem.h>
#include <opencv2/opencv.hpp>
#include <math.h>
//#include "MotionPlan.h"

using namespace cv;
/**
 * This class includes the provide DEM, the cost map and the obstacles map.
 */
class MobileManipMap {

private:
	/**
	 * The mapping of the data (row/column) onto the 1D data arrays (single index) is defined as follows:
	 * * index = row * cols + col
	 */
  RoverGuidance_Dem rgDem;
  double offsetXYZ[3];
  unsigned int numXnodes;
  unsigned int numYnodes;
  double resDem;
  Mat matElevationMap;
  Mat slopeMap;
  Mat obstacleMap;
  std::vector<std::vector<double>> vecElevationMap;
  std::vector<std::vector<double>> currentObstaclesMap;
  std::vector<std::vector<double>> currentCostMap;

public:
	/**
	 * Constructor that receives the map, process it and generates the cost and obstacles maps.
	 */
  MobileManipMap();
  MobileManipMap(RoverGuidance_Dem dem);
  int setRGDem(RoverGuidance_Dem &dem);
  int setImageDem(Mat inputDem, double resDem);
  void showElevationMap();
  void showSlopeMap();
  void showObstacleMap();
  void getCostMap(std::vector<std::vector<double>> &costMap);
  void getElevationMap(std::vector<std::vector<double>> &elevationMap);
  std::vector<std::vector<double>>* getCostMapPointer();
  double getResolution(); 
private:
  bool calculateElevationMap();
  bool calculateSlopeMap();
	/**
	 * Based on currentDEM, it calculates or recalculates the currentCostMap.
	 */
  bool calculateCostMap();

	/**
	 * It calculates o recalculates the obstacles map based on currentDEM
	 */
  bool calculateObstacleMap();

};

#endif
