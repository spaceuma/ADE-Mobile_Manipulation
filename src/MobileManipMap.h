#ifndef __MOBILE_MANIP_MAP__
#define __MOBILE_MANIP_MAP__

#include <types/RoverGuidance_Dem.h>
#include <opencv2/opencv.hpp>
#include <math.h>
//#include "MotionPlan.h"
#include "Waypoint.hpp"
#include "FastMarching.h"

using namespace cv;
using namespace FastMarching_lib;
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
  double d_res;
  Mat matElevationMap;
  Mat slopeMap;
  Mat obstacleMap;
  Mat proximityMap;
  FastMarching fmShadower;
  std::vector<std::vector<double>> vvd_elevation_map;
  std::vector<std::vector<bool>>   vvb_obstacle_map;
  std::vector<std::vector<double>> vvd_cost_map;
  std::vector<std::vector<double>> vvd_proximity_map;
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
  void setCostMap(std::vector<std::vector<double>> &costMap);
  void setElevationMap(std::vector<std::vector<double>> &elevationMap, double res);
  int setImageDem(Mat inputDem, double resDem);
  void showElevationMap();
  void showSlopeMap();
  void showObstacleMap();
  void getCostMap(std::vector<std::vector<double>> &costMap);
  void getElevationMap(std::vector<std::vector<double>> &elevationMap);
  std::vector<std::vector<double>>* getCostMapPointer();
  double getResolution(); 
  bool addSampleFacingObstacles(base::Waypoint sample_pos);
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


  bool getSamplingCostMap(std::vector<std::vector<double>>& vvd_cost_map, base::Waypoint w_sample);
  bool calculateProximityToObstaclesMap();
};

#endif
