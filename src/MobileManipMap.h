#ifndef __MOBILE_MANIP_MAP__
#define __MOBILE_MANIP_MAP__

#include <math.h>
#include <opencv2/opencv.hpp>
#include <types/RoverGuidance_Dem.h>
//#include "MotionPlan.h"
#include "FastMarching.h"
#include "Waypoint.hpp"

using namespace cv;
using namespace FastMarching_lib;
/**
 * This class includes the provide DEM, the cost map and the obstacles map.
 */
class MobileManipMap
{

private:
    /**
     * The mapping of the data (row/column) onto the 1D data arrays (single
     * index) is defined as follows:
     * * index = row * cols + col
     */
    RoverGuidance_Dem rgDem;
    unsigned int ui_num_cols;
    unsigned int ui_num_rows;
    double d_res;
    double d_elevation_min;
    Mat slopeMap;
    Mat obstacleMap;
    Mat proximityMap;
    FastMarching fmShadower;
    std::vector<std::vector<double>> vvd_elevation_map;
    std::vector<std::vector<int>> vvi_traversability_map;
    std::vector<std::vector<double>> vvd_cost_map;
    std::vector<std::vector<double>> vvd_proximity_map;
    std::vector<std::vector<double>> currentObstaclesMap;
    std::vector<std::vector<double>> currentCostMap;

public:
    /**
     * Constructor that receives the map, process it and generates the cost and
     * obstacles maps.
     */
    MobileManipMap();
    MobileManipMap(const RoverGuidance_Dem &dem);
    int setRGDem(const RoverGuidance_Dem &dem);
    void setCostMap(std::vector<std::vector<double>> &costMap);
    void setElevationMap(std::vector<std::vector<double>> &elevationMap,
                         double res);
    void getCostMap(std::vector<std::vector<double>> &costMap);
    void getElevationMap(std::vector<std::vector<double>> &elevationMap);
    std::vector<std::vector<double>> *getCostMapPointer();
    double getResolution();
    double getMinElevation();
    bool addSampleFacingObstacles(base::Waypoint sample_pos);

private:
    bool calculateElevationMap();
    bool calculateTraversabilityMap();
    bool calculateProximityToObstaclesMap();
    void calculateCostValues();
    /**
     * Based on currentDEM, it calculates or recalculates the currentCostMap.
     */
    bool calculateCostMap();

    /**
     * It calculates o recalculates the obstacles map based on currentDEM
     */

    bool getSamplingCostMap(std::vector<std::vector<double>> &vvd_cost_map,
                            base::Waypoint w_sample);
};

#endif
