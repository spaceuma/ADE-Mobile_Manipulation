#ifndef __MOBILE_MANIP_MAP__
#define __MOBILE_MANIP_MAP__

#include "FastMarching.h"
#include "Waypoint.hpp"
#include <math.h>
#include <opencv2/opencv.hpp>
#include <types/RoverGuidance_Dem.h>

using namespace cv;
using namespace FastMarching_lib;
/**
 * This class includes all information concerning the map (elevation, cost...)
 */
class MobileManipMap
{

private:
    /**
     * The mapping of the data (row/column) onto the 1D data arrays (single
     * index) is defined as follows:
     * * index = row * cols + col
     */
    RoverGuidance_Dem rg_dem;
    /**
     * Number of map columns
     */
    unsigned int ui_num_cols;
    /**
     * Number of map rows
     */
    unsigned int ui_num_rows;
    /**
     * Map resolution in meters
     */
    double d_res;
    /**
     * Radius of sampling outter ring
     */
    double d_outter_sampling_dist;
    /**
     * Radius of sampling inner ring
     */
    double d_inner_sampling_dist;
    /**
     * Avoidance distance for risk area
     */
    double d_avoid_dist = 1.0;
    /**
     * Map minimal value of elevation in meters
     */
    double d_elevation_min;
    /**
     * Sample position in waypoint format
     */
    base::Waypoint w_sample_pos;
    /**
     * Fast Marching class for sample facing cost update
     */
    FastMarching fm_sample_facing;
    /**
     * Matrix containing elevation values
     */
    std::vector<std::vector<double>> vvd_elevation_map;
    /**
     * Matrix containing values regarding obstacles
     * 0 => obstacle
     * 1 => safe
     */
    std::vector<std::vector<int>> vvi_obstacle_map;
    /**
     * Matrix containing values regarding traversability
     * 0 => Area occupied by obstacles
     * 1 => Dilatation of obstacles, neither rover or sample can be placed here
     * 2 => Second dilatation of obstacles, the sampling area can remove part of this
     * 3 => Possible locations for the rover and the sampling area
     * 4 => Sampling Area 
     */
    std::vector<std::vector<int>> vvi_traversability_map;
    /**
     * Matrix containing cost values (cost for obstacles is INFINITY)
     */
    std::vector<std::vector<double>> vvd_cost_map;
    /**
     * Matrix containing minimum distance to obstacles
     */
    std::vector<std::vector<double>> vvd_proximity_map;

public:
    /**
     * Constructor that receives the map, process it and generates the cost and
     * obstacles maps
     */
    MobileManipMap(const RoverGuidance_Dem &rg_dem_m);
    /**
     * Constructor that introduces pre-computed elevation and cost maps
     */
    MobileManipMap(std::vector<std::vector<double>> &vvd_elevation_map_m,
                   std::vector<std::vector<double>> &vvd_cost_map_m,
                   double d_res_m);
    /**
     * Function to introduce the Sample into the costmap using FACE
     */
    void computeFACE(base::Waypoint w_sample_pos_m);
    /**
     * Function to get the current traversability map
     */
    void getTraversabilityMap(std::vector<std::vector<int>> &vvi_traversability_map_m);
    /**
     * Function to get the current cost map
     */
    void getCostMap(std::vector<std::vector<double>> &vvd_cost_map_m);
    /**
     * Function to get the current elevation map
     */
    void getElevationMap(std::vector<std::vector<double>> &vvd_elevation_map_m);
    /**
     * Function to get the current elevation map with its minimum starting at zero
     */
    void getElevationMapToZero(std::vector<std::vector<double>> &vvd_elevation_map_m);
    /**
     * Returns the map resolution --> d_res
     */
    double getResolution();
    /**
     * Returns the minimum value of elevation --> d_elevation_min
     */
    double getMinElevation();
    /**
     * Checks if the waypoint position is inside the map
     */
    bool isOutside(const base::Waypoint &w_sample_pos_m);
    /**
     * Checks if the waypoint position is within obstacle area
     */
    bool isObstacle(const base::Waypoint w_pos_m);

private:
    /**
     * RG DEM is checked and loaded into MobileManipMap
     */
    void loadDEM(const RoverGuidance_Dem &rg_dem_m);
    /**
     * RG DEM is checked and loaded into MobileManipMap
     */
    void loadSample(const base::Waypoint &w_sample_pos_m);
    /**
     * Memory is reserved for the matrices
     */
    void initializeMatrices();
    /**
     * The elevation map vvd_elevation_map is calculated from rg_dem
     */
    bool calculateElevationMap();
    /**
     * The traversability map vvi_traversability_map is calculated
     */
    bool calculateTraversabilityMap();
    /**
     * The proximity map vvd_proximity_map is calculated
     */
    bool calculateProximityToObstaclesMap();
    /**
     * Cost values are assigned to the cost map
     */
    void calculateCostValues();
    /**
     * Cost Map is modified
     */
    bool addSampleFacingObstacles();
};

#endif
