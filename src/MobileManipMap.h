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
     * Map minimal value of elevation in meters 
     */
    double d_elevation_min;
    /**
     * Fast Marching class for sample facing cost update 
     */
    FastMarching fm_sample_facing;
    /**
     * Matrix containing elevation values 
     */
    std::vector<std::vector<double>> vvd_elevation_map;
    /**
     * Matrix containing values regarding traversability
     * 0 => obstacle
     * 1 => traversable (far from sample)
     * 2 => traversable (near sample)  
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
     * Simple form of constructor
     */
    MobileManipMap();
    /**
     * Constructor that receives the map, process it and generates the cost and
     * obstacles maps
     */
    MobileManipMap(const RoverGuidance_Dem &rg_dem_m);
    /**
     * Straightforward way to introduce the input DEM
     */
    int setRGDem(const RoverGuidance_Dem &rg_dem_m);
    /**
     * Setting a pre-computed cost map
     */
    void setCostMap(std::vector<std::vector<double>> &vvd_cost_map_m);
    /**
     * Setting a pre-computed elevation map
     */
    void setElevationMap(std::vector<std::vector<double>> &vvd_elevation_map_m,
                         double d_res_m);
    /**
     * Function to get the current cost map
     */
    void getCostMap(std::vector<std::vector<double>> &vvd_cost_map_m);
    /**
     * Function to get the current elevation map
     */
    void getElevationMap(std::vector<std::vector<double>> &vvd_elevation_map_m);
    /**
     * Returns the map resolution --> d_res
     */
    double getResolution();
    /**
     * Returns the minimum value of elevation --> d_elevation_min
     */
    double getMinElevation();
    /**
     * Updates the traversability and cost maps based on the sample position
     */
    bool addSampleFacingObstacles(base::Waypoint w_sample_pos_m);

private:
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
     * The cost map vvd_cost_map is calculated based on proximity and
     * traversability maps
     */
    bool calculateCostMap();
    /**
     * Cost values are assigned to the cost map
     */
    void calculateCostValues();
    };

#endif
