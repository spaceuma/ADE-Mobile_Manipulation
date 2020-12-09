#ifndef __MOBILE_MANIP_MAP__
#define __MOBILE_MANIP_MAP__

#include "FastMarching.h"
#include "RoverGuidance_InputDataStruct.h"
#include "Waypoint.hpp"
#include <math.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace FastMarching_lib;


/**
 * MMMap Component internal states
 */
enum MMMapState
{
    NO_DEM, // There is currently no usable DEM
    DEM_LOADED, // The DEM is loaded, but remains to be prepared for planning
    FACE_COMPUTED // DEM is ready for path planning
};

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
    RoverGuidance_Dem rg_nav_dem;
    
    /**
     * Map Data Status
     */
    MMMapState mapstate;
    
    /**
     * Debugging messages activation
     */ 
    bool b_debug_mode = false;
    
    /**
     * DEM structure data
     */
    unsigned int ui_num_cols; // Number of columns 
    unsigned int ui_num_rows; // Number of rows
    double d_res; // Resolution (in meters)  
    std::vector<double> vd_global_offset{0, 0, 0}; // Global offset (bottom left node, in meters)

    /**
     * DEM geometry data
     */
    std::vector<std::vector<double>> vvd_elevation_map; // Elevation (meters)
    std::vector<std::vector<double>> vvd_smoothed_elevation_map; // Processed Elevation (meters)
    std::vector<std::vector<double>> vvd_sd_map; // Spherical deviation (in degrees)
    std::vector<std::vector<double>> vvd_nx_map; // X-component of normal vector field
    std::vector<std::vector<double>> vvd_ny_map; // Y-component of normal vector field
    std::vector<std::vector<double>> vvd_nz_map; // Z-component of normal vector field 
    std::vector<std::vector<double>> vvd_slope_map; // Slope steepness (in degrees) 
    std::vector<std::vector<double>> vvd_local_slope_map; // Slope steepness (in degrees) 
    std::vector<std::vector<double>> vvd_aspect_map; // Slope Aspect direction (in radians)
    std::vector<std::vector<double>> vvd_local_aspect_map; // Slope Aspect direction (in radians)
    std::vector<std::vector<int8_t>> vvi_validity_map; // Valid(1)-Nonvalid(0) pixels 
    std::vector<std::vector<int8_t>> vvi_loc_validity_map; // Valid(1)-Nonvalid(0) pixels 
    std::vector<std::vector<int8_t>> vvi_nav_validity_map; // Valid(1)-Nonvalid(0) pixels 
    std::vector<std::vector<double>> vvd_loc_elevation_map; // Elevation (meters)
    double d_elevation_min; // Existing minimal value of elevation (in meters)
    
    /**
     * THRESHOLD VALUES
     */
    double d_slope_threshold = 20.0; // Average slope
    double d_sd_threshold = 16.82; // Spherical deviation
    double d_valid_ratio_threshold = .5; //Valid pixels ratio
    double d_contour_ratio_threshold = .3; //contour/valid pixels ratio
    
    /**
     * CONFIG VALUES 
     */
    int i_validity_morph_iterations = 2; //Iterations for the morphological CLOSE operation on validity map
    double d_avoid_dist = 1.0; //Avoidance distance for risk area
    double d_occupancy_dist = 1.6; // Occupancy radius
    double d_minreach_dist = 1.0;//1.344; // Min reachability distance
    double d_maxreach_dist = 1.3;//1.584; // Max reachability distance

    /**
     * DEM Navigation Data 
     */
    std::vector<std::vector<int>> vvi_obstacle_map; // Obstacle(0)-Safe(1)
    std::vector<std::vector<int>> vvi_loc_obstacle_map; // Obstacle(0)-Safe(1)
    std::vector<std::vector<int>> vvi_nav_obstacle_map; // Obstacle(0)-Safe(1)
    std::vector<std::vector<double>> vvd_proximity_map; // Distance to closest obstacle (meters)
    std::vector<std::vector<double>> vvd_loc_proximity_map; // Distance to closest obstacle in LocCam (meters)
    std::vector<std::vector<int>> vvi_traversability_map; // Obstacle(0)-FirstDilatation(1)-SecondDilatation(2)-RoverTraversableArea(3)-SamplingArea(4)
    std::vector<std::vector<double>> vvd_cost_map; // Cost Map (obstacles are INFINITY)

    /**
     * FACE (Frontal Approach Cost Edition)
     */
    FastMarching fm_sample_facing; // Fast Marching class
    double d_outter_sampling_dist; // Sampling Ring Outter Radius
    double d_inner_sampling_dist; // Sampling Ring Inner Radius
    base::Waypoint w_sample_pos; // Sample position


    /**
     * Internal Functions
     */
    void checkValidityMap(double &d_valid_ratio, double &d_contour_ratio, bool b_isLoc);
    bool loadGlobalSample(const base::Waypoint &w_sample_pos_m);
    bool calculateElevationMap(bool b_isLoc);
    bool calculateTraversabilityMap();
    bool calculateProximityToObstaclesMap();
    void calculateCostValues();
    bool addSampleFacingObstacles();

public:

    /**
     * Constructors
     */
    MobileManipMap(bool b_debug_mode_m = false);
    MobileManipMap(const RoverGuidance_Dem &rg_dem_m,
                   unsigned int &ui_isDEM_loaded,
                   bool b_debug_mode_m = false);
    MobileManipMap(std::vector<std::vector<double>> &vvd_elevation_map_m,
                   std::vector<std::vector<double>> &vvd_cost_map_m,
                   double d_res_m,
                   base::Waypoint w_sample_pos_m,
                   double d_avoid_dist_m,
                   double d_maxreach_dist_m);
    /**
     * RG DEMs Introduction
     */
    unsigned int loadDEM(const RoverGuidance_Dem &rg_dem_m, bool b_update = false);
    unsigned int loadLocDEM(const RoverGuidance_Dem &rg_dem_m);

    /**
     * Function to introduce the Sample into the costmap using FACE
     */
    unsigned int computeFACE(base::Waypoint w_sample_pos_m);
    
    /**
     * Output Variables Functions
     */
    base::Waypoint getSample(); // Sample Pos in local coordinates
    void getTraversabilityMap(
        std::vector<std::vector<int>> &vvi_traversability_map_m);
    void getValidityMap(std::vector<std::vector<int8_t>> &vvi_validity_map_m);
    std::vector<std::vector<double>> *getCostMap();
    void getCostMap(std::vector<std::vector<double>> &vvd_cost_map_m);
    void getSlopeMap(std::vector<std::vector<double>> &vvd_slope_map_m);
    void getSDMap(std::vector<std::vector<double>> &vvd_sd_map_m);
    bool getElevationMap(std::vector<std::vector<double>> &vvd_elevation_map_m);
    bool getElevationMapToZero(
        std::vector<std::vector<double>> &vvd_elevation_map_m);
    double getResolution();
    double getMinElevation();
    std::vector<double> getOffset();
    std::vector<double> *getPointer2Offset();
    void printDEMinfo();

    /**
     * Check Functions.
     */
    bool isOutside(const base::Waypoint &w_sample_pos_m); // Waypoint inside the map
    bool isObstacle(const base::Waypoint w_pos_m); // Waypoint within obstacle area
    bool checkObstacles(std::vector<base::Waypoint> &vw_rover_path_m); // Path contacting obstacle area
    bool isSampleLoaded(); // Is sample loaded and FACE computed


    /**
     * Set configurable values
     */
    void setThresholdValues(double d_temptative_slope_threshold, 
                            double d_temptative_sd_threshold, 
                            double d_temptative_valid_ratio_threshold,
                            double d_temptative_contour_ratio_threshold);
    void setConfigValues(int i_close_iter,
		         double d_avoid_dist, 
                         double d_occ_radius, 
                         double d_min_reach,
                         double d_max_reach);

};

#endif
