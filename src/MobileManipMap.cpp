#include "MobileManipMap.h"
#include <iostream>

using namespace std;
using namespace cv;


/*
 * CONSTRUCTOR
 *
 */

MobileManipMap::MobileManipMap(bool b_debug_mode_m)
{

    // Set internal state to NO_DEM
    this->mapstate = NO_DEM;

    // To show DEBUG messages
    this->b_debug_mode = b_debug_mode_m;

}


/*
 * CONSTRUCTOR + loadDEM()
 *
 */

MobileManipMap::MobileManipMap(const RoverGuidance_Dem &dem,
                               unsigned int &ui_isDEM_loaded,
                               bool b_debug_mode_m)
{

    // Set internal state to NO_DEM
    this->mapstate = NO_DEM;

    // Load the input DEM
    ui_isDEM_loaded = this->loadDEM(dem);
    if (ui_isDEM_loaded == 0)
    {
        mapstate = DEM_LOADED;
    }

    // To show DEBUG messages
    this->b_debug_mode = b_debug_mode_m;

}


/*
 * loadDEM()
 *  - Function to load and process input DEM
 */

unsigned int MobileManipMap::loadDEM(const RoverGuidance_Dem &dem,
	                             bool b_update)
{

    /*
     * 1 - Inizialization 
     */

    // Check if input DEM updates the previous one or not
    if (b_update) 
    {

        if (this->mapstate == NO_DEM)
	{

	    // There is no previous DEM to update
	    return 8; 	

	}
	
	if (dem.nodeSize_m != this->d_res)
	{

            // Resolution between DEMs is different 
	    std::cout << "[MM] \033[35m[---ERROR--]"
                      " [MobileManipMap::loadDEM()]\033[0m Value of resolution "
		      << dem.nodeSize_m << " meters is not equal to original"
		      " DEM resolution " << this->d_res << " meters"  
		      << std::endl;
            return 1;

	}

        // Input DEM is saved
	this->rg_dem = dem;

    }
    else
    {

        std::cout << "[MM] \033[35m[----------]"
		" [MobileManipMap::loadDEM()]\033[0m New incoming Nav DEM " 
		<< std::endl;

        // Internal status is resetted to NO_DEM
	this->mapstate = NO_DEM;

        // Check Resolution
	if (dem.nodeSize_m <= 0)
        {

            std::cout << "[MM] \033[35m[---ERROR--]"
		    " [MobileManipMap::loadDEM()]\033[0m Value of resolution " 
		      << dem.nodeSize_m << " is invalid (equal or less than "
		      "zero)" << std::endl;
            return 1;

        }

	// Check number of rows
        if (dem.rows < 5)
        {

            std::cout << "[MM] \033[35m[---ERROR--]"
		    " [MobileManipMap::loadDEM()]\033[0m Number of rows " 
		      << dem.rows << " is invalid (less than five)" 
		      << std::endl;
            return 2;

	}

	if (dem.cols < 5) // Columns
        {

            std::cout << "[MM] \033[35m[---ERROR--]"
		    " [MobileManipMap::loadDEM()]\033[0m Value of columns " 
		      << dem.cols << " is invalid (less than five)" 
		      << std::endl;
            return 3;

	}
    
        // DEM data is saved
	try
        {

            for (unsigned int i = 0; i < 3; i++)
            {
                this->vd_global_offset[i] = dem.mapOrigin_m_Mlg[i];
            }

	}
        catch (exception &e)
        {
            
            // Offset in wrong format
            return 4;

        }
        this->d_res = dem.nodeSize_m;
        this->ui_num_rows = dem.rows;
        this->ui_num_cols = dem.cols;
        this->rg_nav_dem = dem;
        this->rg_dem = dem;

    }
    

    /*
     *  2 - DEM processing
     */

    try
    {

	// If it is nav DEM
        if (!b_update)
	{	

            // Matrix allocation in memory
            std::cout << "[MM] \033[35m[----------]"
		    " [MobileManipMap::loadDEM()]\033[0m Allocating new maps"
		    << std::endl;
            // Common - Used for computation only
            this->vvd_elevation_map.clear();
            this->vvd_smoothed_elevation_map.clear();
            this->vvi_validity_map.clear();
            this->vvd_slope_map.clear();
            this->vvd_local_slope_map.clear();
            this->vvd_aspect_map.clear();
            this->vvd_local_aspect_map.clear();
            this->vvd_sd_map.clear();
            this->vvd_nx_map.clear();
            this->vvd_ny_map.clear();
            this->vvd_nz_map.clear();
            this->vvi_face_obstacle_map.clear();
            this->vvi_loc_obstacle_map.clear();
            this->vvi_nav_obstacle_map.clear();
            this->vvi_traversability_map.clear();
            this->vvd_cost_map.clear();
            this->vvd_face_proximity_map.clear();
	
	    // Nav
	    //this->vvd_nav_elevation_map.clear();	
            //this->vvi_nav_validity_map.clear();
            //this->vvi_nav_obstacle_map.clear();
            this->vvd_nav_proximity_map.clear();

            // Loc
            this->vvd_loc_elevation_map.clear();	
            this->vvi_loc_validity_map.clear();
            this->vvd_loc_proximity_map.clear();

	    // Matrices initialization
            std::cout << "[MM] \033[35m[----------]"
		    " [MobileManipMap::loadDEM()]\033[0m Initializing new maps"
		    << std::endl;
            std::vector<double> vd_row(this->ui_num_cols, INFINITY);
            std::vector<int> vi_row(this->ui_num_cols, 0);
            std::vector<int8_t> vit_row(this->ui_num_cols);
            
	    // Assignation of dummy values
	    for (uint j = 0; j < this->ui_num_rows; j++)
            {

                this->vvd_elevation_map.push_back(vd_row);
                this->vvd_loc_elevation_map.push_back(vd_row);
                this->vvd_smoothed_elevation_map.push_back(vd_row);
                this->vvi_validity_map.push_back(vit_row);
                this->vvi_loc_validity_map.push_back(vit_row);
                this->vvi_nav_validity_map.push_back(vit_row);
                this->vvi_loc_validity_map.push_back(vit_row);
                this->vvd_slope_map.push_back(vd_row);
                this->vvd_local_slope_map.push_back(vd_row);
                this->vvd_aspect_map.push_back(vd_row);
                this->vvd_local_aspect_map.push_back(vd_row);
                this->vvd_sd_map.push_back(vd_row);
                this->vvd_nx_map.push_back(vd_row);
                this->vvd_ny_map.push_back(vd_row);
                this->vvd_nz_map.push_back(vd_row);
                this->vvi_face_obstacle_map.push_back(vi_row);
                this->vvi_loc_obstacle_map.push_back(vi_row);
                this->vvi_nav_obstacle_map.push_back(vi_row);
                this->vvi_traversability_map.push_back(vi_row);
                this->vvd_cost_map.push_back(vd_row);
                this->vvd_face_proximity_map.push_back(vd_row);
                this->vvd_loc_proximity_map.push_back(vd_row);
                this->vvd_nav_proximity_map.push_back(vd_row);

            }
 
        } // Finished initializing the matrices

        // Save and process the validity data
	double d_valid_ratio, d_contour_ratio;
        std::cout << "[MM] \033[35m[----------]"
		" [MobileManipMap::loadDEM()]\033[0m Importing Validity Map"
	       	<< std::endl;
        this->processValidityMap(d_valid_ratio, d_contour_ratio, b_update);
	
	// Check the cuality of the validity parameters
	if (!b_update)
	{

            // Not enough valid pixels?
            if (d_valid_ratio < this->d_valid_ratio_threshold)
            {

                std::cout << "[MM] \033[31m[--ERROR---]"
		      " [MobileManipMap::loadDEM()]\033[0m The valid_ratio is "
		      << d_valid_ratio << ", threshold is " 
		      << d_valid_ratio_threshold << std::endl;
                return 6;

	    }

            // Too many contour pixels?
            if (d_contour_ratio > this->d_contour_ratio_threshold)
            {

                std::cout << "[MM] \033[31m[--ERROR---]"
                   " [MobileManipMap::loadDEM()]\033[0m The contour_ratio is " 
		   << d_contour_ratio << ", threshold is " 
		   << d_contour_ratio_threshold << std::endl;
                return 7;

	    }

	}

        // Process the input elevation data to generate obstacles and cost
        std::cout << "[MM] \033[35m[----------]"
		" [MobileManipMap::loadDEM()]\033[0m Importing Elevation Map"
		<< std::endl;
        this->processElevationMap(b_update);

        // Calculates the proximity to obstacles
        std::cout << "[MM] \033[35m[----------]"
		" [MobileManipMap::loadDEM()]\033[0m Calculating Proximity to"
		" obstacles" << std::endl;
	this->calculateObstacleProximityMap(b_update);

	// Update the internal status
        mapstate = DEM_LOADED;
        std::cout << "[MM] \033[1;35m[----------]"
		" [MobileManipMap::loadDEM()]\033[0m DEM successfully imported"
	       	<< std::endl;

    }
    catch (bad_alloc &ba)
    {

        // The data could not be allocated
        mapstate = NO_DEM;
        return 5;

    }

    // Everything went fine
    return 0;
}



bool MobileManipMap::checkObstacles(std::vector<base::Waypoint> &vw_rover_path_m) // Introduce here the path
{
    std::vector<int> vi_pos(2, 0);
    for (uint i = 0; i < vw_rover_path_m.size() - 1; i++) // Last waypoint is the sample
    {
        vi_pos[0] = (int)(vw_rover_path_m[i].position[0] / this->d_res + 0.5);
        vi_pos[1] = (int)(vw_rover_path_m[i].position[1] / this->d_res + 0.5);
    // TODO - take care of exceptions regarding unvalid indexes
    // TODO - There may be cases where previous path is just a very small distance under d_occupancy_dist... due to checking with a node... (maybe interpolate?)
        if (this->vvd_loc_proximity_map[vi_pos[1]][vi_pos[0]] <= this->d_dilation*this->d_occupancy_dist)
        {
            std::cout << "Detected obstacle near waypoint " << i << ", which is ( " << vw_rover_path_m[i].position[0] << ", " << vw_rover_path_m[i].position[1] << " ), where proximity is " << this->vvd_loc_proximity_map[vi_pos[1]][vi_pos[0]] << std::endl;
            return true;
        }
    }
    return false;
}



// Valid ratio is the ratio between the number of valid pixels and the total
// Contour ratio is the ratio between the number of corner pixels and valid
// pixels
void MobileManipMap::processValidityMap(double &d_valid_ratio,
                                        double &d_contour_ratio,
				        bool b_isLoc)
{
    Mat mat_valid_map
        = Mat::zeros(cv::Size(ui_num_cols, ui_num_rows), CV_32FC1);

    unsigned int ui_point_counter = 0, ui_valid_pixels = 0,
                 ui_total_pixels = ui_num_cols * ui_num_rows;
    for (int j = 0; j < mat_valid_map.rows; j++)
    {
        for (int i = 0; i < mat_valid_map.cols; i++)
        {
            this->vvi_validity_map[j][i]
                = this->rg_dem.p_pointValidityFlag[i + j * this->ui_num_cols];
	    if (b_isLoc)
	    {
                this->vvi_loc_validity_map[j][i] = this->vvi_validity_map[j][i];
	    }
	    else
	    {
                this->vvi_nav_validity_map[j][i] = this->vvi_validity_map[j][i];
	    }
            mat_valid_map.at<float>(j, i) = (float)this->vvi_validity_map[j][i];
            if (this->vvi_validity_map[j][i] == 1)
            {
                ui_valid_pixels++;
            }
        }
    }

    mat_valid_map.convertTo(mat_valid_map, CV_8UC1);
    std::vector<std::vector<Point>> vvp_contours;

    findContours(
        mat_valid_map, vvp_contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

    for (int k = 0; k < vvp_contours.size(); k++)
    {
        ui_point_counter += vvp_contours[k].size();
    }

    d_valid_ratio = (double)ui_valid_pixels / (double)ui_total_pixels;
    d_contour_ratio = (double)ui_point_counter / (double)ui_valid_pixels;
}


bool MobileManipMap::processElevationMap(bool b_isLoc)
{
    std::vector<double> row;
    d_elevation_min = INFINITY;

    for (int j = 0; j < this->ui_num_rows; j++)
    {
        for (int i = 0; i < this->ui_num_cols; i++)
        {
            if (this->vvi_validity_map[j][i] == 1)
	    {
	        this->vvd_elevation_map[j][i]
                    = this->rg_dem.p_heightData_m[i + j * this->ui_num_cols];
                if (this->vvd_elevation_map[j][i] < this->d_elevation_min)
                {
                    this->d_elevation_min = this->vvd_elevation_map[j][i];
                }
	    }
	    else
	    {
	        this->vvd_elevation_map[j][i] = INFINITY;
	    }
        }
    }

    if (this->b_debug_mode)
    {
        std::cout << "Elevation Map is saved as vector<vector<double>>"
                  << std::endl;
    }

    int i_occupancy_kernel = (int)(0.6 / this->d_res);
    int i_nodes;
    double d_elevation;
    for (int j = 0; j < this->ui_num_rows; j++)
    {
        for (int i = 0; i < this->ui_num_cols; i++)
        {
            if (this->vvi_validity_map[j][i] == 1)
            {
                i_nodes = 0;
                d_elevation = 0;
                for (int l = -i_occupancy_kernel; l <= i_occupancy_kernel; l++)
                {
                    for (int k = -i_occupancy_kernel; k <= i_occupancy_kernel;
                         k++)
                    {
                        if ((j + l >= 0) && (j + l <= this->ui_num_rows - 1)
                            && (i + k >= 0) && (i + k <= this->ui_num_cols - 1)
                            && (sqrt(pow((double)l, 2) + pow((double)k, 2))
                                < (double)i_occupancy_kernel))
                        {
                            if (this->vvi_validity_map[j + l][i + k] == 1)
                            {
                                i_nodes++;
                                d_elevation
                                    += this->vvd_elevation_map[j + l][i + k];
                            }
                        }
                    }
                }
                this->vvd_smoothed_elevation_map[j][i]
                    = d_elevation / (double)i_nodes;
            }
            else
            {
                this->vvd_smoothed_elevation_map[j][i]
                    = this->vvd_elevation_map[j][i];
            }
        }
    }

    if (this->b_debug_mode)
    {
        std::cout << "Smoothed elevation map is computed" << std::endl;
    }

    double dx, dy;
    for (int j = 1; j < this->ui_num_rows - 1; j++)
    {
        for (int i = 1; i < this->ui_num_cols - 1; i++)
        {
            if (this->vvi_validity_map[j][i] == 1)
            {
                if (this->vvi_validity_map[j][i + 1] == 0)
                {
                    if (this->vvi_validity_map[j][i - 1] == 0)
                    {
                        dx = 0;
                    }
                    else
                    {
                        dx = (this->vvd_elevation_map[j][i]
                              - this->vvd_elevation_map[j][i - 1])
                             / (this->d_res);
                    }
                }
                else if (this->vvi_validity_map[j][i - 1] == 0)
                {
                    dx = (this->vvd_elevation_map[j][i + 1]
                          - this->vvd_elevation_map[j][i])
                         / (this->d_res);
                }
                else
                {
                    dx = (this->vvd_elevation_map[j][i + 1]
                          - this->vvd_elevation_map[j][i - 1])
                         / (2 * this->d_res);
                }
                if (this->vvi_validity_map[j + 1][i] == 0)
                {
                    if (this->vvi_validity_map[j - 1][i] == 0)
                    {
                        dy = 0;
                    }
                    else
                    {
                        dy = (this->vvd_elevation_map[j][i]
                              - this->vvd_elevation_map[j - 1][i])
                             / (this->d_res);
                    }
                }
                else if (this->vvi_validity_map[j - 1][i] == 0)
                {
                    dy = (this->vvd_elevation_map[j + 1][i]
                          - this->vvd_elevation_map[j][i])
                         / (this->d_res);
                }
                else
                {
                    dy = (this->vvd_elevation_map[j + 1][i]
                          - this->vvd_elevation_map[j - 1][i])
                         / (2 * this->d_res);
                }

                this->vvd_local_slope_map[j][i] = atan(sqrt(pow(dx, 2) + pow(dy, 2)));
                this->vvd_local_aspect_map[j][i] = atan2(dy, dx);
                this->vvd_nx_map[j][i] = sin(this->vvd_local_aspect_map[j][i])
                                         * sin(this->vvd_local_slope_map[j][i]);
                this->vvd_ny_map[j][i] = cos(this->vvd_local_aspect_map[j][i])
                                         * sin(this->vvd_local_slope_map[j][i]);
                this->vvd_nz_map[j][i] = cos(this->vvd_local_slope_map[j][i]);
	    }
            else
            {
                this->vvd_local_slope_map[j][i] = 0.0;
                this->vvd_local_aspect_map[j][i] = 0.0;
                this->vvd_nx_map[j][i] = 0.0;
                this->vvd_ny_map[j][i] = 0.0;
                this->vvd_nz_map[j][i] = 0.0;
            }
        }
    }

    for (int j = 1; j < this->ui_num_rows - 1; j++)
    {
        for (int i = 1; i < this->ui_num_cols - 1; i++)
        {
            if (this->vvi_validity_map[j][i] == 1)
            {
                if (this->vvi_validity_map[j][i + 1] == 0)
                {
                    if (this->vvi_validity_map[j][i - 1] == 0)
                    {
                        dx = 0;
                    }
                    else
                    {
                        dx = (this->vvd_smoothed_elevation_map[j][i]
                              - this->vvd_smoothed_elevation_map[j][i - 1])
                             / (this->d_res);
                    }
                }
                else if (this->vvi_validity_map[j][i - 1] == 0)
                {
                    dx = (this->vvd_smoothed_elevation_map[j][i + 1]
                          - this->vvd_smoothed_elevation_map[j][i])
                         / (this->d_res);
                }
                else
                {
                    dx = (this->vvd_smoothed_elevation_map[j][i + 1]
                          - this->vvd_smoothed_elevation_map[j][i - 1])
                         / (2 * this->d_res);
                }
                if (this->vvi_validity_map[j + 1][i] == 0)
                {
                    if (this->vvi_validity_map[j - 1][i] == 0)
                    {
                        dy = 0;
                    }
                    else
                    {
                        dy = (this->vvd_smoothed_elevation_map[j][i]
                              - this->vvd_smoothed_elevation_map[j - 1][i])
                             / (this->d_res);
                    }
                }
                else if (this->vvi_validity_map[j - 1][i] == 0)
                {
                    dy = (this->vvd_smoothed_elevation_map[j + 1][i]
                          - this->vvd_smoothed_elevation_map[j][i])
                         / (this->d_res);
                }
                else
                {
                    dy = (this->vvd_smoothed_elevation_map[j + 1][i]
                          - this->vvd_smoothed_elevation_map[j - 1][i])
                         / (2 * this->d_res);
                }

                this->vvd_slope_map[j][i]
                    = atan(sqrt(pow(dx, 2) + pow(dy, 2))) * 180.0 / 3.1416;
                this->vvd_aspect_map[j][i] = atan2(dy, dx);
                if (this->vvd_slope_map[j][i] >= this->d_slope_threshold)
	        {
                    if(b_isLoc)
		    {
                        this->vvi_loc_obstacle_map[j][i] = 1;
		    }
		    else
		    {
                        this->vvi_nav_obstacle_map[j][i] = 1;
		    } 
	        } 
            }
            else
            {
                this->vvd_slope_map[j][i] = 0.0;
                this->vvd_aspect_map[j][i] = 0.0;
            }
        }
    }

    double d_sumnx, d_sumny, d_sumnz, d_R, d_Rratio;
    for (int j = 1; j < this->ui_num_rows - 1; j++)
    {
        for (int i = 1; i < this->ui_num_cols - 1; i++)
        {
            if (this->vvi_validity_map[j][i] == 1)
            {
                i_nodes = 0;
                d_sumnx = 0;
                d_sumny = 0;
                d_sumnz = 0;
                for (int l = -i_occupancy_kernel; l <= i_occupancy_kernel; l++)
                {
                    for (int k = -i_occupancy_kernel; k <= i_occupancy_kernel;
                         k++)
                    {
                        if ((j + l > 0) && (j + l < this->ui_num_rows - 1)
                            && (i + k > 0) && (i + k < this->ui_num_cols - 1)
                            && (sqrt(pow((double)l, 2) + pow((double)k, 2))
                                < (double)i_occupancy_kernel))
                        {
                            if (this->vvi_validity_map[j + l][i + k] == 1)
                            {
                                i_nodes++;
                                d_sumnx += this->vvd_nx_map[j + l][i + k];
                                d_sumny += this->vvd_ny_map[j + l][i + k];
                                d_sumnz += this->vvd_nz_map[j + l][i + k];
                            }
                        }
                    }
                }
                d_R = sqrt(pow(d_sumnx, 2) + pow(d_sumny, 2) + pow(d_sumnz, 2));
                // A numerical error is avoided when d_R == i_nodes -> d_Rratio
                // ~ 1.0 +- error
                d_Rratio = min(d_R / (double)i_nodes, 0.9999999);
                this->vvd_sd_map[j][i]
                    = sqrt(-2 * log(d_Rratio)) * 180.0 / 3.1416;
                if (isnan(this->vvd_sd_map[j][i]))
                {
                    std::cout << "At node " << i << "," << j << " is zero "
                              << std::endl;
                    std::cout << "  i_nodes =  " << i_nodes << std::endl;
                    std::cout << "  d_R =  " << d_R << std::endl;
                    std::cout << "  d_sumnx =  " << d_sumnx << std::endl;
                    std::cout << "  d_sumny =  " << d_sumny << std::endl;
                    std::cout << "  d_sumnz =  " << d_sumnz << std::endl;
                    std::cout << "  ratio = " << d_R / (double)i_nodes
                              << std::endl;
                    std::cout << "  log = " << log(d_R / (double)i_nodes)
                              << std::endl;
                }
                if (this->vvd_sd_map[j][i] >= this->d_sd_threshold)
	        {
                    if(b_isLoc)
		    {
                        this->vvi_loc_obstacle_map[j][i] = 1;
		    }
		    else
		    {
                        this->vvi_nav_obstacle_map[j][i] = 1;
		    } 
	        } 
            }
            else
            {
                this->vvd_sd_map[j][i] = 0.0;
            }
        }
    }

    if (this->b_debug_mode)
    {
        std::cout
            << "Slope, Aspect and Normal Vector Components Maps are computed"
            << std::endl;
        std::cout << "i_occupancy_kernel = " << i_occupancy_kernel << std::endl;
    }

    return true;
}


/*
 * Calculate the proximity to obstacles
 */

void MobileManipMap::calculateObstacleProximityMap(bool b_isUpdate)
{
     // Here compute a proximity map
    Mat mat_proximity_map;
    Mat mat_obstacle_map
        = Mat::zeros(cv::Size(ui_num_cols, ui_num_rows), CV_32FC1);
    Mat mat_validity_obstacle_map
        = Mat::zeros(cv::Size(ui_num_cols, ui_num_rows), CV_32FC1);
    Mat mat_validity_map
        = Mat::zeros(cv::Size(ui_num_cols, ui_num_rows), CV_32FC1);

    if (b_isUpdate) // TODO: Optimize this by only executing when an obstacle is found
    {
        for (int j = 0; j < this->ui_num_rows; j++)
        {
            for (int i = 0; i < this->ui_num_cols; i++)
            {
                if ((this->vvi_loc_obstacle_map[j][i] == 1)&&(this->vvi_loc_validity_map[j][i] == 1))
                {
                    mat_obstacle_map.at<float>(j, i) = 0;
                }
                else
                {
                    mat_obstacle_map.at<float>(j, i) = 1;
                }
            }
        }        
        mat_obstacle_map.convertTo(mat_obstacle_map, CV_8UC1);
    }
    else
    {
        for (int j = 0; j < this->ui_num_rows; j++)
        {
            for (int i = 0; i < this->ui_num_cols; i++)
            {
                if (this->vvi_nav_obstacle_map[j][i] == 1)
                {
                    mat_obstacle_map.at<float>(j, i) = 0;
                }
                else
                {
                    mat_obstacle_map.at<float>(j, i) = 1;
                }
                mat_validity_map.at<float>(j, i)
                    = (float)this->vvi_nav_validity_map[j][i];
            }
        }       
        threshold(
            mat_validity_map, mat_validity_obstacle_map, 0.5, 255, THRESH_BINARY);
   
        mat_validity_obstacle_map.convertTo(mat_validity_obstacle_map, CV_8UC1);
        mat_obstacle_map.convertTo(mat_obstacle_map, CV_8UC1);
    
        morphologyEx(mat_validity_obstacle_map,
                     mat_validity_obstacle_map,
                     MORPH_CLOSE,
                     getStructuringElement(MORPH_ELLIPSE, Size(3, 3)),
                     Point(-1, -1),
                     this->i_validity_morph_iterations);    

        bitwise_and(mat_obstacle_map, mat_validity_obstacle_map, mat_obstacle_map);

    }
    distanceTransform(mat_obstacle_map, mat_proximity_map, DIST_L2, 5);
    mat_proximity_map = mat_proximity_map * this->d_res;

    if (b_isUpdate)
    {

        for (int j = 0; j < this->ui_num_rows; j++)
        {

            for (int i = 0; i < this->ui_num_cols; i++)
            {

                this->vvd_loc_proximity_map[j][i] = mat_proximity_map.at<float>(j, i);
            
	    }

	}

    }
    else
    {

        for (int j = 0; j < this->ui_num_rows; j++)
        {

            for (int i = 0; i < this->ui_num_cols; i++)
            {
            
                this->vvd_nav_proximity_map[j][i] = mat_proximity_map.at<float>(j, i);
            
	    }

	}

    }

}





MobileManipMap::MobileManipMap(
    std::vector<std::vector<double>> &vvd_elevation_map_m,
    std::vector<std::vector<double>> &vvd_cost_map_m,
    double d_res_m,
    base::Waypoint w_sample_pos_m,
    double d_avoid_dist_m,
    double d_maxreach_dist_m)
{
    this->d_res = d_res_m;
    this->d_avoid_dist = d_avoid_dist_m;
    this->d_maxreach_dist = d_maxreach_dist_m;
    this->vvd_elevation_map.clear();
    std::vector<double> row;
    this->d_elevation_min = INFINITY;
    this->d_inner_sampling_dist = this->d_avoid_dist + this->d_maxreach_dist;
    this->d_outter_sampling_dist
        = this->d_inner_sampling_dist
          + 1.72 * this->d_res; // TODO - Maybe this should be 1.42? = sqrt(2)
    for (uint j = 0; j < vvd_elevation_map_m.size(); j++)
    {
        for (uint i = 0; i < vvd_elevation_map_m[0].size(); i++)
        {
            row.push_back(vvd_elevation_map_m[j][i]);
            if (vvd_elevation_map_m[j][i] < this->d_elevation_min)
            {
                this->d_elevation_min = vvd_elevation_map_m[j][i];
            }
        }
        vvd_elevation_map.push_back(row);
        row.clear();
    }
    this->vvd_cost_map.clear();
    for (uint j = 0; j < vvd_cost_map_m.size(); j++)
    {
        for (uint i = 0; i < vvd_cost_map_m[0].size(); i++)
        {
            if (vvd_cost_map_m[j][i] > 0)
            {
                row.push_back(vvd_cost_map_m[j][i]);
            }
            else
            {
                row.push_back(INFINITY);
            }
        }
        this->vvd_cost_map.push_back(row);
        row.clear();
    }
    this->ui_num_cols = vvd_elevation_map_m[0].size();
    this->ui_num_rows = vvd_elevation_map_m.size();
    this->loadGlobalSample(w_sample_pos_m);
    this->mapstate = FACE_COMPUTED;
}

unsigned int MobileManipMap::computeFACE(base::Waypoint w_sample_pos_m, base::Waypoint w_rover_pos_m)
{
    if (mapstate == NO_DEM)
    {
        return 1; // Error: there is no existing DEM
    }
    else
    {
        mapstate = DEM_LOADED;
    }
    if (!this->loadGlobalSample(w_sample_pos_m))
    {
        return 2; // Error: sample out of the DEM
    }

    try
    {
        // Compute vvb_face_obstacle_map
        if(!this->calculateTraversabilityMap(w_rover_pos_m))
	{
            return 4; // Goal is too close to the rover
	}
        if (this->b_debug_mode)
        {
            std::cout << "Computed Traversability Map" << std::endl;
        }
        // Compute vvd_face_proximity_map and vvd_cost_map
        this->calculateCostMap(w_rover_pos_m);
        if (this->b_debug_mode)
        {
            std::cout << "Computed FACE" << std::endl;
        }
        // this->addValidityCost();
        if (isObstacle(this->w_sample_pos))
        {
            return 3;
        }
    }
    catch (exception &e)
    {
        cout << " An exception occured while calculating the cost map" << endl;
        throw e;
    }
    mapstate = FACE_COMPUTED;
    return 0;
}


bool MobileManipMap::loadGlobalSample(const base::Waypoint &w_sample_pos_m)
{
    base::Waypoint w_sample_localpos;
    w_sample_localpos.position[0]
        = w_sample_pos_m.position[0] - this->vd_global_offset[0];
    w_sample_localpos.position[1]
        = w_sample_pos_m.position[1] - this->vd_global_offset[1];
    w_sample_localpos.position[2] = w_sample_pos_m.position[2];
    if (isOutside(w_sample_localpos))
    {
        return false;
    }
    else
    {
        this->w_sample_pos = w_sample_localpos;
        return true;
    }
}

bool MobileManipMap::isOutside(const base::Waypoint &w_sample_localpos_m)
{
    if ((w_sample_localpos_m.position[0] < this->d_res)
        || (w_sample_localpos_m.position[1] < this->d_res)
        || (w_sample_localpos_m.position[0]
            > ((double)this->ui_num_cols - 2) * this->d_res)
        || (w_sample_localpos_m.position[1]
            > ((double)this->ui_num_rows - 2) * this->d_res))
    {
                return true;
    }
    else
    {
        return false;
    }
}

void MobileManipMap::printDEMinfo()
{
    std::cout << "res: "
                  << this->d_res << "; cols: " << this->ui_num_cols
                  << "; rows: " << this->ui_num_rows << "; offset: (" << this->vd_global_offset[0] << ", " << this->vd_global_offset[1] << ")";
}

bool MobileManipMap::isObstacle(const base::Waypoint w_localpos_m)
{
    std::vector<int> vi_pos(2, 0);
    vi_pos[0] = (int)(w_localpos_m.position[0] / this->d_res + 0.5);
    vi_pos[1] = (int)(w_localpos_m.position[1] / this->d_res + 0.5);
    // TODO - take care of exceptions regarding unvalid indexes
    if (this->vvd_cost_map[vi_pos[1]][vi_pos[0]] == INFINITY)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool MobileManipMap::isSampleLoaded()
{
    return this->mapstate == FACE_COMPUTED;
}

base::Waypoint MobileManipMap::getSample()
{
    return this->w_sample_pos;
}

void MobileManipMap::getCostMap(
    std::vector<std::vector<double>> &vvd_cost_map_m)
{
    vvd_cost_map_m = this->vvd_cost_map;
}

std::vector<std::vector<double>> *MobileManipMap::getCostMap()
{
    return &(this->vvd_cost_map);
}


void MobileManipMap::getSlopeMap(
    std::vector<std::vector<double>> &vvd_slope_map_m)
{
    vvd_slope_map_m = this->vvd_slope_map;
}

void MobileManipMap::getSDMap(
		std::vector<std::vector<double>> &vvd_sd_map_m)
{
    vvd_sd_map_m = this->vvd_sd_map;
}

void MobileManipMap::getTraversabilityMap(
    std::vector<std::vector<int>> &vvi_traversability_map_m)
{
    vvi_traversability_map_m = this->vvi_traversability_map;
}

void MobileManipMap::getValidityMap(
    std::vector<std::vector<int8_t>> &vvi_validity_map_m)
{
    vvi_validity_map_m = this->vvi_validity_map;
}

bool MobileManipMap::getElevationMap(
    std::vector<std::vector<double>> &vvd_elevation_map_m)
{
    if (this->mapstate != NO_DEM)
    {
        vvd_elevation_map_m = this->vvd_elevation_map;
        return true;
    }
    else
    {
        return false;
    }
}

bool MobileManipMap::getElevationMapToZero(
    std::vector<std::vector<double>> &vvd_elevation_map_m)
{
    // This is a map for the ArmPlanner library
    if (this->mapstate != NO_DEM)
    {
        vvd_elevation_map_m = this->vvd_elevation_map;
        for (uint j = 0; j < vvd_elevation_map_m.size(); j++)
        {
            for (uint i = 0; i < vvd_elevation_map_m[0].size(); i++)
            {
                if (vvd_elevation_map[j][i] == INFINITY)
		{
                    vvd_elevation_map_m[j][i] = 0.0;
		}
		else
		{
                    vvd_elevation_map_m[j][i] -= getMinElevation();
		}
            }
        }
        return true;
    }
    else
    {
        return false;
    }
}

double MobileManipMap::getMinElevation()
{
    return this->d_elevation_min;
}


double MobileManipMap::getMinReach()
{
    return this->d_minreach_dist;
}

double MobileManipMap::getMaxReach()
{
    return this->d_maxreach_dist;
}



bool MobileManipMap::calculateTraversabilityMap(base::Waypoint w_rover_pos_m)
{

    // Adjust the Avoidance distance according to Rover-Sample distance

    double d_roverToSampleDist = sqrt(pow(w_sample_pos.position[0]
                 - w_rover_pos_m.position[0],2) + pow(w_sample_pos.position[1]
                 - w_rover_pos_m.position[1],2));

    std::cout << "[MM] \033[35m[----------]"
	    " [MobileManipMap::calculateTraversabilityMap()]\033[0m Rover-Sample Distance = " << d_roverToSampleDist << " meters" << std::endl; 

    if (d_roverToSampleDist < this->d_maxreach_dist + 5*1.42*this->d_res)
    {
        std::cout << "[MM] \033[35m[----------]"
	    " [MobileManipMap::calculateTraversabilityMap()]\033[0m Rover-Sample Distance is under " << this->d_maxreach_dist + 2.84*this->d_res << " meters" << std::endl; 
        return false;
    }

    double d_minAvoidDist = (d_roverToSampleDist - this->d_maxreach_dist - 5*1.42*this->d_res);

    std::cout << "[MM] \033[35m[----------]"
	    " [MobileManipMap::calculateTraversabilityMap()]\033[0m The min Obstacle Avoidance Distance is " << d_minAvoidDist << std::endl; 

    d_minAvoidDist = max(0.0, d_minAvoidDist);
	    
    if (this->d_avoid_dist > d_minAvoidDist)
    {
        std::cout << "[MM] \033[35m[----------]"
	    " [MobileManipMap::calculateTraversabilityMap()]\033[0m The obstacle avoidance distance is updated from " << this->d_avoid_dist << " to " << d_minAvoidDist << std::endl; 
        this->d_avoid_dist = d_minAvoidDist;
    }
 
    // FACE distances
    this->d_inner_sampling_dist = this->d_avoid_dist + this->d_maxreach_dist;
    this->d_outter_sampling_dist
        = this->d_inner_sampling_dist
          + 1.42 * this->d_res; // TODO - Maybe this should be 1.42? = sqrt(2)



    std::cout << "[MM] \033[35m[----------]"
	    " [MobileManipMap::calculateTraversabilityMap()]\033[0m d_inner_sampling_dist = " << this->d_inner_sampling_dist << std::endl; 
    
    std::cout << "[MM] \033[35m[----------]"
	    " [MobileManipMap::calculateTraversabilityMap()]\033[0m d_outter_sampling_dist = " << this->d_outter_sampling_dist << std::endl; 


    double d_proximity; 
    // Traversability and Obstacle maps are computed
    for (int j = 0; j < this->ui_num_rows; j++)
    {
        for (int i = 0; i < this->ui_num_cols; i++)
        {

            d_proximity = min(this->vvd_nav_proximity_map[j][i],
			      this->vvd_loc_proximity_map[j][i]);

            if ((this->b_clear_underneath)&&(sqrt(pow((double)i * this->d_res
                 - w_rover_pos_m.position[0],2) + pow((double)j * this->d_res
                 - w_rover_pos_m.position[1],2))
                    <= this->d_occupancy_dist))
            {
                this->vvi_face_obstacle_map[j][i] = 1;
                this->vvi_traversability_map[j][i] = 3;
	    }
	    else if (d_proximity <= 0.0)
            {
                this->vvi_face_obstacle_map[j][i] = 0;
                this->vvi_traversability_map[j][i] = 0;
            }
            else if ((d_proximity <= max(0.0,
                     this->d_dilation*this->d_occupancy_dist - this->d_minreach_dist))
                     || (i == 0) || (j == 0) || (i == this->ui_num_cols - 1)
                     || (j == this->ui_num_rows - 1))
            {
                this->vvi_face_obstacle_map[j][i] = 0;
                this->vvi_traversability_map[j][i] = 1;
            }
            else if (d_proximity
                     <= this->d_dilation*this->d_occupancy_dist)
            {
                if (sqrt(pow((double)i * this->d_res
                                 - this->w_sample_pos.position[0],
                             2)
                         + pow((double)j * this->d_res
                                   - this->w_sample_pos.position[1],
                               2))
                    > this->d_minreach_dist)
                {
                    this->vvi_face_obstacle_map[j][i] = 0;
                    this->vvi_traversability_map[j][i] = 2;
                }
                else
                {
                    this->vvi_face_obstacle_map[j][i] = 1;
                    this->vvi_traversability_map[j][i] = 4;
                }
            }
            else
            {
                if (sqrt(pow((double)i * this->d_res
                                 - this->w_sample_pos.position[0],
                             2)
                         + pow((double)j * this->d_res
                                   - this->w_sample_pos.position[1],
                               2))
                    > this->d_minreach_dist)
                {
                    this->vvi_face_obstacle_map[j][i] = 1;
                    this->vvi_traversability_map[j][i] = 3;
                }
                else
                {
                    this->vvi_face_obstacle_map[j][i] = 1;
                    this->vvi_traversability_map[j][i] = 4;
                }
            }
        }
    }

    this->fm_sample_facing.getShadowedCostMap(this->vvi_face_obstacle_map,
                                              this->d_res,
                                              this->d_inner_sampling_dist,
                                              this->d_outter_sampling_dist,
                                              this->w_sample_pos);
    if (b_debug_mode)
    {
        std::cout << "Shadowing Proccess is computed" << std::endl;
    }


    if (b_debug_mode)
    {
        std::cout << "Obstacle and Traversability Maps are computed"
                  << std::endl;
    }
    return true;
}


bool MobileManipMap::rectifyElevationUnderneath(base::Waypoint w_rover_pos_m)
{
    double d_sum_elevation = 0.0, d_num_valid = 0.0, d_average_elevation;
    std::cout << "[MM] \033[35m[----------] [MobileManipMap::rectifyElevationUnderneath()]\033[0m Getting average elevation value underneath" << std::endl; 
    for (int j = 0; j < this->ui_num_rows; j++)
    {
        for (int i = 0; i < this->ui_num_cols; i++)
        {
            if ((this->vvi_validity_map[j][i] == 1)&&(sqrt(pow((double)i * this->d_res
                 - w_rover_pos_m.position[0],2) + pow((double)j * this->d_res
                 - w_rover_pos_m.position[1],2))
                    <= this->d_occupancy_dist))
            {
                d_sum_elevation += this->vvd_elevation_map[j][i];
		d_num_valid += 1.0;
	    }
	}
    }

    if (d_num_valid < 0.01)
    {
        std::cout << "[MM] \033[35m[----------] [MobileManipMap::rectifyElevationUnderneath()]\033[0m All nodes under " << this->d_occupancy_dist << " m from rover are invalid" << std::endl; 
        std::cout << "[MM] \033[1;35m[----------] [MobileManipMap::rectifyElevationUnderneath()]\033[0m Could not set any elevation data underneath" << std::endl; 
        return false;
    }

    d_average_elevation = d_sum_elevation / d_num_valid;
    
    std::cout << "[MM] \033[35m[----------] [MobileManipMap::rectifyElevationUnderneath()]\033[0m Average elevation underneath is " << d_average_elevation << " m"  << std::endl; 
    for (int j = 0; j < this->ui_num_rows; j++)
    {
        for (int i = 0; i < this->ui_num_cols; i++)
        {
            if ((this->vvi_validity_map[j][i] == 0)&&(sqrt(pow((double)i * this->d_res
                 - w_rover_pos_m.position[0],2) + pow((double)j * this->d_res
                 - w_rover_pos_m.position[1],2))
                    <= this->d_occupancy_dist))
            {
                this->vvd_elevation_map[j][i] = d_average_elevation;
	    }
	}
    }     
    std::cout << "[MM] \033[1;35m[----------] [MobileManipMap::rectifyElevationUnderneath()]\033[0m Finished setting elevation values underneath" << std::endl; 
    return true;
}


bool MobileManipMap::calculateCostMap(base::Waypoint w_rover_pos_m)
{
    Mat mat_proximity_map;
    Mat mat_obstacle_map
        = Mat::zeros(cv::Size(ui_num_cols, ui_num_rows), CV_32FC1);
    for (int j = 0; j < this->ui_num_rows; j++)
    {
        for (int i = 0; i < this->ui_num_cols; i++)
        {
            if (vvi_face_obstacle_map[j][i] == 0)
            {
                mat_obstacle_map.at<float>(j, i) = 0;
            }
            else
            {
                mat_obstacle_map.at<float>(j, i) = 1;
            }
        }
    }
    mat_obstacle_map.convertTo(mat_obstacle_map, CV_8UC1);
    distanceTransform(mat_obstacle_map, mat_proximity_map, DIST_L2, 5);
    mat_proximity_map = mat_proximity_map * this->d_res;
    for (int j = 0; j < this->ui_num_rows; j++)
    {
        for (int i = 0; i < this->ui_num_cols; i++)
        {
            this->vvd_face_proximity_map[j][i] = mat_proximity_map.at<float>(j, i);
        }
    }
    double d_distToRover, d_distToObstacle;
    for (uint j = 0; j < this->ui_num_rows; j++)
    {
        for (uint i = 0; i < this->ui_num_cols; i++)
        {
            // TODO: maybe consider close to border as infinity cost?
            if ((this->vvd_face_proximity_map[j][i] < this->d_res)
                || (this->vvi_traversability_map[j][i] <= 2))
            {
                this->vvd_cost_map[j][i] = INFINITY;
            }
            else if (this->vvd_face_proximity_map[j][i] < 2*this->d_res)
	    {
                this->vvd_cost_map[j][i] = 5.0; //To avoid discontinuities
	    }
	    else if (this->vvd_face_proximity_map[j][i] < this->d_avoid_dist)
            {

                // TODO: this should be configurable
		d_distToRover = max(0.0, 1 - sqrt(pow((double)i * this->d_res
                 - w_rover_pos_m.position[0],2) + pow((double)j * this->d_res
                 - w_rover_pos_m.position[1],2))/this->d_avoid_dist);

		d_distToObstacle = (this->d_avoid_dist
                               - (double)this->vvd_face_proximity_map[j][i])
                            / this->d_avoid_dist;

                this->vvd_cost_map[j][i]
                    = 1.0 + 4.0*max(0.0,(d_distToObstacle - d_distToRover));
            }
            else
            {
                this->vvd_cost_map[j][i] = 1.0;
            }
        }
    }
    return true;
}



double MobileManipMap::getResolution()
{
    return this->d_res;
}

std::vector<double> *MobileManipMap::getPointer2Offset()
{
    return &(this->vd_global_offset);
}

std::vector<double> MobileManipMap::getOffset()
{
    return this->vd_global_offset;
}

void MobileManipMap::setThresholdValues(double d_temptative_slope_threshold, 
		                        double d_temptative_sd_threshold, 
				        double d_temptative_valid_ratio_threshold,
				        double d_temptative_contour_ratio_threshold)
{
 
    if ((d_temptative_slope_threshold < 0.0)||(d_temptative_slope_threshold > 90.0))
    {
        std::cout << "[MM] \033[1;35m[----------] [MobileManipMap::setThresholdValues()]\033[0m Slope threshold = " <<  this->d_slope_threshold << " degrees" << std::endl;
    }
    else
    {
        std::cout << "[MM] \033[35m[----------] [MobileManipMap::setThresholdValues()]\033[0m Slope threshold " <<  this->d_slope_threshold << " to " << d_temptative_slope_threshold << " degrees" << std::endl;
        this->d_slope_threshold = d_temptative_slope_threshold;
    }

    if ((d_temptative_sd_threshold < 0.0)||(d_temptative_sd_threshold > 90.0))
    {
        std::cout << "[MM] \033[35m[----------] [MobileManipMap::setThresholdValues()]\033[0m SD threshold value = " <<  this->d_sd_threshold << " degrees" << std::endl;
    }
    else
    {
        std::cout << "[MM] \033[35m[----------] [MobileManipMap::setThresholdValues()]\033[0m SD threshold " <<  this->d_sd_threshold << " to " << d_temptative_sd_threshold << " degrees" << std::endl;
        this->d_sd_threshold = d_temptative_sd_threshold;
    }

    if ((d_temptative_valid_ratio_threshold < 0.0)||(d_temptative_valid_ratio_threshold > 1.0))
    {
        std::cout << "[MM] \033[35m[----------] [MobileManipMap::setThresholdValues()]\033[0m Valid Ratio threshold = " <<  this->d_valid_ratio_threshold << std::endl;
    }
    else
    {
        std::cout << "[MM] \033[35m[----------] [MobileManipMap::setThresholdValues()]\033[0m Valid Ratio threshold " <<  this->d_valid_ratio_threshold << " to " << d_temptative_valid_ratio_threshold << std::endl;
        this->d_valid_ratio_threshold = d_temptative_valid_ratio_threshold;
    }
    
    if ((d_temptative_contour_ratio_threshold < 0.0)||(d_temptative_contour_ratio_threshold > 1.0))
    {
        std::cout << "[MM] \033[35m[----------] [MobileManipMap::setThresholdValues()]\033[0m Contour Ratio threshold is " <<  this->d_contour_ratio_threshold << std::endl;
    }
    else
    {
        std::cout << "[MM] \033[35m[----------] [MobileManipMap::setThresholdValues()]\033[0m Contour Ratio threshold " <<  this->d_contour_ratio_threshold << " to " << d_temptative_contour_ratio_threshold << std::endl;
        this->d_contour_ratio_threshold = d_temptative_contour_ratio_threshold;
    }
    std::cout << "[MM] \033[1;35m[----------] [MobileManipMap::setThresholdValues()]\033[0m Finished setting Threshold values " << std::endl; 
}


void MobileManipMap::setConfigValues(int i_temptative_close_iter,
		         double d_temptative_avoid_dist, 
                         double d_temptative_occ_radius, 
                         double d_temptative_min_reach,
                         double d_temptative_max_reach,
			 double d_temptative_dilation,
			 bool b_temptative_clear_underneath)
{
    if (i_temptative_close_iter < 0)
    {
        std::cout << "[MM] \033[1;35m[----------] [MobileManipMap::setConfigValues()]\033[0m CLOSE iterations = " <<  this->i_validity_morph_iterations << std::endl;
    }
    else
    {
        std::cout << "[MM] \033[35m[----------] [MobileManipMap::setConfigValues()]\033[0m New value of CLOSE iterations " <<  this->i_validity_morph_iterations << " to " << i_temptative_close_iter << std::endl; 
        this->i_validity_morph_iterations = i_temptative_close_iter;
    }
    
    if (d_temptative_avoid_dist < 0.0)
    {
        std::cout << "[MM] \033[1;35m[----------] [MobileManipMap::setConfigValues()]\033[0m Avoidance distance = " <<  this->d_avoid_dist << std::endl;
    }
    else
    {
        std::cout << "[MM] \033[35m[----------] [MobileManipMap::setConfigValues()]\033[0m Avoidance distance " <<  this->d_avoid_dist << " to " << d_temptative_avoid_dist << std::endl;
        this->d_avoid_dist = d_temptative_avoid_dist;
    }


    if (d_temptative_occ_radius < 0.0)
    {
        std::cout << "[MM] \033[1;35m[----------] [MobileManipMap::setConfigValues()]\033[0m Occupancy radius = " <<  this->d_occupancy_dist << std::endl;
    }
    else
    {
        std::cout << "[MM] \033[35m[----------] [MobileManipMap::setConfigValues()]\033[0m Occupancy radius " <<  this->d_occupancy_dist << " to " << d_temptative_occ_radius << " meters"<< std::endl; 
        this->d_occupancy_dist = d_temptative_occ_radius;
    }

    if ((d_temptative_min_reach < 0.0)||(d_temptative_max_reach < 0.0)||(d_temptative_min_reach > d_temptative_max_reach))
    {
        std::cout << "[MM] \033[1;35m[----------] [MobileManipMap::setConfigValues()]\033[0m Min Reachability = " <<  this->d_minreach_dist << " meters" << std::endl;
        std::cout << "[MM] \033[1;35m[----------] [MobileManipMap::setConfigValues()]\033[0m Min Reachability = " <<  this->d_maxreach_dist << " meters" << std::endl;
    }
    else
    {
        std::cout << "[MM] \033[35m[----------] [MobileManipMap::setConfigValues()]\033[0m Min Reachability " << this->d_minreach_dist << " to " << d_temptative_min_reach << " meters" << std::endl;
        std::cout << "[MM] \033[35m[----------] [MobileManipMap::setConfigValues()]\033[0m Max Reachability " << this->d_maxreach_dist << " to " << d_temptative_max_reach << " meters" << std::endl;
        this->d_minreach_dist = d_temptative_min_reach;
        this->d_maxreach_dist = d_temptative_max_reach;
    }

    if ((d_temptative_dilation < 0.0)||(d_temptative_dilation > 1.0))
    {
        std::cout << "[MM] \033[1;35m[----------] [MobileManipMap::setConfigValues()]\033[0m Dilation Ratio = " <<  this->d_dilation << std::endl;
    }
    else
    {
        std::cout << "[MM] \033[35m[----------] [MobileManipMap::setConfigValues()]\033[0m Dilation Ratio " <<  this->d_dilation << " to " << d_temptative_dilation << std::endl; 
        this->d_dilation = d_temptative_dilation;
	if (this->d_dilation < 1.0)
	{
            std::cout << "[MM] \033[1;35m[-WARNING!--] [MobileManipMap::setConfigValues()]\033[0m Dilation distance is reduced, rover traverse may be compromised by nearby obstacles" << std::endl; 
	}
    }

    std::cout << "[MM] \033[35m[----------] [MobileManipMap::setConfigValues()]\033[0m Custom Traversable underneath " <<  this->b_clear_underneath << " to " << b_temptative_clear_underneath << std::endl; 
        this->b_clear_underneath = b_temptative_clear_underneath;
    if (b_clear_underneath)
    {
        std::cout << "[MM] \033[1;35m[-WARNING!--] [MobileManipMap::setConfigValues()]\033[0m Setting Rover Underneath as traversable, please ensure there is no visible obstacle nearby using external means" << std::endl; 
    }

    std::cout << "[MM] \033[1;35m[----------] [MobileManipMap::setConfigValues()]\033[0m Finished setting Configuration values " << std::endl; 
}


