#include "MobileManipMap.h"
#include <iostream>

using namespace std;
using namespace cv;

MobileManipMap::MobileManipMap(bool b_debug_mode_m)
{
    mapstate = NO_DEM;
    this->b_debug_mode = b_debug_mode_m;
}


MobileManipMap::MobileManipMap(const RoverGuidance_Dem &dem, unsigned int &ui_isDEM_loaded, bool b_debug_mode_m)
{
    mapstate = NO_DEM;
    ui_isDEM_loaded = this->loadDEM(dem);
    if (ui_isDEM_loaded == 0)
    {
        mapstate = DEM_LOADED;
    }
    this->b_debug_mode = b_debug_mode_m;
}

MobileManipMap::MobileManipMap(
    std::vector<std::vector<double>> &vvd_elevation_map_m,
    std::vector<std::vector<double>> &vvd_cost_map_m,
    double d_res_m, base::Waypoint w_sample_pos_m,
    double d_avoid_dist_m, double d_maxreach_dist_m)
{
    this->d_res = d_res_m;
    this->d_avoid_dist = d_avoid_dist_m;
    this->d_maxreach_dist = d_maxreach_dist_m;
    this->vvd_elevation_map.clear();
    std::vector<double> row;
    this->d_elevation_min = INFINITY;
    this->d_inner_sampling_dist = this->d_avoid_dist + this->d_maxreach_dist; 
    this->d_outter_sampling_dist = this->d_inner_sampling_dist + 1.72*this->d_res; //TODO - Maybe this should be 1.42? = sqrt(2)
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

unsigned int MobileManipMap::computeFACE(base::Waypoint w_sample_pos_m,
		double d_avoid_dist_m, double d_maxreach_dist_m)
{
    if (mapstate == NO_DEM)
    {
        return 1; // Error: there is no existing DEM
    }
    else
    {
        mapstate = DEM_LOADED;
    }
    if(!this->loadGlobalSample(w_sample_pos_m))
    {
        return 2; // Error: sample out of the DEM
    }

    // FACE distances
    this->d_avoid_dist = d_avoid_dist_m;
    this->d_maxreach_dist = d_maxreach_dist_m;
    this->d_inner_sampling_dist = this->d_avoid_dist + this->d_maxreach_dist; 
    this->d_outter_sampling_dist = this->d_inner_sampling_dist + 1.72*this->d_res; //TODO - Maybe this should be 1.42? = sqrt(2)
 
    try
    {
        // Compute vvb_obstacle_map
        this->calculateTraversabilityMap();
	if (this->b_debug_mode)
	{
	    std::cout << "Computed Traversability Map" << std::endl;
	}
        // Compute vvd_proximity_map
        this->addSampleFacingObstacles();
	if (this->b_debug_mode)
	{
	    std::cout << "Computed FACE" << std::endl;
	}
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


unsigned int MobileManipMap::loadDEM(const RoverGuidance_Dem &dem)
{
    this->mapstate = NO_DEM;
    // Check Resolution
    if (dem.nodeSize_m <= 0)
    {
        return 1;
    }
    this->d_res = dem.nodeSize_m;
    // Check Rows
    if (dem.rows < 5)
    {
        return 2;
    }
    
    this->ui_num_rows = dem.rows;

    if (this->b_debug_mode)
    {
        std::cout << "Loading DEM: the number of rows is " << this->ui_num_rows << std::endl;
    }

    // Check Columns
    if (dem.cols < 5)
    {
        return 3;
    }
    this->ui_num_cols = dem.cols;

    if (this->b_debug_mode)
    {
        std::cout << "Loading DEM: the number of columns is " << this->ui_num_cols << std::endl;
    }

    // The offset is read, it is supposed to be a 3-element array
    try
    {
        for (unsigned int i = 0; i < 3; i++)
        {
            this->vd_global_offset[i] = dem.mapOrigin_m_Mlg[i];
        }
    }
    catch (exception &e)
    {
        return 4;
    }

    // DEM is stored
    this->rg_dem = dem;
  
    // Initialization of matrices
    try
    {
        this->vvd_elevation_map.clear();
        this->vvd_slope_map.clear();
        this->vvd_sd_map.clear();
        this->vvi_obstacle_map.clear();
        this->vvi_traversability_map.clear();
        this->vvd_cost_map.clear();
        this->vvd_proximity_map.clear();
        std::vector<double> vd_row(this->ui_num_cols);
        std::vector<int> vi_row(this->ui_num_cols);
        std::vector<int8_t> vit_row(this->ui_num_cols);
        for (uint j = 0; j < this->ui_num_rows; j++)
        {
            this->vvd_elevation_map.push_back(vd_row);
            this->vvi_validity_map.push_back(vit_row);
            this->vvd_slope_map.push_back(vd_row);
            this->vvd_aspect_map.push_back(vd_row);
            this->vvd_sd_map.push_back(vd_row);
            this->vvd_nx_map.push_back(vd_row);
            this->vvd_ny_map.push_back(vd_row);
            this->vvd_nz_map.push_back(vd_row);
            this->vvi_obstacle_map.push_back(vi_row);
            this->vvi_traversability_map.push_back(vi_row);
            this->vvd_cost_map.push_back(vd_row);
            this->vvd_proximity_map.push_back(vd_row);
        }        // Assignation of DEM parameters
        this->calculateElevationMap(); 
	mapstate = DEM_LOADED;
    }
    catch (bad_alloc &ba)
    {
	mapstate = NO_DEM;
	return 5;
    }   
    return 0;
}

bool MobileManipMap::loadGlobalSample(const base::Waypoint &w_sample_pos_m)
{
    base::Waypoint w_sample_localpos;
    w_sample_localpos.position[0] = w_sample_pos_m.position[0] - this->vd_global_offset[0];
    w_sample_localpos.position[1] = w_sample_pos_m.position[1] - this->vd_global_offset[1];
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
        std::cout << "[INFO] [MobileManipMap::isOutside] - dem res: " << this->d_res << "; cols: " << this->ui_num_cols << "; rows: " << this->ui_num_rows << "\n";
        return true;
    }
    else
    {
        return false;
    }
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

std::vector<std::vector<double>> *MobileManipMap::getCostMap()
{
    return &(this->vvd_cost_map);
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
    if (this->mapstate != NO_DEM)
    {
        vvd_elevation_map_m = this->vvd_elevation_map;
        for (uint j = 0; j < vvd_elevation_map_m.size(); j++)
        {
            for (uint i = 0; i < vvd_elevation_map_m[0].size(); i++)
            {
                vvd_elevation_map_m[j][i] -= getMinElevation();
            }
        }
	return true;
    }
    else
    {
        return false;
    }	
}


bool MobileManipMap::calculateElevationMap()
{
    std::vector<double> row;
    d_elevation_min = INFINITY;
    
    
    for (int j = 0; j < this->ui_num_rows; j++)
    {
        for (int i = 0; i < this->ui_num_cols; i++)
        {
            this->vvd_elevation_map[j][i]
                = this->rg_dem.p_heightData_m[i + j * this->ui_num_cols];
            if (this->vvd_elevation_map[j][i] < this->d_elevation_min)
            {
                this->d_elevation_min = this->vvd_elevation_map[j][i];
            }
	    this->vvi_validity_map[j][i] = this->rg_dem.p_pointValidityFlag[i + j * this->ui_num_cols];

        }
    }

    if (this->b_debug_mode)
    {
        std::cout << "Elevation Map is saved as vector<vector<double>>" << std::endl;
    }

    double dx,dy;
    for (int j = 1; j < this->ui_num_rows - 1; j++)
    {
        for (int i = 1; i < this->ui_num_cols - 1; i++)
        {
            if (this->vvi_validity_map[j][i+1] == 0)
	    {
                if(this->vvi_validity_map[j][i-1] == 0)
		{
                    dx = 0;
		}
		else
		{
                    dx = (this->vvd_elevation_map[j][i] - this->vvd_elevation_map[j][i-1])/(this->d_res);
		}
	    }
	    else if (this->vvi_validity_map[j][i-1] == 0)
	    {
                dx = (this->vvd_elevation_map[j][i+1] - this->vvd_elevation_map[j][i])/(this->d_res);
	    }
	    else
	    {
                dx = (this->vvd_elevation_map[j][i+1] - this->vvd_elevation_map[j][i-1])/(2*this->d_res);
	    }
            if (this->vvi_validity_map[j+1][i] == 0)
	    {
                if(this->vvi_validity_map[j-1][i] == 0)
		{
                    dy = 0;
		}
		else
		{
                    dy = (this->vvd_elevation_map[j][i] - this->vvd_elevation_map[j-1][i])/(this->d_res);
		}
	    }
	    else if (this->vvi_validity_map[j-1][i] == 0)
	    {
                dy = (this->vvd_elevation_map[j+1][i] - this->vvd_elevation_map[j][i])/(this->d_res);
	    }
	    else
	    {
                dy = (this->vvd_elevation_map[j+1][i] - this->vvd_elevation_map[j-1][i])/(2*this->d_res);
	    }

            this->vvd_slope_map[j][i] = atan(sqrt(pow(dx,2) + pow(dy,2)));
	    this->vvd_aspect_map[j][i] = atan2(dy,dx);
	    this->vvd_nx_map[j][i] = sin(this->vvd_aspect_map[j][i])*sin(this->vvd_slope_map[j][i]);
	    this->vvd_ny_map[j][i] = cos(this->vvd_aspect_map[j][i])*sin(this->vvd_slope_map[j][i]);
	    this->vvd_nz_map[j][i] = cos(this->vvd_slope_map[j][i]); 
        }
    }

    int i_occupancy_kernel = (int) (.5 / this->d_res);
    int i_nodes;
    double d_sumnx, d_sumny, d_sumnz, d_R;
    for (int j = 1; j < this->ui_num_rows - 1; j++)
    {
        for (int i = 1; i < this->ui_num_cols - 1; i++)
        {
            i_nodes = 0;
	    d_sumnx = 0;
	    d_sumny = 0;
	    d_sumnz = 0;
            for (int l = - i_occupancy_kernel; l <= i_occupancy_kernel ; l++)
	    {
                for (int k = - i_occupancy_kernel; k <= i_occupancy_kernel ; k++)
		{
                    if((j+l >= 0)&&(j+l < this->ui_num_rows - 1)&&(i+k >= 0)&&(i+k < this->ui_num_cols - 1)&&(sqrt(pow((double)l,2)+pow((double)k,2)) < (double)i_occupancy_kernel))
		    {
                        i_nodes++;
			d_sumnx += this->vvd_nx_map[j + l][i + k]; 
			d_sumny += this->vvd_ny_map[j + l][i + k];
			d_sumnz += this->vvd_nz_map[j + l][i + k];
		    }
		}
	    }
	    d_R = sqrt(pow(d_sumnx,2)+pow(d_sumny,2)+pow(d_sumnz,2)); 
	    this->vvd_sd_map[j][i] = sqrt( - 2 * log(d_R/(double)i_nodes)) * 180.0/3.1416; 
	}
    }


    if (this->b_debug_mode)
    {
        std::cout << "Slope, Aspect and Normal Vector Components Maps are computed" << std::endl;
	std::cout << "i_occupancy_kernel = " << i_occupancy_kernel << std::endl;
    }

    return true;
}

double MobileManipMap::getMinElevation()
{
    return this->d_elevation_min;
}

bool MobileManipMap::calculateTraversabilityMap()
{
    // TODO - Detect whether the sample is at a distance from obstacles too close, being UNREACHABLE
    Mat mat_elevation_map
        = Mat::zeros(cv::Size(ui_num_cols, ui_num_rows), CV_64F);
    Mat mat_obstacle_map
        = Mat::zeros(cv::Size(ui_num_cols, ui_num_rows), CV_32FC1);
    Mat mat_slope_map
        = Mat::zeros(cv::Size(ui_num_cols, ui_num_rows), CV_32FC1);
    Mat dx, dy, elev, angle, mag, mat_proximity_map;
    
    double scale = 0.125; // 1/8 to normalize sobel filter
    double delta = 0;

    // Elevation Mat is initialized
    for (int j = 0; j < this->ui_num_rows; j++)
    {
        for (int i = 0; i < this->ui_num_cols; i++)
        {
            mat_elevation_map.at<double>(j, i) = this->vvd_elevation_map[j][i];
        }
    }
   
    if (b_debug_mode)
    {
        std::cout << "Elevation Mat is initialized" << std::endl;
        imshow("Elevation Matrix", mat_elevation_map);
	waitKey(0);
    }

    // Slope Mat is computed
    mat_elevation_map.convertTo(elev, CV_32F, 1.0, 0);
    Sobel(elev, dx, CV_32F, 1, 0, 3, scale, delta, BORDER_DEFAULT);
    Sobel(elev, dy, CV_32F, 0, 1, 3, scale, delta, BORDER_DEFAULT);
    cartToPolar(dx, dy, mag, angle);

    for (int j = 0; j < mat_slope_map.rows; j++)
    {
        for (int i = 0; i < mat_slope_map.cols; i++)
        {
            mat_slope_map.at<float>(j, i)
		    = (float)this->vvd_slope_map[j][i] * 180.0/3.1416;
                //= atan(mag.at<float>(j, i) * 1.0 / this->d_res) * 180.0
                  /// 3.1416;
            /*
	    if (this->vvi_validity_map[j][i] == 0)
	    {
                 mat_slope_map.at<float>(j, i)
		    = 0.0;
	    }
	    */
        }
    }

    if (b_debug_mode)
    {
        std::cout << "Slope Mat is computed" << std::endl;
        imshow("Slope Matrix", mat_slope_map);
	waitKey(0);
    }

    // Obstacle Mat is computed
    threshold(mat_slope_map, mat_obstacle_map, 30.0, 255, THRESH_BINARY_INV);//TODO-Include here configurable parameter for slope threshold
    if (b_debug_mode)
    {
        std::cout << "A Threshold of 30.0 degrees is applied to the slope mat" << std::endl;
        imshow("Obstacle Matrix", mat_obstacle_map);
	waitKey(0);
    }

    mat_obstacle_map.convertTo(mat_obstacle_map, CV_8UC1);
    if (b_debug_mode)
    {
        std::cout << "Obstacle Mat is computed" << std::endl;
    }

    // Preliminar Proximity map is computed
    distanceTransform(mat_obstacle_map, mat_proximity_map, DIST_L2, 5);
    mat_proximity_map = mat_proximity_map * this->d_res;

    if (b_debug_mode)
    {
        std::cout << "Preliminar Proximity Map is computed" << std::endl;
        imshow("Proximity Matrix", mat_proximity_map);
	waitKey(0);
    }

    // Traversability and Obstacle maps are computed
    for (int j = 0; j < this->ui_num_rows; j++)
    {
        for (int i = 0; i < this->ui_num_cols; i++)
        {
            if (mat_proximity_map.at<float>(j, i) <= 0.0)
	    {
                this->vvi_obstacle_map[j][i] = 0;
                this->vvi_traversability_map[j][i] = 0;
	    }
	    else if ((mat_proximity_map.at<float>(j, i) <= this->d_maxreach_dist) || 
			    (i == 0) || (j == 0)
                || (i == this->ui_num_cols - 1)
                || (j == this->ui_num_rows - 1))
            {
                this->vvi_obstacle_map[j][i] = 0;
                this->vvi_traversability_map[j][i] = 1;
            }
            else if (mat_proximity_map.at<float>(j, i) <= this->d_occupancy_dist) 
            {
                if (sqrt(pow((double)i*this->d_res - 
			       this->w_sample_pos.position[0],2)+pow((double)j*this->d_res - 
				       this->w_sample_pos.position[1],2)) > this->d_maxreach_dist)
		{
                    this->vvi_obstacle_map[j][i] = 0;
                    this->vvi_traversability_map[j][i] = 2;
		}
		else
		{
                    this->vvi_obstacle_map[j][i] = 1;
                    this->vvi_traversability_map[j][i] = 4;
		}

	    }
            else
	    {
		if (sqrt(pow((double)i*this->d_res - 
			       this->w_sample_pos.position[0],2)+pow((double)j*this->d_res - 
				       this->w_sample_pos.position[1],2)) > 0.94)
		{
                    this->vvi_obstacle_map[j][i] = 1;
                    this->vvi_traversability_map[j][i] = 3;
		}
		else
		{
                    this->vvi_obstacle_map[j][i] = 1;
                    this->vvi_traversability_map[j][i] = 4;
		}
	    }	    
        }
    }
    if (b_debug_mode)
    {
        std::cout << "Obstacle and Traversability Maps are computed" << std::endl;
    }
    return true;
}

bool MobileManipMap::addSampleFacingObstacles()
{
    this->fm_sample_facing.getShadowedCostMap(this->vvi_obstacle_map, this->d_res, this->d_inner_sampling_dist, this->d_outter_sampling_dist, this->w_sample_pos);
    if (b_debug_mode)
    {
        std::cout << "Shadowing Proccess is computed" << std::endl;
    }
    this->calculateProximityToObstaclesMap();

    for (uint j = 0; j < this->ui_num_rows; j++)
    {
        for (uint i = 0; i < this->ui_num_cols; i++)
        {
            if ((this->vvd_proximity_map[j][i] < this->d_res)||(this->vvi_traversability_map[j][i] <= 2))
            {
                this->vvd_cost_map[j][i] = INFINITY;
            }
            else if (this->vvd_proximity_map[j][i] < this->d_avoid_dist)
            {
                this->vvd_cost_map[j][i]
                        = 1.0
                          + 10.0
                                * (this->d_avoid_dist - (double)this->vvd_proximity_map[j][i])/this->d_avoid_dist;
            }
	    else
            {
                this->vvd_cost_map[j][i] = 1.0;
	    }
        }
    }
    return true;
}

bool MobileManipMap::calculateProximityToObstaclesMap()
{
    Mat mat_proximity_map;
    Mat mat_obstacle_map
        = Mat::zeros(cv::Size(ui_num_cols, ui_num_rows), CV_32FC1);
    for (int j = 0; j < this->ui_num_rows; j++)
    {
        for (int i = 0; i < this->ui_num_cols; i++)
        {
            if (vvi_obstacle_map[j][i] == 0)
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
            this->vvd_proximity_map[j][i] = mat_proximity_map.at<float>(j, i);
        }
    }
    return true;
}

void MobileManipMap::calculateCostValues()
{
    for (uint j = 0; j < this->ui_num_rows; j++)
    {
        for (uint i = 0; i < this->ui_num_cols; i++)
        {
            if (this->vvd_proximity_map[j][i] < this->d_res)
            {
                this->vvd_cost_map[j][i] = INFINITY;
            }
            else
            {
                if (this->vvd_proximity_map[j][i]
                    < this->d_avoid_dist) 
                {
                    this->vvd_cost_map[j][i]
                        = 1.0 + 10.0* (this->d_avoid_dist - (double)this->vvd_proximity_map[j][i])/this->d_avoid_dist;
                }
                else
                {
                    this->vvd_cost_map[j][i] = 1.0;
                }
            }
        }
    }
}

double MobileManipMap::getResolution()
{
    return this->d_res;
}

std::vector<double> MobileManipMap::getOffset()
{
    return this->vd_global_offset;
}
