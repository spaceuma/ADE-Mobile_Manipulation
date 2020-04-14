#include "MobileManipMap.h"
#include <iostream>

using namespace std;
using namespace cv;

MobileManipMap::MobileManipMap(const RoverGuidance_Dem &dem)
{
    try
    {
        // Assignation of DEM parameters
        this->loadDEM(dem);
        // Initialization of vector matrices
        this->initializeMatrices();
    }
    catch (bad_alloc &ba)
    {
        cout
            << ba.what()
            << " exception occured while allocating memory for the map matrices"
            << endl;
        throw ba;
    }
    catch (exception &e)
    {
        cout << " An exception occured while reading DEM" << endl;
        throw e;
    }

    try
    {
        this->calculateElevationMap();
        // Compute vvb_obstacle_map
        this->calculateTraversabilityMap();
        // Compute vvd_proximity_map
        this->calculateProximityToObstaclesMap();
        // Compute vvd_cost_map
        this->calculateCostValues();
    }
    catch (exception &e)
    {
        cout << " An exception occured while calculating the cost map" << endl;
        throw e;
    }
}

MobileManipMap::MobileManipMap(const RoverGuidance_Dem &dem,
                               base::Waypoint w_sample_pos_m)
{
    try
    {
        // Assignation of DEM parameters
        this->loadDEM(dem);
        // Initialization of vector matrices
        this->initializeMatrices();
    }
    catch (bad_alloc &ba)
    {
        cout
            << ba.what()
            << " exception occured while allocating memory for the map matrices"
            << endl;
        throw ba;
    }
    catch (exception &e)
    {
        cout << " An exception occured while reading DEM" << endl;
        throw e;
    }

    try
    {
        this->loadSample(w_sample_pos_m);
    }
    catch (exception &e)
    {
        cout << " An exception occured while loading the Sample position"
             << endl;
        throw e;
    }
    try
    {
        this->calculateElevationMap();
        // Compute vvb_obstacle_map
        this->calculateTraversabilityMap();
        // Compute vvd_proximity_map
        this->addSampleFacingObstacles();
    }
    catch (exception &e)
    {
        cout << " An exception occured while calculating the cost map" << endl;
        throw e;
    }
}

MobileManipMap::MobileManipMap(
    std::vector<std::vector<double>> &vvd_elevation_map_m,
    std::vector<std::vector<double>> &vvd_cost_map_m,
    double d_res_m)
{
    this->d_res = d_res_m;
    this->vvd_elevation_map.clear();
    std::vector<double> row;
    this->d_elevation_min = INFINITY;
    this->d_inner_sampling_dist = this->d_avoid_dist + 0.94;//TODO - 0.94 should come from max reachability 
    this->d_outter_sampling_dist = this->d_inner_sampling_dist + 1.72*this->d_res;
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
}

void MobileManipMap::computeFACE(base::Waypoint w_sample_pos_m)
{
    try
    {
        this->loadSample(w_sample_pos_m);
    }
    catch (exception &e)
    {
        cout << " An exception occured while loading the Sample position"
             << endl;
        throw e;
    }
    try
    {
        this->calculateElevationMap();
        // Compute vvb_obstacle_map
        this->calculateTraversabilityMap();
        // Compute vvd_proximity_map
        this->addSampleFacingObstacles();
    }
    catch (exception &e)
    {
        cout << " An exception occured while calculating the cost map" << endl;
        throw e;
    }

}


void MobileManipMap::loadDEM(const RoverGuidance_Dem &dem)
{
    if (dem.nodeSize_m <= 0)
    {
        cout << " MobileManipMap Constructor EXCEPTION: DEM node size is "
                "nonvalid with value = "
             << dem.nodeSize_m << endl;
        throw exception();
    }
    this->d_res = dem.nodeSize_m;
    if (dem.cols < 5)
    {
        cout << " MobileManipMap Constructor EXCEPTION: DEM number of columns "
                "is nonvalid with value = "
             << dem.cols << endl;
        throw exception();
    }
    this->ui_num_cols = dem.cols;
    if (dem.rows < 5)
    {
        cout << " MobileManipMap Constructor EXCEPTION: DEM number of rows is "
                "nonvalid with value = "
             << dem.rows << endl;
        throw exception();
    }
    this->ui_num_rows = dem.rows;
    this->rg_dem = dem;
    this->d_res = dem.nodeSize_m;
    this->d_inner_sampling_dist = this->d_avoid_dist + 0.94;//TODO - 0.94 should come from max reachability 
    this->d_outter_sampling_dist = this->d_inner_sampling_dist + 1.72*this->d_res;
}

void MobileManipMap::loadSample(const base::Waypoint &w_sample_pos_m)
{
    if (isOutside(w_sample_pos_m))
    {
        cout << " MobileManipMap Constructor EXCEPTION: the sample is out of "
                "the map"
             << endl;
        throw exception();
    }
    this->w_sample_pos = w_sample_pos_m;
}

bool MobileManipMap::isOutside(const base::Waypoint &w_sample_pos_m)
{
    if ((w_sample_pos_m.position[0] < this->d_res)
        || (w_sample_pos_m.position[1] < this->d_res)
        || (w_sample_pos_m.position[0]
            > ((double)this->ui_num_cols - 2) * this->d_res)
        || (w_sample_pos_m.position[1]
            > ((double)this->ui_num_rows - 2) * this->d_res))
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool MobileManipMap::isObstacle(const base::Waypoint w_pos_m)
{
    std::vector<int> vi_pos(2, 0);
    vi_pos[0] = (int)(w_pos_m.position[0] / this->d_res + 0.5);
    vi_pos[1] = (int)(w_pos_m.position[1] / this->d_res + 0.5);
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

void MobileManipMap::initializeMatrices()
{
    this->vvd_elevation_map.clear();
    this->vvi_traversability_map.clear();
    this->vvd_cost_map.clear();
    this->vvd_proximity_map.clear();
    std::vector<double> vd_row(this->ui_num_cols);
    std::vector<int> vi_row(this->ui_num_cols);
    for (uint j = 0; j < this->ui_num_rows; j++)
    {
        this->vvd_elevation_map.push_back(vd_row);
        this->vvi_traversability_map.push_back(vi_row);
        this->vvd_cost_map.push_back(vd_row);
        this->vvd_proximity_map.push_back(vd_row);
    }
}
void MobileManipMap::getCostMap(
    std::vector<std::vector<double>> &vvd_cost_map_m)
{
    vvd_cost_map_m = this->vvd_cost_map;
}

void MobileManipMap::getElevationMap(
    std::vector<std::vector<double>> &vvd_elevation_map_m)
{
    vvd_elevation_map_m = this->vvd_elevation_map;
}

void MobileManipMap::getElevationMapToZero(
    std::vector<std::vector<double>> &vvd_elevation_map_m)
{
    vvd_elevation_map_m = this->vvd_elevation_map;
    for (uint j = 0; j < vvd_elevation_map_m.size(); j++)
    {
        for (uint i = 0; i < vvd_elevation_map_m[0].size(); i++)
        {
            vvd_elevation_map_m[j][i] -= getMinElevation();
        }
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
        }
    }
    return true;
}

double MobileManipMap::getMinElevation()
{
    return this->d_elevation_min;
}

bool MobileManipMap::calculateTraversabilityMap()
{
    Mat mat_elevation_map
        = Mat::zeros(cv::Size(ui_num_rows, ui_num_cols), CV_64F);
    Mat mat_obstacle_map
        = Mat::zeros(cv::Size(ui_num_rows, ui_num_cols), CV_32FC1);
    Mat mat_slope_map
        = Mat::zeros(cv::Size(ui_num_rows, ui_num_cols), CV_32FC1);
    Mat dx, dy, elev;
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

    // Slope Mat is computed
    mat_elevation_map.convertTo(elev, CV_32F, 1.0, 0);
    Sobel(elev, dx, CV_32F, 1, 0, 3, scale, delta, BORDER_DEFAULT);
    Sobel(elev, dy, CV_32F, 0, 1, 3, scale, delta, BORDER_DEFAULT);
    Mat angle, mag, mat_proximity_map;
    cartToPolar(dx, dy, mag, angle);

    for (int j = 0; j < mat_slope_map.rows; j++)
    {
        for (int i = 0; i < mat_slope_map.cols; i++)
        {
            mat_slope_map.at<float>(j, i)
                = atan(mag.at<float>(j, i) * 1.0 / this->d_res) * 180.0
                  / 3.1416;
        }
    }

    threshold(mat_slope_map, mat_obstacle_map, 20.0, 255, THRESH_BINARY_INV);

    mat_obstacle_map.convertTo(mat_obstacle_map, CV_8UC1);

    distanceTransform(mat_obstacle_map, mat_proximity_map, DIST_L2, 5);
    mat_proximity_map = mat_proximity_map * this->d_res;
    threshold(mat_proximity_map, mat_proximity_map, 0.5, 0, THRESH_TOZERO);

    // Borders are considered obstacles
    for (int j = 0; j < this->ui_num_rows; j++)
    {
        for (int i = 0; i < this->ui_num_cols; i++)
        {
            if ((mat_proximity_map.at<float>(j, i) == 0) || (i == 0) || (j == 0)
                || (i == this->ui_num_cols - 1)
                || (j == this->ui_num_rows - 1))
            {
                this->vvi_traversability_map[j][i] = 0;
            }
            else
            {
                this->vvi_traversability_map[j][i] = 1;
            }
        }
    }
    return true;
}

bool MobileManipMap::addSampleFacingObstacles()
{
    this->fm_sample_facing.getShadowedCostMap(
        this->vvi_traversability_map, this->d_res, this->d_inner_sampling_dist, this->d_outter_sampling_dist, this->w_sample_pos);

    this->calculateProximityToObstaclesMap();

    for (uint j = 0; j < this->ui_num_rows; j++)
    {
        for (uint i = 0; i < this->ui_num_cols; i++)
        {
            if (this->vvd_proximity_map[j][i] < this->d_res)
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
        = Mat::zeros(cv::Size(ui_num_rows, ui_num_cols), CV_32FC1);
    for (int j = 0; j < this->ui_num_rows; j++)
    {
        for (int i = 0; i < this->ui_num_cols; i++)
        {
            if (vvi_traversability_map[j][i] == 0)
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
            vvd_proximity_map[j][i] = mat_proximity_map.at<float>(j, i);
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
                    < this->d_avoid_dist) // ToDo: this is a adhoc risk distance value!!
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
