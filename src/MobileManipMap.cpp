#include "MobileManipMap.h"
#include <iostream>

using namespace std;
using namespace cv;

MobileManipMap::MobileManipMap()
{
    // TODO - implement MobileManipMap::MobileManipMap
}

MobileManipMap::MobileManipMap(const RoverGuidance_Dem &dem)
{
    // TODO - implement MobileManipMap::MobileManipMap
    this->setRGDem(dem);
}

int MobileManipMap::setRGDem(const RoverGuidance_Dem &dem)
{
    // Assignation of DEM parameters
    this->rgDem = dem;
    this->ui_num_cols = dem.cols;
    this->ui_num_rows = dem.rows;
    this->d_res = dem.nodeSize_m;

    // Initialization of vector matrices
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

    if (this->calculateElevationMap())
    {
        this->calculateCostMap();
        return 0;
    }
    else
    {
        return 1;
    }
}

void MobileManipMap::setCostMap(std::vector<std::vector<double>> &costMap)
{
    this->vvd_cost_map.clear();
    std::vector<double> row;
    for (uint j = 0; j < costMap.size(); j++)
    {
        for (uint i = 0; i < costMap[0].size(); i++)
        {
            if (costMap[j][i] > 0)
            {
                row.push_back(costMap[j][i]);
            }
            else
            {
                row.push_back(INFINITY);
            }
        }
        this->vvd_cost_map.push_back(row);
        row.clear();
    }
}

void MobileManipMap::setElevationMap(
    std::vector<std::vector<double>> &elevationMap,
    double res)
{
    this->d_res = res;
    this->vvd_elevation_map.clear();
    std::vector<double> row;
    double d_min_elevation = INFINITY;
    for (uint j = 0; j < elevationMap.size(); j++)
    {
        for (uint i = 0; i < elevationMap[0].size(); i++)
        {
            row.push_back(elevationMap[j][i]);
            if (elevationMap[j][i] < d_min_elevation)
            {
                d_min_elevation = elevationMap[j][i];
            }
        }
        vvd_elevation_map.push_back(row);
        row.clear();
    }
    for (uint j = 0; j < elevationMap.size(); j++)
    {
        for (uint i = 0; i < elevationMap[0].size(); i++)
        {
            vvd_elevation_map[j][i] = vvd_elevation_map[j][i] - d_min_elevation;
        }
    }
}

void MobileManipMap::getCostMap(std::vector<std::vector<double>> &costMap)
{
    costMap = this->vvd_cost_map;
}

void MobileManipMap::getElevationMap(
    std::vector<std::vector<double>> &elevationMap)
{
    elevationMap = this->vvd_elevation_map;
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
                = this->rgDem.p_heightData_m[i + j * this->ui_num_cols];
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

bool MobileManipMap::addSampleFacingObstacles(base::Waypoint sample_pos)
{
    this->fm_sample_facing.getShadowedCostMap(
        this->vvi_traversability_map, this->d_res, 1.5, sample_pos);

    this->calculateProximityToObstaclesMap();

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
                if ((this->vvd_proximity_map[j][i] < 1.0)
                    && (vvi_traversability_map[j][i]
                        < 2)) // ToDo: this is a adhoc risk distance value!!
                {
                    this->vvd_cost_map[j][i]
                        = 1.0
                          + 20.0
                                * (1.0 - (double)this->vvd_proximity_map[j][i]);
                }
                else
                {
                    if (this->vvd_proximity_map[j][i] < 1.01 * this->d_res)
                    {
                        this->vvd_cost_map[j][i]
                            = 1.0
                              + 20.0
                                    * (1.0
                                       - (double)this->vvd_proximity_map[j][i]);
                    }
                    else
                    {
                        this->vvd_cost_map[j][i] = 1.0;
                    }
                }
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
                    < 1.0) // ToDo: this is a adhoc risk distance value!!
                {
                    this->vvd_cost_map[j][i]
                        = 1.0
                          + 20.0
                                * (1.0 - (double)this->vvd_proximity_map[j][i]);
                }
                else
                {
                    this->vvd_cost_map[j][i] = 1.0;
                }
            }
        }
    }
}

bool MobileManipMap::calculateCostMap()
{
    // Compute vvb_obstacle_map
    this->calculateTraversabilityMap();
    // Compute vvd_proximity_map
    this->calculateProximityToObstaclesMap();
    // Compute vvd_cost_map
    this->calculateCostValues();
}

double MobileManipMap::getResolution()
{
    return this->d_res;
}
