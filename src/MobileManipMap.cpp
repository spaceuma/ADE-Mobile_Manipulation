#include "MobileManipMap.h"
#include <iostream>

using namespace std;
using namespace cv;

MobileManipMap::MobileManipMap() {
	// TODO - implement MobileManipMap::MobileManipMap
}

MobileManipMap::MobileManipMap(RoverGuidance_Dem dem) {
	// TODO - implement MobileManipMap::MobileManipMap
  this->setRGDem(dem);
}

int MobileManipMap::setRGDem(RoverGuidance_Dem &dem){
  // TODO - implement MobileManipMap::setDem
  cout << "" << endl;
  /*for (uint j = 0; j<dem.rows; j++)
  {
        for (uint i = 0; i<dem.cols; i++)
        {
            std::cout << "Initial Value is " << dem.p_heightData_m[i+j*dem.cols] << std::endl;
        }
  }*/
  this->rgDem = dem;
  this->offsetXYZ[0] = dem.mapOrigin_m_Mlg[0];
  this->offsetXYZ[1] = dem.mapOrigin_m_Mlg[1];
  this->offsetXYZ[2] = dem.mapOrigin_m_Mlg[2];
  this->numXnodes = dem.cols;
  this->numYnodes = dem.rows;
  this->resDem = dem.nodeSize_m;
  /*for (uint j = 0; j<this->numYnodes; j++)
  {
        for (uint i = 0; i<this->numXnodes; i++)
        {
            std::cout << "Initial Value is " << dem.p_heightData_m[i+j*dem.cols] << std::endl;
            std::cout << "Current Value is " << this->rgDem.p_heightData_m[i+j*this->numXnodes] << std::endl;
        }
  }*/

  std::cout << " Number of columns = " << this->numXnodes << std::endl;
  std::cout << " Number of rows = " << this->numYnodes << std::endl;
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

int MobileManipMap::setImageDem(Mat inputDem, double resDem)
{
  flip(inputDem, this->matElevationMap, 0); //0 = flip upside down
  this->resDem = resDem;
  if (this->calculateCostMap())
  {
    return 0;
  }
  else
  {
    return 1;
  }
}

void MobileManipMap::getCostMap(std::vector<std::vector<double>> &costMap){
  costMap = currentCostMap;
}

void MobileManipMap::getElevationMap(std::vector<std::vector<double>> &elevationMap){
  elevationMap = this->vecElevationMap;
}

bool MobileManipMap::calculateElevationMap() {
  // TODO - implement MobileManipMap::calculateElevationMap
  this->matElevationMap = Mat::zeros(cv::Size(numYnodes,numXnodes), CV_64F);
  this->vecElevationMap.clear();
  std::vector<double> row;
  //this->elevationMap.resize(numYnodes);
  for (int j = 0; j < this->numYnodes; j++)
  {
    //double* p  = this->elevationMap.ptr<double>(j);
    for (int i = 0; i < this->numXnodes; i++)
    {
      try
      {
        this->matElevationMap.at<double>(j,i) = this->rgDem.p_heightData_m[i + j*this->numXnodes];
        /*cout << "Index " << i + j*this->numXnodes << endl;
        cout << "Node " << i << "," << j << endl;
	cout << "Elevation is " << elevationMap.at<double>(j,i) << endl;
	cout << "Initial Value was " << this->rgDem.p_heightData_m[i + j*this->numXnodes] << endl;*/
	//p[i] = this->rgDem.p_heightData_m[i + j*i];
        //cout << "Elevation is " << p[i] << endl;
      }
      catch(...)
      {
        cout << "MMMap: Exception occurred reading column " << i << ", row " << j << endl;
        cout << "MMMap: input dem size shall be " << this->numXnodes << "x" << this->numYnodes << endl;
        throw;
      }
    }
  }
  double min, max;
  cv::minMaxLoc(this->matElevationMap, &min, &max);
  for (int j = 0; j < this->numYnodes; j++)
  {
    for (int i = 0; i < this->numXnodes; i++)
    {
      row.push_back( this->rgDem.p_heightData_m[i + j*this->numXnodes] - min );
    }
    this->vecElevationMap.push_back( row );
    row.clear();
  }
  return true;
}

void MobileManipMap::showElevationMap()
{
  Mat mapToShow;
  //flip(this->elevationMap, mapToShow,0);
  double min, max;
  minMaxLoc(this->matElevationMap, &min, &max);
  //cout << " The min is " << min << " and the max " << max << endl;
  mapToShow = this->matElevationMap - min;
  flip(mapToShow, mapToShow,0);
  mapToShow.convertTo(mapToShow, CV_32F, 1.0 / (max-min), 0);
  namedWindow("Elevation Map", WINDOW_NORMAL);
  imshow("Elevation Map", mapToShow);
}

void MobileManipMap::showSlopeMap()
{
  Mat flippedMap, mapToShow;
  flip(this->slopeMap, flippedMap,0);
  flippedMap.convertTo(mapToShow, CV_32F, 1.0 / 90.0, 0);
  namedWindow("Slope Map", WINDOW_NORMAL);
  imshow("Slope Map", mapToShow);

  /*FileStorage fs("slopeMap.yml", FileStorage::WRITE);
  fs << "SlopeMap" << this->slopeMap;
  fs.release();*/
}

void MobileManipMap::showObstacleMap()
{
  Mat flippedMap, mapToShow;
  double min, max;
  minMaxLoc(this->obstacleMap, &min, &max);
  cout << " The min is " << min << " and the max " << max << endl;
  flip(this->obstacleMap, flippedMap,0);
  flippedMap.convertTo(mapToShow, CV_32F, 1.0 / max, 0);
  namedWindow("Obstacle Map", WINDOW_NORMAL);
  imshow("Obstacle Map", mapToShow);
}

bool MobileManipMap::calculateSlopeMap() {
  Mat dx, dy, elev;
  double scale = 0.125; // 1/8 to normalize sobel filter
  double delta = 0;
  this->matElevationMap.convertTo(elev, CV_32F, 1.0 , 0);
  //getDerivKernels(dx,dy,1,1,3,true,CV_32F);
  Sobel(elev, dx, CV_32F, 1,0, 3, scale, delta, BORDER_DEFAULT );
  Sobel(elev, dy, CV_32F, 0,1, 3, scale, delta, BORDER_DEFAULT );
  Mat angle, mag;
  cartToPolar(dx, dy, mag, angle);
  //Size contSize = mag.size();
  /*if (mat.isContinous())
  {
    contSize.width *= contSize.height;
    contSize.height = 1;
  }*/

  this->slopeMap = Mat::zeros(this->matElevationMap.size(), CV_32FC1); 
  for (int j = 0; j < slopeMap.rows; j++)
  {
    for (int i = 0; i < slopeMap.cols; i++)
    {
      //cout << "Magnitude is " << endl;
      this->slopeMap.at<float>(j,i) = atan(mag.at<float>(j,i) *1.0/this->resDem) * 180.0 / 3.1416;
    }
  }
  /*Size contSize = this->slopeMap.size();
  cout << "Size is " << contSize.height << "x" << contSize.width << endl;
  Mat mapToShow;
  double min, max;
  minMaxLoc(mag, &min, &max);
  mag.convertTo(mapToShow, CV_32F, 1.0 / (max-min), 0);
  namedWindow("Magnitude Map", WINDOW_NORMAL);
  imshow("Magnitude Map", mapToShow);
  waitKey();*/
  return true;
}

bool MobileManipMap::calculateObstacleMap()
{
  Mat obstacleMap = Mat::zeros(this->matElevationMap.size(), CV_32FC1);
  threshold(this->slopeMap, obstacleMap, 20.0, 255, THRESH_BINARY_INV);

  // Borders are considered obstacles
  for (int i = 0; i < slopeMap.cols; i++)
  {
    obstacleMap.at<float>(0,i) = 0;
    obstacleMap.at<float>(this->numYnodes-1,i) = 0;
  }

  for (int j = 0; j < slopeMap.rows; j++)
  {
    obstacleMap.at<float>(j,0) = 0;
    obstacleMap.at<float>(j,this->numXnodes-1) = 0;
  }

  Mat dist;
  obstacleMap.convertTo(obstacleMap, CV_8UC1);

  /*namedWindow("Obstacle map", WINDOW_NORMAL);
  imshow("Obstacle map", obstacleMap);
  waitKey();*/
  distanceTransform(obstacleMap, dist, DIST_L2, 5);
  dist = dist*this->resDem;
  threshold(dist,dist,0.5,0,THRESH_TOZERO);
  this->obstacleMap = dist;
  //threshold(dist,dist,20.0,0,THRESH_TRUNC);
  //normalize(dist, dist, 0, 1.0, NORM_MINMAX);
  /*namedWindow("Distance Image", WINDOW_NORMAL);
  imshow("Distance Image",dist);
  waitKey();*/
  return true;
}

bool MobileManipMap::calculateCostMap()
{
  this->calculateSlopeMap();
  this->calculateObstacleMap();
  
  this->currentCostMap.clear();

  std::vector<double> row_cost;
  float current_proximity;

  for (uint j = 0; j < this->numYnodes; j++)
  {
    for (uint i = 0; i < this->numXnodes; i++)
    {
      current_proximity = this->obstacleMap.at<float>(j,i);
      if (current_proximity < 0.5) // Geometric Obstacle
      {
         row_cost.push_back(INFINITY);
      }
      else
      {
          if (current_proximity < 1.0) // Risky Distance
          {
            row_cost.push_back(1.0 + 20.0*(1.0 - current_proximity));
          }
          else
          {
            row_cost.push_back(1.0);
          }
      }
    }
    currentCostMap.push_back(row_cost);
    row_cost.clear();
  }
  std::cout << " The size of the costMap is " << currentCostMap.size() << " x " << currentCostMap[0].size() << std::endl; 
  
}

double MobileManipMap::getResolution(){
  return this->resDem;
}
