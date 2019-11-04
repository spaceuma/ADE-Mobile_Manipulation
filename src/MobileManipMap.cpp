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

int MobileManipMap::setRGDem(RoverGuidance_Dem dem){
  // TODO - implement MobileManipMap::setDem
  cout << "" << endl;
  this->rgDem = dem;
  this->offsetXYZ[0] = dem.mapOrigin_m_Mlg[0];
  this->offsetXYZ[1] = dem.mapOrigin_m_Mlg[1];
  this->offsetXYZ[2] = dem.mapOrigin_m_Mlg[2];
  this->numXnodes = dem.cols;
  this->numYnodes = dem.rows;
  this->resDem = dem.nodeSize_m;
  if (this->calculateElevationMap())
  {
    return 0;
  }
  else
  {
    return 1;
  }
}

int MobileManipMap::setImageDem(Mat inputDem)
{
  flip(inputDem, this->elevationMap, 0); //0 = flip upside down
  if (this->calculateCostMap())
  {
    return 0;
  }
  else
  {
    return 1;
  }
}

bool MobileManipMap::calculateElevationMap() {
  // TODO - implement MobileManipMap::calculateElevationMap
  this->elevationMap = Mat::zeros(cv::Size(numYnodes,numXnodes), CV_64F);
  this->elevationMap.resize(numYnodes);
  for (int j = 0; j < this->numYnodes; j++)
  {
    for (int i = 0; i < this->numXnodes; i++)
    {
      try
      {
        elevationMap.at<double>(j,i) = this->rgDem.p_heightData_m[i + j*i];
      }
      catch(...)
      {
        cout << "MMMap: Exception occurred reading column " << i << ", row " << j << endl;
        cout << "MMMap: input dem size shall be " << this->numXnodes << "x" << this->numYnodes << endl;
        throw;
      }
    }
  }
  return true;
}

void MobileManipMap::showElevationMap()
{
  Mat mapToShow;
  flip(this->elevationMap, mapToShow,0);
  imshow("Elevation Map", mapToShow);
}

void MobileManipMap::showSlopeMap()
{
  Mat flippedMap, mapToShow;
  flip(this->slopeMap, flippedMap,0);
  flippedMap.convertTo(mapToShow, CV_32F, 1.0 / 255, 0);
  namedWindow("Slope Map", WINDOW_NORMAL);
  imshow("Slope Map", flippedMap);

  /*FileStorage fs("slopeMap.yml", FileStorage::WRITE);
  fs << "SlopeMap" << this->slopeMap;
  fs.release();*/
}

bool MobileManipMap::calculateSlopeMap() {
  Mat dx, dy;
  float scale = 0.125; // 1/8 to normalize sobel filter
  float delta = 0;
  //getDerivKernels(dx,dy,1,1,3,true,CV_32F);
  Sobel(this->elevationMap, dx, CV_32F, 1,0, 3, scale, delta, BORDER_DEFAULT );
  Sobel(this->elevationMap, dy, CV_32F, 0,1, 3, scale, delta, BORDER_DEFAULT );
  Mat angle, mag;
  cartToPolar(dx, dy, mag, angle);
  //Size contSize = mag.size();
  /*if (mat.isContinous())
  {
    contSize.width *= contSize.height;
    contSize.height = 1;
  }*/

  this->slopeMap = Mat::zeros(this->elevationMap.size(), CV_32FC1); 
  for (int j = 0; j < slopeMap.rows; ++j)
  {
    for (int i = 0; i < slopeMap.cols; ++i)
    {
      //cout << "Magnitude is " << endl;
      this->slopeMap.at<float>(j,i) = atan(mag.at<float>(j,i) *0.25) * 180.0 / 3.1416;
    }
  }
  Size contSize = this->slopeMap.size();
  cout << "Size is " << contSize.height << "x" << contSize.width << endl;
  Mat mapToShow;
  mag.convertTo(mapToShow, CV_32F, 1.0 / 255, 0);
  namedWindow("Magnitude Map", WINDOW_NORMAL);
  imshow("Magnitude Map", mapToShow);
  waitKey();
}

bool MobileManipMap::calculateObstacleMap()
{
  Mat obstacleMap = Mat::zeros(this->elevationMap.size(), CV_32FC1);
  threshold(this->slopeMap, obstacleMap, 15.0, 255, THRESH_BINARY_INV);

  Mat dist;
  obstacleMap.convertTo(obstacleMap, CV_8UC1);

  namedWindow("Obstacle map", WINDOW_NORMAL);
  imshow("Obstacle map", obstacleMap);
  waitKey();
  distanceTransform(obstacleMap, dist, DIST_L2, 3);
  threshold(dist,dist,5.0,0,THRESH_TOZERO);
  threshold(dist,dist,20.0,0,THRESH_TRUNC);
  normalize(dist, dist, 0, 1.0, NORM_MINMAX);
  namedWindow("Distance Image", WINDOW_NORMAL);
  imshow("Distance Image",dist);
  waitKey();
}

bool MobileManipMap::calculateCostMap()
{
  this->calculateSlopeMap();
  this->calculateObstacleMap();
  
}

void MobileManipMap::checkObstacles(RoverGuidance_Dem locCamDEM, MotionPlan motionPlan) {
	// TODO - implement MobileManipMap::checkObstacles
	throw "Not yet implemented";
}
