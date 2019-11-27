#include <gtest/gtest.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "MobileManipMap.h"
#include "MM_status.h"
#include <fstream>

using namespace cv;

void readMatrixFile(std::string map_file, double res, RoverGuidance_Dem* rgDem)
{
    std::cout << "Reading map " << map_file << std::endl;
    std::string line;
    std::ifstream e_file(map_file.c_str(), std::ios::in);
    double n_row = 0, n_col = 0;
    std::vector<double> vector_elevationData;

    if (e_file.is_open())
    {
        while (std::getline(e_file, line))
        {
            std::stringstream ss(line);
            std::string cell;
            while (std::getline(ss, cell, ' '))
            {
                double val;
                std::stringstream numeric_value(cell);
                numeric_value >> val;
                vector_elevationData.push_back(val);
                n_col++;
            }
            n_row++;
        }
        e_file.close();

        n_col /= n_row;
        std::cout << "Map of " << n_col << " x " << n_row << " loaded." << std::endl;
    }
    else
    {
        std::cout << "Problem opening the map file" << std::endl;
    }

    //double elevationData[rgDem->v_elevationData_m.size()];

    //double elevationData[vector_elevationData.size()];
    for (uint i = 0; i<vector_elevationData.size(); i++)
    {
        rgDem->p_heightData_m[i] = vector_elevationData[i];
    }
    rgDem->cols = n_col;
    rgDem->rows = n_row;
    rgDem->nodeSize_m = res;
    //rgDem->p_heightData_m = elevationData;
    /*for (uint j = 0; j<rgDem->rows; j++)
    {
        for (uint i = 0; i<rgDem->cols; i++)
        {
            std::cout << "Initial Value is " << rgDem->p_heightData_m[i+j*rgDem->cols] << std::endl;
        }
    }*/
}

TEST(MMMapTest, constructorTest){
  /*double dummyHeightData [9] = {0.0, 1.0, 2.0, 0.0, 1.0, 2.0, 0.0, 1.0, 2.0};
  RoverGuidance_Dem dummyDem;
  dummyDem.cols = 10;
  dummyDem.rows = 10;
  dummyDem.p_heightData_m = &dummyHeightData[0];*/
  double res = 0.04; //meters
  RoverGuidance_Dem* dummyDem = new RoverGuidance_Dem;
  double dummyArray[40000];
  dummyDem->p_heightData_m = dummyArray;
  readMatrixFile("test/unit/data/ColmenarRocks_smaller_4cmDEM.csv", res, dummyDem);
  /*for (uint j = 0; j<dummyDem->rows; j++)
  {
        for (uint i = 0; i<dummyDem->cols; i++)
        {
            std::cout << "Initial Value is " << dummyDem->p_heightData_m[i+j*dummyDem->cols] << std::endl;
        }
  }*/
  MobileManipMap dummyMap;
  dummyMap.setRGDem((* dummyDem));
  dummyMap.showElevationMap();
  waitKey();
  dummyMap.showSlopeMap();
  waitKey();
  dummyMap.showObstacleMap();
  waitKey();
  //ASSERT_NO_THROW(dummyMap.setImageDem(decosDEM,0.028));
  //ASSERT_NO_THROW(dummyMap.showSlopeMap());

  //namedWindow("DECOS Map", WINDOW_AUTOSIZE);
  //imshow("DECOS Map", decosDEM);
  //waitKey();
  //destroyWindow("DECOS Map");
}


