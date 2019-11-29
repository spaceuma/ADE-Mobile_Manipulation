#include <gtest/gtest.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "MobileManipMap.h"
#include "MM_status.h"
#include "readMatrixFile.h"

using namespace cv;

TEST(MMMapTest, constructorTest){
  /*double dummyHeightData [9] = {0.0, 1.0, 2.0, 0.0, 1.0, 2.0, 0.0, 1.0, 2.0};
  RoverGuidance_Dem dummyDem;
  dummyDem.cols = 10;
  dummyDem.rows = 10;
  dummyDem.p_heightData_m = &dummyHeightData[0];*/
  double res = 0.04; //meters
  double n_row, n_col;
  std::vector<double> vector_elevationData;

  RoverGuidance_Dem* dummyDem = new RoverGuidance_Dem;
  readMatrixFile("test/unit/data/ColmenarRocks_smaller_4cmDEM.csv", res, n_row, n_col, vector_elevationData);
  std::cout << "The size is " << vector_elevationData.size() << std::endl;
  double dummyArray[(int)n_row*(int)n_col];
  dummyDem->p_heightData_m = dummyArray;
  for (uint i = 0; i<vector_elevationData.size(); i++)
  {
    dummyDem->p_heightData_m[i] = vector_elevationData[i];
  }
  
  dummyDem->cols = n_col;
  dummyDem->rows = n_row;
  dummyDem->nodeSize_m = res;
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


