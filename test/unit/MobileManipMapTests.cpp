#include <gtest/gtest.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "MobileManipMap.h"
#include "MM_status.h"

using namespace cv;

TEST(MMMapTest, constructorTest){
  double dummyHeightData [9] = {0.0, 1.0, 2.0, 0.0, 1.0, 2.0, 0.0, 1.0, 2.0};
  RoverGuidance_Dem dummyDem;
  dummyDem.cols = 10;
  dummyDem.rows = 10;
  dummyDem.p_heightData_m = &dummyHeightData[0];

  Mat decosDEM;
  decosDEM = imread(samples::findFile( "/home/ares/ADE_Mobile-manipulation/test/unit/data/decosHeightField.tif" ), IMREAD_GRAYSCALE);

  MobileManipMap dummyMap;
  //ASSERT_NO_THROW(dummyMap.setRGDem(dummyDem));
  ASSERT_NO_THROW(dummyMap.setImageDem(decosDEM));
  ASSERT_NO_THROW(dummyMap.showSlopeMap());

  //namedWindow("DECOS Map", WINDOW_AUTOSIZE);
  //imshow("DECOS Map", decosDEM);
  waitKey();
  //destroyWindow("DECOS Map");
}


