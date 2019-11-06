#include <gtest/gtest.h>
#include "FastMarching.h"
#include "Waypoint.hpp"
#include "WaypointNavigation.hpp"
#include <math.h>

using namespace FastMarching_lib;

TEST(FastMarchingTests, bugcontrol){
	double a = INFINITY;
	double b = INFINITY;
	double c = 1;

	BiFastMarching dummyFM;
  	EXPECT_EQ(INFINITY,dummyFM.getEikonal(a,b,c));

}
