#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

int main()
{
    Mat image = imread("FusedDEM.pgm", IMREAD_UNCHANGED);
    double minv, maxv;
    minMaxLoc(image, &minv, &maxv);
    std::cout << "The min is " << minv << " and the max is " << maxv << std::endl;
    image.convertTo(image, CV_8U);
    namedWindow("Image window", WINDOW_AUTOSIZE);
    imshow("Image window", image);
    waitKey(0);
}
