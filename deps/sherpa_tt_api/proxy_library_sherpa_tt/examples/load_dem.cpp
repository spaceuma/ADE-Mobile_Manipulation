#include <proxy_library_sherpa_tt/ProxyLibrarySherpaTT.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace proxy_library;
int main(int argc, char** argv){
    // DEM is not supported in SherpaTT ADE.
    /*
    DEM dem;
    bool st = dem.loadFromFile("dem.dat");
    if(!st){
        std::clog << "Error loading DEM" <<std::endl;
        return -1;
    }else {
        std::cout << "Loaded Success: "<<st<<std::endl;
    }

    //Print DEM
    std::cout << dem.toString() << std::endl;

    //The data structure dem contains all the data.. the following code is
    //just to display the DEM (to validate that loading was successful)

    //Conversion to OpenCV
    cv::namedWindow("map", false);
    std::cout << "Converting to OpenCV" <<std::endl;
    cv::Mat cvm(dem.m_heightMap.m_height,dem.m_heightMap.m_width, CV_32FC1, (void *)&(dem.m_heightMap.m_data[0]));

    //Convert into 'visible' values
    cv::Mat converted(dem.m_heightMap.m_height,dem.m_heightMap.m_width, CV_8UC1);
    float scale = 50;
    float offset = 100;
    std::cout << "Scaling..."<<std::endl;
    cv::convertScaleAbs(cvm, converted, scale, offset);

    //Get Height via API
    size_t u,v;
    u=100;
    v=150;
    float height1 = dem.m_heightMap.getPixel<float>(u,v)*dem.m_metersPerIntensity;
    std::cout << "height api: " << height1 << std::endl;

    //Get Height via OpenCV
    float height2 = cvm.at<float>(v, u)*dem.m_metersPerIntensity;
    std::cout << "height cv: " << height2 << std::endl;

    //Show map
    std::cout << "Displaying.. press enter to exit"<<std::endl;
    cv::imshow("map", converted);
    cv::waitKey(0);
    return 0;
    */
}
