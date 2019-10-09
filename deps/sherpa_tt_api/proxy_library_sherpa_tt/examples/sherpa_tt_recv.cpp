#include <proxy_library_sherpa_tt/ProxyLibrarySherpaTT.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

using namespace proxy_library;

void displayCameraImage(std::string const& cameraName, bool& isImageFetched, Image& image);

int main() {
    
    // The proxy config.
    Config config;
    config.printReceivedTypeInfos = true;
    config.udpConfig.print_com_infos_ms = 1000;
    config.udpConfig.max_fragment_size = 65000;
    config.udpConfig.init_send = true;
    config.udpConfig.ip_addr = "127.0.0.1"; //"127.0.0.1"; // "192.168.1.22"
    config.udpConfig.listen_port = 20001;
    config.udpConfig.to_addrs.push_back("127.0.0.1"); //"127.0.0.1" // "192.168.1.21"
    //config.udpConfig.to_addrs.push_back("10.250.247.2");
    config.udpConfig.to_port = 20000;
    config.udpConfig.default_max_queue_size = 1;

    // The proxy.
    ProxyLibrarySherpaTT proxy(config);


    while(true) {
        std::cout << "\n================================= " << std::endl;
        bool st;

        /*
        DEM dem;
        st = proxy.getDEM(dem);
        std::cout << "DEM: " << st << std::endl;
        if(st == 1){
            //Save DEM type
            dem.storeToFile("dem.dat");

            //Conversion to OpenCV
            cv::Mat cvm(dem.m_heightMap.m_height,dem.m_heightMap.m_width, CV_32FC1, (void *)&(dem.m_heightMap.m_data[0]));

            //Convert into 'visible' values
            cv::Mat converted(dem.m_heightMap.m_height,dem.m_heightMap.m_width, CV_8UC1);
            float scale = 50;
            float offset = 100;
            cv::convertScaleAbs(cvm, converted, scale, offset);

            //Show map
            cv::imshow("map", converted);

            //Save 'visible' map
            cv::imwrite("dem.png", converted);
            //Save floating point image (will not be shown properly in normal image viewers)
            cv::imwrite("dem.bmp", cvm);

            //Use OpenCV to access values for now:
            //Cartesian coordinates...
            float x = 2.0;
            float y = 0.0;
            //... mapped to grid coordinates
            int w = dem.m_heightMap.m_width;
            int h = dem.m_heightMap.m_height;
            int cu = w/2;
            int cv = w/2;
            int u = cu+x*(1.0/dem.m_metersPerPixelX);
            int v = cv+y*(1.0/dem.m_metersPerPixelY);

            //Acces of values in the grid
            float height1 = dem.m_heightMap.getPixel<float>(u,v);
            float h1 = dem.m_heightMap.getPixel<float>(cu,cv);
            std::cout << "Height one meter in front of the robot: " << height1 << ", and underneath the robot:" << h1 <<std::endl;
        }*/

        DGPS dgps;
        st = proxy.getDGPS(dgps);
        std::cout << "DGPS: " << st << std::endl;

        /*
        Image fimg;
        st = proxy.getFrontalCameraImage(fimg);
        if(st == 1){
            cv::Mat cvm;
            if(fimg.m_frameMode == JPEG){
               cvm = cv::imdecode(fimg.m_data, 1);
            }else{
                std::cout << "Unexpected frame mode: "<<fimg.m_frameMode<<std::endl;
                cvm = cv::Mat(fimg.m_height,fimg.m_width, CV_8UC3, (void *)&(fimg.m_data[0]));
            }
            
            //cv::Mat converted;
            //cv::cvtColor(cvm, converted, CV_BGR2RGB);

            cv::imshow("camera", cvm);
            cv::imwrite( "cam.png", cvm );
        }
        cv::waitKey(1);
        std::cout << "FrontalImage: " << st << std::endl;
        */

        Image gimg;
        st = proxy.getGripperCameraImage(gimg);
        displayCameraImage("Gripper Image", st, gimg);

        IMU imu;
        st = proxy.getIMUData(imu);
        std::cout << "IMU: " << st << std::endl;

        Joints mjoints;
        st = proxy.getManipulatorJointState(mjoints);
        std::cout << "ManipulatorJoints: " << st << std::endl;

        Joints rjoints;
        st = proxy.getMobileBaseJointState(rjoints);
        std::cout << "MobileBaseJoints: " << st << std::endl;

        Pose pose;
        st = proxy.getPose(pose);
        std::cout << "Pose: " << st << std::endl;
        std::cout << "--------------------------------- " << std::endl;

        usleep(100000);
    }
}

void displayCameraImage(std::string const& cameraName, bool& isImageFetched, Image& cameraImage){
    cv::namedWindow(cameraName, false);

    if(isImageFetched == 1){
        cv::Mat cvm;

        if(cameraImage.m_frameMode == JPEG){
            cvm = cv::imdecode(cameraImage.m_data, 1);

        }else{
            std::cout << "Unexpected frame mode: " << cameraImage.m_frameMode << std::endl;
            cvm = cv::Mat(cameraImage.m_height, cameraImage.m_width, CV_8UC3, (void *)&(cameraImage.m_data[0]));
            cv::cvtColor(cvm, cvm, cv::COLOR_RGB2BGR);
        }

        cv::imshow(cameraName, cvm);
    }
    cv::waitKey(1);

    std::cout << cameraName << ": " << isImageFetched << std::endl;
}
