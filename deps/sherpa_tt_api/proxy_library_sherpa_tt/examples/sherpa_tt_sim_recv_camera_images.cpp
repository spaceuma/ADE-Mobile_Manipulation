#include <proxy_library_sherpa_tt/ProxyLibrarySherpaTTSim.hpp>

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
    config.udpConfig.ip_addr = "127.0.0.1";
    config.udpConfig.listen_port = 20011;
    config.udpConfig.to_addrs.push_back("127.0.0.1");
    config.udpConfig.to_port = 20010;
    config.udpConfig.default_max_queue_size = 1;

    // The proxy.
    ProxyLibrarySherpaTTSim proxy(config);

    // The images to fetch.
    Image ptuCameraLeftImage;
    Image ptuCameraRightImage;
    Image fixedCameraLeftImage;
    Image fixedCameraRightImage;

    // Fetch status indicator.
    bool isImageFetched;
    
    while(true) {
        std::cout << "\n================================= " << std::endl;
        
        // PTU Camera - Left   
        isImageFetched = proxy.getSimPTUCameraImageLeft(ptuCameraLeftImage);
        displayCameraImage("PTU Camera - Left" , isImageFetched, ptuCameraLeftImage);

        // PTU Camera - Left 
        isImageFetched = proxy.getSimPTUCameraImageRight(ptuCameraRightImage);
        displayCameraImage("PTU Camera - Right" , isImageFetched, ptuCameraRightImage);

        // Fixed Camera - Left   
        isImageFetched = proxy.getSimFixCameraImageLeft(fixedCameraLeftImage);
        displayCameraImage("Fixed Camera - Left" , isImageFetched, fixedCameraLeftImage);

        // Fixed Camera - Right 
        isImageFetched = proxy.getSimFixCameraImageRight(fixedCameraRightImage);
        displayCameraImage("Fixed Camera - Right" , isImageFetched, fixedCameraRightImage);

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
