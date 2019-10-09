#include <proxy_library_sherpa_tt/ProxyLibrarySherpaTT.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int main() {
    using namespace proxy_library;
    
    Config config;
    config.printReceivedTypeInfos = false;
    config.udpConfig.max_fragment_size = 65000;
    config.udpConfig.init_send = true;
    config.udpConfig.ip_addr = "127.0.0.1"; //"127.0.0.1"; // "192.168.1.22"
    config.udpConfig.listen_port = 20001;
    config.udpConfig.to_addrs.push_back("127.0.0.1"); //"127.0.0.1" // "192.168.1.21"
    //config.udpConfig.to_addrs.push_back("10.250.247.2");
    config.udpConfig.to_port = 20000;
    config.udpConfig.print_com_infos_ms = 0;
    config.udpConfig.default_max_queue_size = 1;

    ProxyLibrarySherpaTT proxy(config);

    int countCycles=0;
    //int demReceived=0;
    int poseReceived=0;
    while(true) {
        bool st;
        countCycles++;

        /*
        DEM dem;
        st = proxy.getDEM(dem);
        if(st == 1){
            demReceived++;
            std::cout << "demReceived:" << demReceived << "\tratiodemReceived:" <<
                         ((float)demReceived)/((float)countCycles) << std::endl;
        }*/


        Pose pose;
        st = proxy.getPose(pose);
        if(st==1){
            poseReceived++;
            std::cout << "poseReceived:" << poseReceived << "\tratioposeReceived:" <<
                         ((float)poseReceived)/((float)countCycles) << std::endl;
        }
        std::cout << std::endl;

        usleep(100000);
    }
}
