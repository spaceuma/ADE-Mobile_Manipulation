#include <proxy_library_sherpa_tt/ProxyLibrarySherpaTT.hpp>
#include <base/Time.hpp>
#include <boost/circular_buffer.hpp>
#include <iostream>

class Measure{
public:
    enum State{
        INITIALIZING,
        BUFFERING,
        RUNNING
    };
    struct Stat{
        State state;
        double mean_hz;
        double mean_sec;
        double buffer_size;
        double buffer_capacity;
    };

    Measure(){
        int buffer_capacity = 50;
        buffer = boost::circular_buffer<base::Time>(buffer_capacity);
        initialized = false;
        stat.state = INITIALIZING;
        stat.buffer_capacity = buffer_capacity;
        stat.buffer_size = 0;
    }
    base::Time t_prev;
    base::Time t_cur;
    bool initialized;
    boost::circular_buffer<base::Time> buffer;
    Stat stat;

    void update(base::Time t){
        if(!initialized){
            t_prev = t;
            initialized = true;
            stat.state = BUFFERING;
            return;
        }
        t_cur = t;
        base::Time dur = t_cur - t_prev;
        buffer.push_back(dur);
        stat.buffer_size = buffer.size();
        if(buffer.full()){
            stat.state = RUNNING;
        }
        t_prev = t_cur;
    }

    Stat eval(){
        int n_samples = 0;
        double accumulated = 0;
        for(base::Time& dur : buffer ){
            accumulated += dur.toSeconds();
            n_samples++;
        }

        stat.mean_sec = accumulated / n_samples;
        stat.mean_hz = 1.0/stat.mean_sec;

        return stat;
    }
};

std::ostream& operator<<(std::ostream& os, const Measure::Stat& stat)
{
    std::string state_str;
    switch (stat.state) {
    case Measure::INITIALIZING:
        state_str = "INITIALIZING";
        break;
    case Measure::BUFFERING:
        state_str = "BUFFERING";
        break;
    case Measure::RUNNING:
        state_str = "RUNNING";
        break;
    default:
        state_str = "UNEXPECTED: "+std::to_string(stat.state);
        break;
    }
    os << "STATE: "<< state_str << ", Buffer Size:"<<stat.buffer_size<< ", Mean Sec: " << stat.mean_sec << ", Mean Hz: " << stat.mean_hz;
    return os;
}

int main() {
    using namespace proxy_library;
    double sample_rate = 1000;

    Config config;
    config.printReceivedTypeInfos = false;
    config.udpConfig.max_fragment_size = 65000;
    config.udpConfig.init_send = true;
    config.udpConfig.ip_addr = "127.0.0.1"; //"127.0.0.1"; // "192.168.1.22"
    config.udpConfig.listen_port = 20001;
    config.udpConfig.to_addrs.push_back("127.0.0.1"); //"127.0.0.1" // "192.168.1.21"
    //config.udpConfig.to_addrs.push_back("10.250.3.54");
    config.udpConfig.to_port = 20000;
    config.udpConfig.print_com_infos_ms = 0;
    config.udpConfig.default_max_queue_size = 1;

    ProxyLibrarySherpaTT proxy(config);
    Measure measure_dem;
    Measure measure_dgps;
    //Measure measure_fimg;
    Measure measure_gimg;
    Measure measure_imu;
    Measure measure_mjoints;
    Measure measure_rjoints;
    Measure measure_pose;

    double print_period = 1;
    base::Time last_printed;

    while(true) {
        bool st;

        /*
        DEM dem;
        st = proxy.getDEM(dem);
        if(st){
            measure_dem.update(base::Time::now());
            //std::cout << "Updated DEM" << std::endl;
        }*/

        DGPS dgps;
        st = proxy.getDGPS(dgps);
        if(st){
            measure_dgps.update(base::Time::now());
            //std::cout << "Updated DGPS" << std::endl;
        }

        /*
        Image fimg;
        st = proxy.getFrontalCameraImage(fimg);
        if(st){
            measure_fimg.update(base::Time::now());
            //std::cout << "Updated FIMG" << std::endl;
        }*/

        Image gimg;
        st = proxy.getGripperCameraImage(gimg);
        if(st){
            measure_gimg.update(base::Time::now());
            //std::cout << "Updated GIMG" << std::endl;
        }

        IMU imu;
        st = proxy.getIMUData(imu);
        if(st){
            measure_imu.update(base::Time::now());
            //std::cout << "Updated IMU" << std::endl;
        }

        Joints mjoints;
        st = proxy.getManipulatorJointState(mjoints);
        if(st){
            measure_mjoints.update(base::Time::now());
            //std::cout << "Updated MJOINTS" << std::endl;
        }

        Joints rjoints;
        st = proxy.getMobileBaseJointState(rjoints);
        if(st){
            measure_rjoints.update(base::Time::now());
            //std::cout << "Updated RJOINTS" << std::endl;
        }

        Pose pose;
        st = proxy.getPose(pose);
        if(st){
            measure_pose.update(base::Time::now());
            //std::cout << "Updated POSE" << std::endl;
        }

        // Print Stats
        if((base::Time::now()-last_printed).toSeconds() > print_period){
            last_printed = base::Time::now();
            std::cout << "\n=======================================\n";
            //std::cout << "DEM: "<<measure_dem.eval()<<"\n";
            std::cout << "DGPS: "<<measure_dgps.eval()<<"\n";
            //std::cout << "FIMG: "<<measure_fimg.eval()<<"\n";
            std::cout << "GIMG: "<<measure_gimg.eval()<<"\n";
            std::cout << "IMU: "<<measure_imu.eval()<<"\n";
            std::cout << "MJOINTS: "<<measure_mjoints.eval()<<"\n";
            std::cout << "RJOINTS: "<<measure_rjoints.eval()<<"\n";
            std::cout << "POSE: "<<measure_pose.eval()<<"\n";
            std::cout << "--------------------------------------"<<std::endl;
        }

        usleep(base::Time::fromSeconds(1.0/sample_rate).toMicroseconds());
    }
}
