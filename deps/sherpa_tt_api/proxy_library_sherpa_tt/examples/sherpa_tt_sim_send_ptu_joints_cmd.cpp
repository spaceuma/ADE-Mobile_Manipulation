#include <proxy_library_sherpa_tt/ProxyLibrarySherpaTTSim.hpp>

#include <iostream>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstdlib> 
#include <regex>

using namespace std;
using namespace proxy_library;

float const MAX_PAN_POSITION = 3.14; // radians.
float const MAX_TILT_POSITION = 1.1; // radians.
float const MIN_SPEED = 0.0; // radians per second. 
float const MAX_SPEED = 1.0; // radians per second.

// Use regex to check if an input is a float.
// Surely there is a better way, like the c++ equivalent of typeof()?
regex const regex_float("-?[0-9]+.?[0-9]*");

void readPTUJointsCommand(Joints& ptuJointsCommand){
    cout << "\n=====================" << endl;
    cout << "Enter new PTU Command" << endl;
    cout << "---------------------\n" << endl;

    // Default target values are set too large in order to force cin loops
    // until the inputted values are withing range.
    float target_pan_position = 100;
    float target_pan_speed = 100;
    float target_tilt_position = 100;
    float target_tilt_speed = 100;

    // This variable is used to check, via regex, if the input is a float.
    string input;

    do{
        cout << "Pan motor joint position (between " << -MAX_PAN_POSITION << " and " << MAX_PAN_POSITION << " rads): ";
        cin >> input;

        // Check if input is a float before assigning it to the float variable.
        if(regex_match(input, regex_float)){
            target_pan_position = stof(input);
        }
    
    }while(!regex_match(input, regex_float) || abs(target_pan_position) > MAX_PAN_POSITION);

    
    do{
        cout << "Pan motor joint speed (between " << MIN_SPEED << " and " << MAX_SPEED << " rads/s): ";
        cin >> input;

        // Check if input is a float before assigning it to the float variable.
        if(regex_match(input, regex_float)){
            target_pan_speed = stof(input);
        }

    }while(!regex_match(input, regex_float) || (target_pan_speed > MAX_SPEED || target_pan_speed < MIN_SPEED));

    
    do{
        cout << "Tilt motor joint position (between " << -MAX_TILT_POSITION << " and " << MAX_TILT_POSITION << " rads): ";
        cin >> input;

        // Check if input is a float before assigning it to the float variable.
        if(regex_match(input, regex_float)){
            target_tilt_position = stof(input);
        }

    }while(!regex_match(input, regex_float) || abs(target_tilt_position) > MAX_TILT_POSITION);

    do{
        cout << "Tilt motor joint speed (between " << MIN_SPEED << " and " << MAX_SPEED << " rads/s): ";
        cin >> input;

        // Check if input is a float before assigning it to the float variable.
        if(regex_match(input, regex_float)){
            target_tilt_speed = stof(input);
        }

    }while(!regex_match(input, regex_float) || (target_tilt_speed > MAX_SPEED || target_tilt_speed < MIN_SPEED));

    // Set target position and position for pan motor joint.
    ptuJointsCommand.m_jointStates[0].m_position = target_pan_position;
    ptuJointsCommand.m_jointStates[0].m_speed = target_pan_speed;

    // Set target position and position for tilt motor joint.
    ptuJointsCommand.m_jointStates[1].m_position = target_tilt_position;
    ptuJointsCommand.m_jointStates[1].m_speed = target_tilt_speed;

}


int main() {
    // The proxy config.
    Config config;
    config.printReceivedTypeInfos = false;
    config.udpConfig.print_com_infos_ms = 0;
    config.udpConfig.max_fragment_size = 65000;        
    config.udpConfig.init_send = true;
    config.udpConfig.ip_addr =  "127.0.0.1";
    config.udpConfig.listen_port = 20011;
    config.udpConfig.to_addrs.push_back("127.0.0.1");
    config.udpConfig.to_port = 20010;
    
    // The proxy.
    ProxyLibrarySherpaTTSim proxy(config);

    // Create the Joints object that will contain the target joint states.
    Joints ptuJointsCommand;
    ptuJointsCommand.m_jointNames.push_back("mastcam_ptu_pan_motor");
    ptuJointsCommand.m_jointNames.push_back("mastcam_ptu_tilt_motor");
    
    // Insert blank joint states for pan and tilt motor joints.
    JointState jsPan;
    ptuJointsCommand.m_jointStates.push_back(jsPan);

    JointState jsTilt;
    ptuJointsCommand.m_jointStates.push_back(jsTilt);

    // Set target motor joints positions and speeds based on user input.
    // Send command after user input completion.
    while(true) {
        readPTUJointsCommand(ptuJointsCommand);
        proxy.sendSimPTUJointsCommand(&ptuJointsCommand);
        usleep(100000);
    }
    
    cin.get();
}
