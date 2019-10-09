#include <proxy_library_sherpa_tt/ProxyLibrarySherpaTT.hpp>
#include <iostream>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
}

using namespace std;
using namespace proxy_library;


MotionCommand readMotionCommand()
{
    int manoeuvreType;
    double curvature_radm;
    double speed_ms;
    double turnRate_rads;
    cout << "\n========================" << endl;
    cout << "Enter new Motion Command" << endl;
    cout << "------------------------\n" << endl;
    cout << "manoeuvreType (0: Ackermann, 1: PointTurn): ";
    cin >> manoeuvreType;
    if(manoeuvreType == 0){ // Ackermann
        cout << "curvature_radm (Ackerman only): ";
        cin >> curvature_radm;
        cout << "speed_ms (Ackerman only): ";
        cin >> speed_ms;
    }else if(manoeuvreType == 1){ // PointTurn
        cout << "turnRate_rads (PointTurn only): ";
        cin >> turnRate_rads;
    }else{
        cerr << "Invalid input... sending stop"<<endl;
        manoeuvreType = 0;
        curvature_radm = 0;
        speed_ms = 0;
    }

    return MotionCommand(manoeuvreType, curvature_radm, speed_ms, turnRate_rads);
}

/* Joints boundaries are:
 *      - mMinJointsLimits: [ -3.31612, -2.0943, -1.9198, -3.49065, -2.2689, -3.4906 ]
 *      - mMaxJointsLimits: [ 2.9670, 1.0471, 3.001, 3.49065, 2.2689, 3.49065 ]
 * 
 * Based on sherpa_tt/orogen/sherpa_tt_mcs/scripts/manipulator/kinematics_task.yml
 */
Joints readArmCommand(){
    Joints j;
    j.m_jointNames.push_back("arm_joint_1");
    j.m_jointNames.push_back("arm_joint_2");
    j.m_jointNames.push_back("arm_joint_3");
    j.m_jointNames.push_back("arm_joint_4");
    j.m_jointNames.push_back("arm_joint_5");
    j.m_jointNames.push_back("arm_joint_6");
    j.m_jointStates.resize(6);

    cout << "\n=====================" << endl;
    cout << "Enter new Arm Command" << endl;
    cout << "---------------------\n" << endl;
    cout << "Joint 1 position (in radians): ";
    cin >> j.m_jointStates[0].m_position;

    cout << "Joint 2 position (in radians): ";
    cin >> j.m_jointStates[1].m_position;

    cout << "Joint 3 position (in radians): ";
    cin >> j.m_jointStates[2].m_position;

    cout << "Joint 4 position (in radians): ";
    cin >> j.m_jointStates[3].m_position;

    cout << "Joint 5 position (in radians): ";
    cin >> j.m_jointStates[4].m_position;

    cout << "Joint 6 position (in radians): ";
    cin >> j.m_jointStates[5].m_position;
    return j;
}

Joints buildDrivingPoseArmCommand(){
    Joints j;
    j.m_jointNames.push_back("arm_joint_1");
    j.m_jointNames.push_back("arm_joint_2");
    j.m_jointNames.push_back("arm_joint_3");
    j.m_jointNames.push_back("arm_joint_4");
    j.m_jointNames.push_back("arm_joint_5");
    j.m_jointNames.push_back("arm_joint_6");
    j.m_jointStates.resize(6);
                                              
    j.m_jointStates[0].m_position = 0;
    j.m_jointStates[1].m_position = -1.833;
    j.m_jointStates[2].m_position = 2.845;
    j.m_jointStates[3].m_position = -0.033;
    j.m_jointStates[4].m_position = -0.785;
    j.m_jointStates[5].m_position = 2.356;

    return j;
}

void printHelp(){
    cout << "\n\n============================================" << endl;
    cout << "Press 'm': to enter a new motion command" << endl;
    cout << "Press 's': to send stop command" << endl;
    cout << "Press 'a': to enter a new arm command" << endl;
    cout << "Press 'd': to enter arm driving pose command" << endl;
    cout << "--------------------------------------------" << endl;
    cout << "Precc 'Ctrl+C' to exit" <<std::endl;
    cout << "--------------------------------------------" << endl;
}

int main() {
    //Init Proxy
    Config config;
    config.printReceivedTypeInfos = false;
    config.udpConfig.max_fragment_size = 65000;        
    config.udpConfig.init_send = true;
    config.udpConfig.ip_addr =  "127.0.0.1"; //"10.250.247.216"; //"127.0.0.1"; // "192.168.1.22"
    config.udpConfig.listen_port = 20001;
    config.udpConfig.to_addrs.push_back("127.0.0.1"); //"10.250.247.2"  // "192.168.1.21"
    config.udpConfig.to_port = 20000;
    config.udpConfig.print_com_infos_ms = 0;
    
    ProxyLibrarySherpaTT proxy(config);

    Joints arm_cmd;
    MotionCommand motion_cmd = MotionCommand(0, 0, 0, 0);
    printHelp();

    // Send test data / motion and joint command.
    while(true) {
        if(kbhit()){
            char c = getchar();
            switch(c){
            case 'm':
                motion_cmd = readMotionCommand();
                printHelp();
                break;
            case 'a':
                arm_cmd = readArmCommand();
                proxy.sendManipulatorJointCommand(&arm_cmd);
                printHelp();
                break;
            case 'd':
                arm_cmd = buildDrivingPoseArmCommand();
                proxy.sendManipulatorJointCommand(&arm_cmd);
                printHelp();
                break;
            case 's':
                motion_cmd = MotionCommand(0, 0, 0, 0);
                printHelp();
                break;
            case '\n':
                break;
            default:
                cout << "Invalid char: "<<c<<endl;
                printHelp();
            }
        }

        proxy.sendMotionCommand(&motion_cmd);
        usleep(100000);
    }
    
    cin.get();
}
