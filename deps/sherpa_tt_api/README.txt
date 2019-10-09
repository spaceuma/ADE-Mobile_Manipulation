Included files
--------------
* README.txt
* install.sh
* Folders with libraries:
   - base-logging
   - base-cmake
   - cmake
   - detached_logger
   - ip_tunnel
   - proxy_library
   - proxy_library_sherpa_tt <-- this is the interesting one :)



Required Preconditions
----------------------
* Linux PC running Ubuntu 18.04 (Bionic Beaver), 64 Bit. Others might also work,
  but this is not tested.
* For the examples in proxy_library_sherpa_tt, OpenCV will be needed.



Installation
------------
1. Copy the included files somewhere. For example ~/sherpa_api

2. Make the script file install.sh executable

     cd ~/sherpa_api
     chmod u+x install.sh

3. Run the installation process

     cd ~/sherpa_api
     ./install.sh

   The installation will take ~2 minutes.
   
4. After the installation, source the generated env.sh file

    source env.sh
   
   

Testing
-------
* Run the simulator (different software package)
   - Make sure you start the "Simulation for High Level Control"
   - Make sure the "API Proxy" is started as well
   
* Start the program sherpa_tt_recv
   - you should see an output like the following scrolling down the terminal:
================================
DEM: 0
DGPS: 0
FrontalImage: 1
GripperImage: 0
IMU: 1
ManipulatorJoints: 1
MobileBaseJoints: 1
Pose: 1

================================
   - When you can see values '1', it means that data was received and the 
     connection is working.
   - You should also see two windows appear displaying camera images and the map
   
* Have a look at the source code in proxy_library_sherpa_tt/examples/
  It illustrates how to establish a connection with the Simulator (will be the
  same for the real robot) and how to access data from the robot or send
  commands.
  

Configuration for use with Simulator
------------------------------------

To control the robot with the remote API start, first your network setup must be
configure on the involved systems

* On Simulator side open the file 
  bundles/sherpa_tt/config/orogen/sherpa_tt_proxy::Task.yml and
    - adjust the properties 'ip_addr' (IP of the System running the Simulator)
    - and 'to_addrs' (IP of the system consuming the SherpaAPI).

* On the Sherpa Remote API side adjust in your code the configuration of the 
  ProxyLibrarySherpaTT according to:
    - config.udpConfig.ip_addr = "127.0.0.1"; <-- IP of PC running Remote API
    - config.udpConfig.to_addrs.push_back("127.0.0.1"); <-- IP of Simulator PC

* Start the following Parts of the Simualtor:
    - "Simulation for High Level Control"
    - "API Proxy"
    

Troubleshooting
---------------

Q: "how is the Map to be interpreted?"
A: The rover always sits in the middle of the map.
   The Rover's x-axis points forwards in driving direction, z points upwards.
   The Map is aligned accordingly: x (sometimes called u) is aligned with the
   Rovers x-axis.
   
Q: "My Robot does not moven even if I give driving commands from the 
   Control GUI"
A: Make sure to select an Arm posture and click the play button for the Arm
   posture. Sending at least one command to the arm is required to initiate the
   data flow in the control system.

