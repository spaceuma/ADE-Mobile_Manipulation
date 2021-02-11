// MIT License
// -----------
// 
// Copyright (c) 2021 University of Malaga
// Permission is hereby granted, free of charge, to any person
// obtaining a copy of this software and associated documentation
// files (the "Software"), to deal in the Software without
// restriction, including without limitation the rights to use,
// copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following
// conditions:
// 
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
// OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
// HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
// WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
// OTHER DEALINGS IN THE SOFTWARE.
// 
// Authors: J. Ricardo Sánchez Ibáñez, Carlos J. Pérez del Pulgar
// Affiliation: Department of Systems Engineering and Automation
// Space Robotics Lab (www.uma.es/space-robotics)


#include "MMError.h"
#include "MMStatus.h"
#include "MobileManipExecutor.h"
#include "MobileManipMap.h"
#include "MotionCommand.h"
#include "MotionPlan.h"
#include "Pose.h"
#include "RoverGuidance_InputDataStruct.h"
#include "Waypoint.hpp"

/**
 * It is the main class that receives all the information from other components,
 * start the execution of the coupled rover-manipulator motion, etc.
 */
class MobileManipMotionPlanner
{

private:

    /**
     * Pointers to sub-components
     */
    MobileManipMap *p_mmmap; // Navigation Maps
    MotionPlan *p_motionplan; // Motion Plan
    MobileManipExecutor *p_mmexecutor; // Execution Handler

    /**
     * Software Execution Codes
     */
    MMStatus status; // Status
    MMStatus priorStatus; //Previous Status
    MMError error; // Error Type

    /**
     * Read external config and threshold values
     */
    bool readConfigFile();
    
    /**
     * Set Code
     */
    void setStatus(MMStatus status_m);
    void setError(MMError error_m);

    /**
     * Variables
     */
    base::Waypoint w_current_rover_position;  // Rover Current Position (Local)
    std::string s_configfile_path; // Path to Configuration File
    bool b_is_atomic_deployed; // Flag for Atomic Operation
    bool b_clear_underneath = false; // Sets the underneath as traversable or not

public:

    /**
     * Constructor, it receives a DEM and generates the Map object.
     */
    MobileManipMotionPlanner(const RoverGuidance_Dem &navCamDEM,
                             std::string s_urdf_path_m,
			     unsigned int ui_operation = 2);

    /**
     * IDLE Preparation Functions
     */    
    bool setArmTargetOperation(unsigned int ui_operation);
    bool updateNavCamDEM(const RoverGuidance_Dem &navCamDEM); 

    /**
     * Coupled/Atomic Motion Plan Generation.
     */
    bool generateMotionPlan(proxy_library::Pose plpose_m,
                            const proxy_library::Joints &j_present_readings,
                            double d_sample_pos_x,
                            double d_sample_pos_y);
    bool initAtomicOperation(const proxy_library::Joints &j_present_readings,
		             const base::Waypoint &w_goal,
			     double d_roll = 0.0,
			     double d_pitch = 3.1416,
			     double d_yaw = 0.0);
    bool initArmReset(const proxy_library::Joints &j_present_readings);

    /**
     * Harness Execution Control Functions.
     */
    bool start(); // Actively starts rover motion execution
    bool abort(); // Execution finishes immediately
    bool pause(proxy_library::MotionCommand &rover_command); // Goes to PAUSE
    bool resumeOperation(); // Returns from PAUSE
    void resumeError(); // Handles ERROR state
    bool ack(); // Acknowledges the operation is finished
    bool updateRoverArmPos(proxy_library::Joints &arm_command,
                           proxy_library::MotionCommand &rover_command,
                           proxy_library::Pose rover_position,
                           proxy_library::Joints arm_joints);
    unsigned int updateAtomicOperation(proxy_library::Joints &arm_command, proxy_library::Joints arm_joints,
		                       bool b_display_status = false);

    /**
     * Replanning Functions.
     */
    void updateSamplePos(proxy_library::Pose sample);
    bool updateLocCamDEM(RoverGuidance_Dem locCamDEM,
                         proxy_library::Joints &arm_command,
                         proxy_library::MotionCommand &rover_command,
                         proxy_library::Joints arm_joints);
 
    /**
     * Software Execution User Information.
     */
    MMError getErrorCode(); // Gets Error Code
    MMStatus getStatus(); // Gets the Status
    void printRoverPathInfo();
    void printExecutionInfo();
    void printStatus();
    void printErrorCode();
    bool isStatusError();

    /**
     * Variables Information.
     */
    double getCurrentRoverYaw(); // Rover Heading Angle
    std::vector<std::vector<double>> *getWristPath(); // Pointer to 3d Wrist Path
    std::vector<base::Waypoint> *getRoverPath(); // Pointer to Rover Path
    std::vector<double> *getOffset(); // Pointer to Offset
    std::vector<std::vector<double>> *getCostMap(); //Pointer to Current CostMap
    bool getMorphMaps(
        std::vector<std::vector<double>> &vvd_elevation_map_m,
        std::vector<std::vector<double>> &vvd_slope_map_m,
        std::vector<std::vector<double>> &vvd_sd_map_m,
        std::vector<std::vector<int8_t>> &vvi_validity_map_m);
    bool getNavigationMaps(
        std::vector<std::vector<int>> &vvi_traversability_map_m,
        std::vector<std::vector<double>> &vvd_cost_map_m);

    /**
     * A pointer to the arm motion profile is returned
     */
    std::vector<std::vector<double>> *getArmMotionProfile();

    /**
     * A pointer to the 3d cost map is returned
     */
    std::vector<std::vector<std::vector<double>>> *get3DCostMap();

};
