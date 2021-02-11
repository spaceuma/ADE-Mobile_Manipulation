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


#ifndef __COLLISION_DETECTOR__
#define __COLLISION_DETECTOR__

#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>
#include <iostream>

class CollisionDetector
{
private:
    dart::utils::DartLoader dl;
    // Create the world
    dart::dynamics::SkeletonPtr sherpatt;
    dart::dynamics::SkeletonPtr manipulator;
    dart::dynamics::SkeletonPtr manipulator_wrist;

    bool includeWrist;

public:
    dart::simulation::WorldPtr mWorld;
    CollisionDetector(std::string s_urdf_path, bool _includeWrist = true);

    ~CollisionDetector();

    void initializeWristModel();
    bool isColliding(const std::vector<double> manip_joints);
    bool isWristColliding(const std::vector<double> manip_joints);
};

#endif
