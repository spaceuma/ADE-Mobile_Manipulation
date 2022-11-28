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

#include "CollisionDetector.h"

CollisionDetector::CollisionDetector(std::string s_urdf_path_m, bool _includeWrist)
{
    includeWrist = _includeWrist;

    mWorld = dart::simulation::World::create("Empty");
    dl.addPackageDirectory("urdf", s_urdf_path_m);    // TODO set this path
    // Load urdf models
    // sherpatt = dl.parseSkeleton("package://urdf/sherpa_tt.urdf");
    sherpatt = dl.parseSkeleton("package://urdf/sherpa_tt.urdf");
    manipulator = dl.parseSkeleton("package://urdf/manipulator.urdf");
    sherpatt->setName("sherpatt");
    manipulator->setName("manipulator");
    mWorld->addSkeleton(sherpatt);
    mWorld->addSkeleton(manipulator);

    if(includeWrist)
    {
        initializeWristModel();
    }

    /* First joint */
    sherpatt->getDof("beta_front_left")->setPosition(-0.46);
    sherpatt->getDof("beta1_fake_front_left")->setPosition(0.0 * M_PI / 180.0);
    sherpatt->getDof("beta2_fake_front_left")->setPosition(-0.46);

    sherpatt->getDof("beta_front_right")->setPosition(-0.46);
    sherpatt->getDof("beta1_fake_front_right")->setPosition(0.0 * M_PI / 180.0);
    sherpatt->getDof("beta2_fake_front_right")->setPosition(-0.46);

    sherpatt->getDof("beta_rear_left")->setPosition(-0.46);
    sherpatt->getDof("beta1_fake_rear_left")->setPosition(0.0 * M_PI / 180.0);
    sherpatt->getDof("beta2_fake_rear_left")->setPosition(-0.46);

    sherpatt->getDof("beta_rear_right")->setPosition(-0.46);
    sherpatt->getDof("beta1_fake_rear_right")->setPosition(0.0 * M_PI / 180.0);
    sherpatt->getDof("beta2_fake_rear_right")->setPosition(-0.46);

    /* Second joint */
    sherpatt->getDof("gamma_front_left")->setPosition(0.61);
    // sherpatt->getDof("gamma1_fake_front_left")->setPosition(0.61);
    sherpatt->getDof("gamma2_fake_front_left")->setPosition(0.61);

    sherpatt->getDof("gamma_front_right")->setPosition(0.61);
    // sherpatt->getDof("gamma1_fake_front_right")->setPosition(0.61);
    sherpatt->getDof("gamma2_fake_front_right")->setPosition(0.61);

    sherpatt->getDof("gamma_rear_left")->setPosition(0.61);
    // sherpatt->getDof("gamma1_fake_rear_left")->setPosition(0.61);
    sherpatt->getDof("gamma2_fake_rear_left")->setPosition(0.61);

    sherpatt->getDof("gamma_rear_right")->setPosition(0.61);
    // sherpatt->getDof("gamma1_fake_rear_right")->setPosition(0.61);
    sherpatt->getDof("gamma2_fake_rear_right")->setPosition(0.61);
}

CollisionDetector::~CollisionDetector() {}

void CollisionDetector::initializeWristModel()
{
    manipulator_wrist = dl.parseSkeleton("package://urdf/manipulator_wrist.urdf");
    manipulator_wrist->setName("manipulator_wrist");
    mWorld->addSkeleton(manipulator_wrist);
}

bool CollisionDetector::isColliding(const std::vector<double> manip_joints)
{
    // Get sherpa into the desired configuration
    /* Pan of the whole leg */
    // sherpatt->getDof("alpha_front_left")->setPosition(0.0 * M_PI / 180.0);

    // Get the manipulator into the desired configuration
    manipulator->getDof("arm_joint_1")->setPosition(manip_joints[0]);
    manipulator->getDof("arm_joint_2")->setPosition(manip_joints[1]);
    manipulator->getDof("arm_joint_3")->setPosition(manip_joints[2]);
    manipulator->getDof("arm_joint_4")->setPosition(manip_joints[3]);
    manipulator->getDof("arm_joint_5")->setPosition(manip_joints[4]);
    manipulator->getDof("arm_joint_6")->setPosition(manip_joints[5]);

    // Look through the collisions manipulator-sherpaTT and manipulator with
    // itself
    auto collisionEngine = mWorld->getConstraintSolver()->getCollisionDetector();
    auto sherpaGroup = collisionEngine->createCollisionGroup(sherpatt.get());
    auto manipGroup = collisionEngine->createCollisionGroup(manipulator.get());

    dart::collision::CollisionOption option;
    dart::collision::CollisionResult result;
    bool collisionBody = sherpaGroup->collide(manipGroup.get(), option, &result);
    bool collisionManip = manipGroup->collide(option, &result);

    /*if(collisionBody)
    {
      std::cout << "The manipulator is in collision with the body" << std::endl;
    }
    if(collisionManip)
    {
      std::cout << "The manipulator is in collision with itself" << std::endl;
    }
    if(!collisionBody&&!collisionManip)
    {
      std::cout << "No collisions detected!" << std::endl;
    }*/

    return collisionBody || collisionManip;
}

bool CollisionDetector::isWristColliding(const std::vector<double> manip_joints)
{
    if(!includeWrist) initializeWristModel();

    // Get sherpa into the desired configuration
    /* Pan of the whole leg */
    // sherpatt->getDof("alpha_front_left")->setPosition(0.0 * M_PI / 180.0);

    // Get the manipulator into the desired configuration
    manipulator_wrist->getDof("arm_joint_1")->setPosition(manip_joints[0]);
    manipulator_wrist->getDof("arm_joint_2")->setPosition(manip_joints[1]);
    manipulator_wrist->getDof("arm_joint_3")->setPosition(manip_joints[2]);
    manipulator_wrist->getDof("arm_joint_4")->setPosition(manip_joints[3]);

    // Look through the collisions manipulator-sherpaTT and manipulator with
    // itself
    auto collisionEngine = mWorld->getConstraintSolver()->getCollisionDetector();
    auto sherpaGroup = collisionEngine->createCollisionGroup(sherpatt.get());
    auto manipGroup = collisionEngine->createCollisionGroup(manipulator_wrist.get());

    dart::collision::CollisionOption option;
    dart::collision::CollisionResult result;
    bool collisionBody = sherpaGroup->collide(manipGroup.get(), option, &result);
    bool collisionManip = manipGroup->collide(option, &result);

    /*if(collisionBody)
    {
      std::cout << "The manipulator is in collision with the body" << std::endl;
    }
    if(collisionManip)
    {
      std::cout << "The manipulator is in collision with itself" << std::endl;
    }
    if(!collisionBody&&!collisionManip)
    {
      std::cout << "No collisions detected!" << std::endl;
    }*/

    return collisionBody || collisionManip;
}
