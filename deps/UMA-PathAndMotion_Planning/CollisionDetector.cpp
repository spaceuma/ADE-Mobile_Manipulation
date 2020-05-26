
#include "CollisionDetector.h"

CollisionDetector::CollisionDetector(std::string s_urdf_path_m)
{
    mWorld = dart::simulation::World::create("Empty");
    dl.addPackageDirectory("urdf", s_urdf_path_m); // TODO set this path
    std::cout << "package directory aded: " << s_urdf_path_m << "\n";
    // Load urdf models
    sherpatt = dl.parseSkeleton("package://urdf/sherpa_tt.urdf");
    manipulator = dl.parseSkeleton("package://urdf/manipulator.urdf");
    sherpatt->setName("sherpatt");
    manipulator->setName("manipulator");
    mWorld->addSkeleton(sherpatt);
    mWorld->addSkeleton(manipulator);
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
    auto collisionEngine
        = mWorld->getConstraintSolver()->getCollisionDetector();
    auto sherpaGroup = collisionEngine->createCollisionGroup(sherpatt.get());
    auto manipGroup = collisionEngine->createCollisionGroup(manipulator.get());

    dart::collision::CollisionOption option;
    dart::collision::CollisionResult result;
    bool collisionBody
        = sherpaGroup->collide(manipGroup.get(), option, &result);
    bool collisionManip = manipGroup->collide(
        option, &result); // TODO fix this, always collides

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
