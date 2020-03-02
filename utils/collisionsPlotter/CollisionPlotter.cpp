#include "CollisionPlotter.h"

CollisionPlotter::CollisionPlotter()
{
}

CollisionPlotter::~CollisionPlotter()
{
}

bool CollisionPlotter::isColliding(dart::simulation::WorldPtr mWorld,
                                   const std::vector<double> manip_joints)
{

  // Load urdf models
  dart::utils::DartLoader dl;
  dl.addPackageDirectory("urdf","/home/ares/ADE-Mobile_Manipulation/data/urdf"); //TODO set this path
  dart::dynamics::SkeletonPtr sherpatt
      = dl.parseSkeleton("package://urdf/sherpa_tt.urdf");
  dart::dynamics::SkeletonPtr manipulator
      = dl.parseSkeleton("package://urdf/manipulator.urdf");
  sherpatt->setName("sherpatt");
  manipulator->setName("manipulator");

  // Get sherpa into the desired configuration
  /* Pan of the whole leg */
  //sherpatt->getDof("alpha_front_left")->setPosition(0.0 * M_PI / 180.0); 

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
  //sherpatt->getDof("gamma1_fake_front_left")->setPosition(0.61);
  sherpatt->getDof("gamma2_fake_front_left")->setPosition(0.61);

  sherpatt->getDof("gamma_front_right")->setPosition(0.61);
  //sherpatt->getDof("gamma1_fake_front_right")->setPosition(0.61);
  sherpatt->getDof("gamma2_fake_front_right")->setPosition(0.61);

  sherpatt->getDof("gamma_rear_left")->setPosition(0.61);
  //sherpatt->getDof("gamma1_fake_rear_left")->setPosition(0.61);
  sherpatt->getDof("gamma2_fake_rear_left")->setPosition(0.61);

  sherpatt->getDof("gamma_rear_right")->setPosition(0.61);
  //sherpatt->getDof("gamma1_fake_rear_right")->setPosition(0.61);
  sherpatt->getDof("gamma2_fake_rear_right")->setPosition(0.61);

  // Get the manipulator into the desired configuration
  manipulator->getDof("arm_joint_1")->setPosition(manip_joints[0] * M_PI / 180.0);
  manipulator->getDof("arm_joint_2")->setPosition(manip_joints[1] * M_PI / 180.0);
  manipulator->getDof("arm_joint_3")->setPosition(manip_joints[2] * M_PI / 180.0);
  manipulator->getDof("arm_joint_4")->setPosition(manip_joints[3] * M_PI / 180.0);
  manipulator->getDof("arm_joint_5")->setPosition(manip_joints[4] * M_PI / 180.0);
  manipulator->getDof("arm_joint_6")->setPosition(manip_joints[5] * M_PI / 180.0);

  // Look through the collisions manipulator-sherpaTT and manipulator with itself
  auto collisionEngine = mWorld->getConstraintSolver()->getCollisionDetector();
  auto sherpaGroup = collisionEngine->createCollisionGroup(sherpatt.get());
  auto manipGroup = collisionEngine->createCollisionGroup(manipulator.get());

  dart::collision::CollisionOption option;
  dart::collision::CollisionResult result;
  bool collisionBody = sherpaGroup->collide(manipGroup.get(), option, &result);
  bool collisionManip = manipGroup->collide(option, &result);

  mWorld->addSkeleton(sherpatt);
  mWorld->addSkeleton(manipulator);

  if(collisionBody)
  {
    std::cout << "The manipulator is in collision with the body" << std::endl;
  }
  if(collisionManip)
  {
    std::cout << "The manipulator is in collision with itself" << std::endl;
  }

  // Position the robots base in a reasonable way
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0, 0, 1);
  auto rot90deg = tf.rotation();
  rot90deg(0,0) = 1;
  rot90deg(0,1) = 0;
  rot90deg(0,2) = 0;
  rot90deg(1,0) = 0;
  rot90deg(1,1) = 0;
  rot90deg(1,2) = 1;
  rot90deg(2,0) = 0;
  rot90deg(2,1) = -1;
  rot90deg(2,2) = 0;
  tf.rotate(rot90deg);
  sherpatt->getJoint(0)->setTransformFromParentBodyNode(tf);
  manipulator->getJoint(0)->setTransformFromParentBodyNode(tf);

  return collisionBody||collisionManip;
}
