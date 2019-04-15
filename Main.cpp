/*
 * Copyright (c) 2014-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2014-2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016-2017, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

// Includes
#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include "MyWindow.hpp"
#include <iostream>
#include <fstream>

 // Namespaces
using namespace std;
using namespace dart::collision;
using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::gui;
using namespace dart::math;
using namespace dart::simulation;
using namespace dart::utils;

// Function Prototypes
SkeletonPtr createKrang(string fullRobotPath, string robotName);
SkeletonPtr createFloor(string floorName);

dart::dynamics::SkeletonPtr createKrang() {
  // Load the Skeleton from a file
  DartLoader loader;
  SkeletonPtr krang =
     loader.parseSkeleton("/home/areeb/dart/09-URDF/KrangWaist/krang_fixed_base.urdf");
  krang->setName("krang");

  // Position its base in a reasonable way
  Eigen::Isometry3d tf  = Eigen::Isometry3d::Identity();
  // Eigen::Isometry3d tfRot  = Eigen::Isometry3d::Identity();

  tf.translation()      = Eigen::Vector3d(0.0, 0.0, 0.0);
  // tf *= Eigen::AngleAxisd(1.57, Eigen::Vector3d::UnitX());
  krang->getJoint(0)->setTransformFromParentBodyNode(tf);

  return krang;
}


SkeletonPtr createFloor(string floorName) {
    SkeletonPtr floor = Skeleton::create(floorName);
    return floor;
}


int main(int argc, char* argv[])
{

  string floorName = "floor";

  // create and initialize the world
  WorldPtr world(new World);
  assert(world != nullptr);

  // load skeletons
  DartLoader dl;
  SkeletonPtr floor = createFloor(floorName);
  SkeletonPtr krang = createKrang();

  double pi = M_PI;

  Eigen::VectorXd inPos(17);
  //world link (fixed)
  //fixbase link (fixed to prev)
  //Base (fixed to prev)
  inPos(0) = 4*pi/6; //Waist (revolute to prev)
  inPos(1) = pi; //Torso (should be pi in URDF..)
  inPos(2) = 0;
  inPos(3) = 0;
  inPos(4) = 0;
  inPos(5) = 0;
  inPos(6) = 0;
  inPos(7) = 0;
  inPos(8) = 0;
  inPos(9) = 0;
  inPos(10) = 0;
  inPos(11) = 0;
  inPos(12) = 0;
  inPos(13) = 0;
  inPos(14) = 0;
  inPos(15) = 0;
  inPos(16) = 0;

  krang->setPositions(inPos);

  world->addSkeleton(floor); //add ground and krang to the world pointer
  world->addSkeleton(krang);

  int numBodies = krang->getNumBodyNodes();

  krang->getJoint(0)->setActuatorType(Joint::ActuatorType::LOCKED);

  for(int i = 2; i<numBodies; i++) {
    krang->getJoint(i)->setActuatorType(Joint::ActuatorType::LOCKED);
  }


  // create and initialize the world
  Eigen::Vector3d gravity(0.0, -9.81, 0.0);
  world->setGravity(gravity);
  world->setTimeStep(1.0/1000);

  // create a window and link it to the world
  MyWindow window(new Controller(krang) );
  window.setWorld(world);

  glutInit(&argc, argv);
  window.initWindow(960, 720, "Krang Waist");
  glutMainLoop();

  return 0;
}
