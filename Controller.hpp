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

#ifndef EXAMPLES_OPERATIONALSPACECONTROL_CONTROLLER_HPP_
#define EXAMPLES_OPERATIONALSPACECONTROL_CONTROLLER_HPP_

#include <Eigen/Eigen>
#include <string>
#include <dart/dart.hpp>

#define DOF 17

/// \brief Operational space controller for 6-dof manipulator
class Controller {
public:
  /// \brief Constructor
  Controller( dart::dynamics::SkeletonPtr _robot);

  /// \brief Destructor
  virtual ~Controller();

  //step forward one pose
  void stepPose(int lineNum);

  //record data for this pose
  void recordData(int recordNum);

  /// \brief
  void update();

  /// \brief Get robot
  dart::dynamics::SkeletonPtr getRobot() const;

  /// \brief Keyboard control
  virtual void keyboard(unsigned char _key, int _x, int _y);

private:

  Eigen::Matrix<double, DOF, 1> qref;
  
  Eigen::Matrix<double, DOF, 1> dqref;

  /// \brief Robot
  dart::dynamics::SkeletonPtr mRobot;

  /// \brief Control forces
  Eigen::VectorXd mForces;

  // Erroneous Force
  double mForceErr;

  //time variable
  double mTime;

  //amplitude
  double A;

  //period
  double b;

  //number of poses in input file
  int poses;

    /// \brief Proportional gain for the virtual spring forces at the end effector
  Eigen::Matrix<double,DOF,DOF> mKp;

  /// \brief Derivative gain for the virtual spring forces at the end effector
  Eigen::Matrix<double,DOF,DOF> mKv;

  /// \brief Proportional gain for the virtual spring forces at the end effector
  // Eigen::Matrix3d mKp;

  /// \brief Derivative gain for the virtual spring forces at the end effector
  // Eigen::Matrix3d mKv;
};

#endif  // EXAMPLES_OPERATIONALSPACECONTROL_CONTROLLER_HPP_
