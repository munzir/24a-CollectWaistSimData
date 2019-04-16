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

#include "Controller.hpp"
#include <nlopt.hpp>
#include <string>
#include <iostream>

#include <fstream>
#include <sstream>

using namespace std;
using namespace dart;
using namespace dart::dynamics;
// using namespace sl;

// Transform objects
dart::dynamics::BodyNode* baseNode;
dart::dynamics::BodyNode* spineNode;
dart::dynamics::BodyNode* bracketNode;
dart::dynamics::BodyNode* zedHolderNode;
dart::dynamics::BodyNode* zedCameraNode;

Eigen::Isometry3d worldTransformCamera;
Eigen::Isometry3d relTransformCH;
Eigen::Isometry3d relTransformHB;
Eigen::Isometry3d relTransformBS;
Eigen::Isometry3d relTransformSB;
Eigen::Isometry3d relTransformBase;

Eigen::Vector3d relTranslationBase;
Eigen::Vector3d worldTranslationBase;

Eigen::Matrix3d relRotationBase;
Eigen::Matrix3d worldRotationBase;

// ZED objects
// sl::Camera zed;
// sl::Pose camera_pose;
// std::thread zed_callback;
// bool quit = false;

// const int MAX_CHAR = 128;

// Output files
ofstream q_out_file("../../24-ParametricIdentification-Waist/simOutData/qWaistData.txt");
ofstream dq_out_file("../../24-ParametricIdentification-Waist/simOutData/dqWaistData.txt");
ofstream ddq_out_file("../../24-ParametricIdentification-Waist/simOutData/ddqWaistData.txt");
ofstream M_out_file("../../24-ParametricIdentification-Waist/simOutData/mWaistData.txt");
ofstream Cg_out_file("../../24-ParametricIdentification-Waist/simOutData/cgWaistData.txt");
ofstream T_out_file("../../24-ParametricIdentification-Waist/simOutData/torqueWaistData.txt");
ofstream time_out_file("../../24-ParametricIdentification-Waist/simOutData/timeWaistData.txt");

// Used for constant forward/backward motion of waist
double pi = 3.14159;
int dir = 0;
double maxPos = 5*pi/6;
double minPos = pi/6;

int mNumLine = 2;
int n = 5;
int mRecordNum = 1;
int mTotalRecorded = 0;

bool endfile = false;

// Function prototypes
// void startZED();
// void runZED();
// void closeZED();
// void transformZEDPose(sl::Transform &pose, float tx);

//==========================================================================
Controller::Controller(SkeletonPtr _robot)
  : mRobot(_robot)
   {
  assert(_robot != nullptr);

  mTime = 0;

  int dof = mRobot->getNumDofs();
  std::cout << "[controller] DoF: " << dof << std::endl;

  mForces.setZero(dof);

  mKp = Eigen::Matrix<double, DOF, DOF>::Identity();
  mKv = Eigen::Matrix<double, DOF, DOF>::Identity();

  // Greater A -> larger angle
  A = 0.8;
  // Great b -> smaller period
  b = 1;

  // mRobot->getDof(0)->setVelocity(1);

  //get a count of how many poses we will be simulating
  string line;
  poses = 0;
  ifstream in_file("../../24-ParametricIdentification-Waist/simInData/finalSetDart.txt");
  while(getline(in_file, line)){
    poses++;
  }

  baseNode = mRobot->getBodyNode(0);
  spineNode = mRobot->getBodyNode(1);
  bracketNode = mRobot->getBodyNode(2);
  zedHolderNode = mRobot->getBodyNode(3);
  zedCameraNode = mRobot->getBodyNode(4);
}

//=========================================================================
Controller::~Controller() {}

//==============================================================================
double error(double dq) {
  double err;
  err = -( 2 / ( 1 + exp(-2*dq)) -1 );
  return err;
}

//=========================================================================
void Controller::update() {
  //Simply set the velocity of the waist (testing zero accel case)
  if(dir == 1){
    mRobot->getDof(0)->setVelocity(1);
  }
  if(dir == 0){
    mRobot->getDof(0)->setVelocity(-1);
  }

  if(mRobot->getDof(0)->getPosition() < minPos){
    dir = 1;
  }
  if(mRobot->getDof(0)->getPosition() > maxPos){
    dir = 0;
  }

  //Record data automatically
  if(mRecordNum == n+1){
    stepPose(mNumLine);
    mRecordNum = 1; //cout <<"mRecordNum RESET" << endl;
    mNumLine++;
  }

  int v1 = rand()%100;
  //if randomly generated number is within arbitrary range of 2, record data
  //larger range increases rate of data recording
  if(v1>20 && v1<22){
    // recordData(mRecordNum);
    // mRecordNum++;
    // mTotalRecorded++;

    // Grab transform of ZED Camera

    // Calculate angle of base from torso and waist orientation
    // Get transformations
    worldTransformCamera = zedCameraNode->getTransform();
    relTransformCH = zedHolderNode->getTransform(zedCameraNode);
    relTransformHB = bracketNode->getTransform(zedHolderNode);
    relTransformBS = spineNode->getTransform(bracketNode);
    relTransformSB = baseNode->getTransform(spineNode);
    // Multiply transformations
    // relTransformBase = worldTransformBracket*relTransformBS*relTransformSB;
    relTransformBase = worldTransformCamera*relTransformCH*relTransformHB*relTransformBS;//*relTransformSB;

    // Get translation and rotation from above computation
    relTranslationBase = relTransformBase.translation();
    relRotationBase = relTransformBase.rotation();
    // Get translation and rotation directly
    worldTranslationBase = spineNode->getTransform().translation();
    worldRotationBase = spineNode->getTransform().rotation();

    // Print out both computed and directly-derived transformations to compare
    cout << "RelTrans:   " << relTranslationBase(0) << "   " << relTranslationBase(1) << "   " << relTranslationBase(2) << endl;
    cout << "WrdTrans:   " << worldTranslationBase(0) << "   " << worldTranslationBase(1) << "   " << worldTranslationBase(2) << endl;
    cout << "RelRot: " << "   " << relRotationBase(0) << "   " << relRotationBase(1) << "   " << relRotationBase(2)
      << "   " << relRotationBase(3) << "   " << relRotationBase(4) << "   " << relRotationBase(5)
      << "   " << relRotationBase(6) << "   " << relRotationBase(7) << "   " << relRotationBase(8) << endl;
    cout << "WrdRot: " << "   " << worldRotationBase(0) << "   " << worldRotationBase(1) << "   " << worldRotationBase(2) 
      << "   " << worldRotationBase(3) << "   " << worldRotationBase(4) << "   " << worldRotationBase(5)
      << "   " << worldRotationBase(6) << "   " << worldRotationBase(7) << "   " << worldRotationBase(8) << endl;
    cout << endl << endl;

    Eigen::Matrix4d matCam = transform2Mat(worldTransformCamera);
    Eigen::Matrix4d matCH = transform2Mat(relTransformCH);
    Eigen::Matrix4d matHB = transform2Mat(relTransformHB);
    Eigen::Matrix4d matBS = transform2Mat(relTransformBS);

    Eigen::Matrix4d matTransSpine = matCam*matCH*matHB*matBS;

    cout << "RelRot: " << endl 
      << "   " << matTransSpine(0) << "   " << matTransSpine(4) << "   " << matTransSpine(8) << "   " << matTransSpine(12) << endl
      << "   " << matTransSpine(1) << "   " << matTransSpine(5) << "   " << matTransSpine(9) << "   " << matTransSpine(13) << endl
      << "   " << matTransSpine(2) << "   " << matTransSpine(6) << "   " << matTransSpine(10) << "   " << matTransSpine(14) << endl
      << "   " << matTransSpine(3) << "   " << matTransSpine(7) << "   " << matTransSpine(11) << "   " << matTransSpine(15) << endl;
  }
}

Eigen::Matrix4d Controller::transform2Mat(Eigen::Isometry3d iso){
  Eigen::Matrix4d mat;
  Eigen::Vector3d translation = iso.translation();
  Eigen::Matrix3d rotation = iso.rotation();
  mat.block<3,3>(0,0) = rotation;
  mat(0,3) = translation(0);
  mat(1,3) = translation(1);
  mat(2,3) = translation(2);
  
  mat(3,0) = 0;
  mat(3,1) = 0;
  mat(3,2) = 0;
  mat(3,3) = 1;

  return mat;
}

//=========================================================================
SkeletonPtr Controller::getRobot() const {
  return mRobot;
}

void Controller::stepPose(int lineNum) {
  int i = 1;
  string line;
  Eigen::VectorXd vect(17);
  std::string::size_type sz;
  cout << "Pose: " << lineNum << endl;
  ifstream in_file("../../24-ParametricIdentification-Waist/simInData/finalSetDart.txt");
  //Reads every line of the file every time until the given lineNum is reached (i == lineNum)
  while(getline(in_file, line)){
    i++;
    if(i == lineNum){ //i == lineNum
      //extract numbers from this line and set the poses
      stringstream lineStream(line);
	    string t;
	    int count = 0;
	    int startNum = 8; //6dof(base) + 2 wheels <- skip these numbers
	    while(getline(lineStream, t, ' ')){
	      //if t is NOT empty, means delimiter was not found on lineStream, instead value is stored in t
	      if(!t.empty()){
          //count up how many numbers have been found on the line
	        count++;
	        if(count > startNum && !lineStream.eof()){
	          //Put number in vector
	          vect(count - 9) = stod(t,&sz); //convert to double and put into vect(count - 9)
	        }
	        else if (count <= startNum){
	          //do nothing
	        }
	        else {
	          //put last number
	          vect(count - 9) = stod(t,&sz);
	        }
	      }
	    } //end while
      endfile = false;
      break;
    } //end if

    else if (i == poses){ //if end of file is reached, set bool endfile to true so that program exits
      endfile = true;
      break;
    }
  }

  if(endfile){
    cout << "Reached EOF" << endl << endl << endl;std::exit(0);
  }
  in_file.close();
  //set position for all joints except waist, zedHolder and zedCam
  mRobot->getDof(1)->setPosition(vect(1));
  mRobot->getDof(2)->setPosition(vect(2));
  for(int k = 5; k<17; k++){
      mRobot->getDof(k)->setPosition(vect(k-2));
  }
}

void Controller::recordData(int recordNum){

  Eigen::VectorXd q = mRobot->getPositions();
  q_out_file << q.transpose() << endl;

  Eigen::VectorXd dq = mRobot->getVelocities();
  dq_out_file << dq.transpose() << endl;

  Eigen::VectorXd ddq = mRobot->getAccelerations();
  ddq_out_file << ddq.transpose() << endl;

  Eigen::MatrixXd M = mRobot->getMassMatrix();
  M_out_file << M << endl;

  Eigen::VectorXd Cg = mRobot->getCoriolisAndGravityForces();
  Cg_out_file << Cg.transpose() << endl;

  time_out_file << mTime << endl;

  // cout << "Torque: " << mForces(0) << endl;
  T_out_file << mForces(0) << endl;

  cout << "Data " << recordNum << " Recorded" << endl;
}


//=========================================================================
void Controller::keyboard(unsigned char /*_key*/, int /*_x*/, int /*_y*/) {

}

void Controller::startZED(){
  // quit = false;
  // zed_callback = std::thread(runZED);
}

// Get transform to center of camera and upate transform isometry
void Controller::runZED(){
  // Get the distance between the center of the camera and the left eye
  // float translation_left_to_center = zed.getCameraInformation().calibration_parameters.T.x * 0.5f;

  // if (zed.grab() == SUCCESS) {
  //   // Get the position of the camera in a fixed reference frame (the World Frame)
  //   TRACKING_STATE tracking_state = zed.getPosition(camera_pose, sl::REFERENCE_FRAME_WORLD);
  //   if (tracking_state == TRACKING_STATE_OK) {
  //     // Get the pose at the center of the camera (baseline/2 on X axis)
  //     transformPose(camera_pose.pose_data, translation_left_to_center);
  //     // Get quaternion, rotation and translation
  //     sl::float4 quaternion = camera_pose.getOrientation();
  //     // Use quaternions for transforms. Use rotation for Euler Angles
  //     sl::float3 rotation = camera_pose.getEulerAngles();
  //     sl::float3 translation = camera_pose.getTranslation();
  //   }
  // }
}

void Controller::closeZED(){
    // quit = true;
    // zed_callback.join();
    // zed.disableTracking("./ZED_spatial_memory"); // Record an area file

    // zed.close();
}

// void Controller::transformZEDPose(sl::Transform &pose, float tx){
//     // sl::Transform transform_;
//     // transform_.setIdentity();
//     // // Move the tracking frame by tx along the X axis
//     // transform_.tx = tx;
//     // // Apply the transformation
//     // pose = Transform::inverse(transform_) * pose * transform_;
// }