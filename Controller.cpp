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

 //output file
 ofstream q_out_file("../data/qWaistData.txt");
 ofstream dq_out_file("../data/dqWaistData.txt");
 ofstream ddq_out_file("../data/ddqWaistData.txt");
 ofstream M_out_file("../data/mWaistData.txt");
 ofstream Cg_out_file("../data/cgWaistData.txt");
 ofstream T_out_file("../data/torqueWaistData.txt");

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
}

//=========================================================================
Controller::~Controller() {}


//=========================================================================
// void printMatrix(Eigen::MatrixXd A){
//   for(int i=0; i<A.rows(); i++){
//     for(int j=0; j<A.cols(); j++){
//       std::cout << A(i,j) << ", ";
//     }
//     std::cout << std::endl;
//   }
//   std::cout << std::endl;
// }


//=========================================================================
void Controller::update() {

  // Get the stuff that we need for the robot
  Eigen::MatrixXd M     = mRobot->getMassMatrix();                // n x n
  Eigen::VectorXd Cg    = mRobot->getCoriolisAndGravityForces();  // n x 1
  Eigen::VectorXd q     = mRobot->getPositions();                 // n x 1
  Eigen::VectorXd dq    = mRobot->getVelocities();                // n x 1


  qref = q;
  dqref << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
  qref(0) += A*cos(b*mTime);
  Eigen::VectorXd ddqref = -mKp*(q - qref) - mKv*(dq - dqref); // n x 1

  mTime += 0.001;
  mForces = M*ddqref + Cg;

  // Apply the joint space forces to the robot
  mRobot->setForces(mForces);
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
    ifstream in_file("../data/filteredPosesrandom6003fullbalance0.001000tolsafe2.000000e-3filter.txt");
    while(getline(in_file, line)){
      i++;
      if (i != lineNum){
        //do nothing
      }
      else{
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
      } //end else
    }
    in_file.close();
    //set position for all joints except waist
    for(int k = 1; k<17; k++){
      mRobot->getDof(k)->setPosition(vect(k));
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
  M_out_file << M << endl << endl << endl;

  Eigen::VectorXd Cg = mRobot->getCoriolisAndGravityForces();
  Cg_out_file << Cg.transpose() << endl;

  // cout << "Torque: " << mForces(0) << endl;
  T_out_file << mForces(0) << endl;

  cout << "Data " << recordNum << " Recorded" << endl;
}


//=========================================================================
void Controller::keyboard(unsigned char /*_key*/, int /*_x*/, int /*_y*/) {

}
