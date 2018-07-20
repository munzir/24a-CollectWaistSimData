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

#include "MyWindow.hpp"

#include <iostream>

//====================================================================
MyWindow::MyWindow(Controller* _controller)
  : SimWindow(),
    mController(_controller){
  assert(_controller != nullptr);

  //Robot will initialize to pose in line 1. When mController->stepPose(mLineNum)
  //is called, it should step to line #2
  mLineNum = 2;

  mRecordNum = 1;
  n = 10;

  poseSwitched = false;
}

//====================================================================
MyWindow::~MyWindow() {}

//====================================================================
void MyWindow::timeStepping() {

  mController->update();

  // Step forward the simulation
  mWorld->step();
}

//====================================================================
void MyWindow::keyboard(unsigned char _key, int _x, int _y) {

  switch (_key) {
    case 'p':
      mController->stepPose(mLineNum);
      poseSwitched = true;
      mRecordNum = 1;
      mLineNum++;
      break;
    case 't':
      if(mRecordNum <= n && poseSwitched){
        mController->recordData(mRecordNum);
        mRecordNum++;
      }
      else{
        poseSwitched = false;
        // mRecordNum = 1;
        std::cout << "WARNING: Change pose (p) before recording more data." << std::endl;
      }
      break;
    default:
      // Default keyboard control
      SimWindow::keyboard(_key, _x, _y);
      break;
  }

  // Keyboard control for Controller
  mController->keyboard(_key, _x, _y);

  glutPostRedisplay();
}
