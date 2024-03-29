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

#ifndef EXAMPLES_OPERATIONALSPACECONTROL_MYWINDOW_HPP_
#define EXAMPLES_OPERATIONALSPACECONTROL_MYWINDOW_HPP_

#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>

#include "Controller.hpp"

/// \brief class MyWindow
class MyWindow : public dart::gui::SimWindow
{
public:
  /// \brief Default constructor
  MyWindow(Controller* _controller);

  /// \brief Destructor
  virtual ~MyWindow();

  // Documentation inherited
  void timeStepping() override;

  // Documentation inherited
  void keyboard(unsigned char _key, int _x, int _y) override;

private:
  /// \brief Operational space controller
  Controller* mController;

  //Number of line from which to get pose
  int mLineNum;

  //Record number for current pose
  int mRecordNum;

  //Max number of recordings wanted for each pose
  int n;

  //check for whether pose has been switched before recording more data
  bool poseSwitched;
};

#endif  // EXAMPLES_OPERATIONALSPACECONTROL_MYWINDOW_HPP_
