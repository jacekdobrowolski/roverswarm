/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <QApplication>

#include <ros/ros.h>

#include "roversim/rover_frame.h"

class RoverApp : public QApplication
{
public:
  ros::NodeHandlePtr nh_;
  int width = 800;
  int height = 800;


  RoverApp(int& argc, char** argv)
    : QApplication(argc, argv)
  {
    ros::init(argc, argv, "roversim", ros::init_options::NoSigintHandler);
    nh_.reset(new ros::NodeHandle);

    std::string width_param_name;
    if (nh_->searchParam("width", width_param_name)) {
      nh_->getParam(width_param_name, width);

    }
    std::string height_param_name;
    if (nh_->searchParam("height", height_param_name)) {
      nh_->getParam(height_param_name, height);
    }
    ROS_INFO("Setting frame size to %d, %d", width, height);
  }

  int exec()
  {
    roversim::RoverFrame frame(width, height);
    frame.show();

    return QApplication::exec();
  }
};

int main(int argc, char** argv)
{
  RoverApp app(argc, argv);
  return app.exec();
}

