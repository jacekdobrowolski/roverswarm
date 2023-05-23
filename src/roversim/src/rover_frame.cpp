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

#include "roversim/rover_frame.h"

#include <QPointF>

#include <ros/package.h>
#include <cstdlib>
#include <ctime>

#define DEFAULT_BG_R 163
#define DEFAULT_BG_G 52
#define DEFAULT_BG_B 24

namespace roversim
{

RoverFrame::RoverFrame(int frame_width, int frame_height, QWidget* parent, Qt::WindowFlags f)
: QFrame(parent, f)
, path_image_(frame_width, frame_height, QImage::Format_ARGB32)
, path_painter_(&path_image_)
, frame_count_(0)
, id_counter_(0)
, private_nh_("~")
{
  setFixedSize(frame_width, frame_height);
  setWindowTitle("RoverSim");

  srand(time(NULL));

  update_timer_ = new QTimer(this);
  update_timer_->setInterval(16);
  update_timer_->start();

  connect(update_timer_, SIGNAL(timeout()), this, SLOT(onUpdate()));

  if (!private_nh_.hasParam("background_r"))
  {
    private_nh_.setParam("background_r", DEFAULT_BG_R);
  }
  if (!private_nh_.hasParam("background_g"))
  {
    private_nh_.setParam("background_g", DEFAULT_BG_G);
  }
  if (!private_nh_.hasParam("background_b"))
  {
    private_nh_.setParam("background_b", DEFAULT_BG_B);
  }

  QVector<QString> rovers;
  rovers.append("mars/rover_100.png");


  QString images_path = (ros::package::getPath("roversim") + "/images/").c_str();
  for (int i = 0; i < rovers.size(); ++i)
  {
    QImage img;
    img.load(images_path + rovers[i]);
    rover_images_.append(img);
  }

  meter_ = 1;

  clear();

  clear_srv_ = nh_.advertiseService("clear", &RoverFrame::clearCallback, this);
  reset_srv_ = nh_.advertiseService("reset", &RoverFrame::resetCallback, this);
  spawn_srv_ = nh_.advertiseService("spawn", &RoverFrame::spawnCallback, this);
  kill_srv_ = nh_.advertiseService("kill", &RoverFrame::killCallback, this);

  ROS_INFO("Starting roversim with node name %s", ros::this_node::getName().c_str()) ;

  width_in_meters_ = (width() - 1) / meter_;
  height_in_meters_ = (height() - 1) / meter_;
  // spawnRover("", width_in_meters_ / 2.0, height_in_meters_ / 2.0, PI / 2.0);

  // spawn all available rover types
  if(false)
  {
    for(int index = 0; index < rovers.size(); ++index)
    {
      QString name = rovers[index];
      name = name.split(".").first();
      name.replace(QString("-"), QString(""));
      spawnRover(name.toStdString(), 1.0 + 1.5 * (index % 7), 1.0 + 1.5 * (index / 7), PI / 2.0, index);
    }
  }
}

RoverFrame::~RoverFrame()
{
  delete update_timer_;
}

bool RoverFrame::spawnCallback(roversim::Spawn::Request& req, roversim::Spawn::Response& res)
{
  std::string name = spawnRover(req.name, req.x, req.y, req.theta);
  if (name.empty())
  {
    ROS_ERROR("A roverd named [%s] already exists", req.name.c_str());
    return false;
  }

  res.name = name;

  return true;
}

bool RoverFrame::killCallback(roversim::Kill::Request& req, roversim::Kill::Response&)
{
  M_Rover::iterator it = rovers_.find(req.name);
  if (it == rovers_.end())
  {
    ROS_ERROR("Tried to kill rover [%s], which does not exist", req.name.c_str());
    return false;
  }

  rovers_.erase(it);
  update();

  return true;
}

bool RoverFrame::hasRover(const std::string& name)
{
  return rovers_.find(name) != rovers_.end();
}

std::string RoverFrame::spawnRover(const std::string& name, float x, float y, float angle)
{
  return spawnRover(name, x, y, angle, rand() % rover_images_.size());
}

std::string RoverFrame::spawnRover(const std::string& name, float x, float y, float angle, size_t index)
{
  std::string real_name = name;
  if (real_name.empty())
  {
    do
    {
      std::stringstream ss;
      ss << "rover" << ++id_counter_;
      real_name = ss.str();
    } while (hasRover(real_name));
  }
  else
  {
    if (hasRover(real_name))
    {
      return "";
    }
  }

  RoverPtr t(new Rover(ros::NodeHandle(real_name), rover_images_[index], QPointF(x, height_in_meters_ - y), angle));
  rovers_[real_name] = t;
  update();

  ROS_INFO("Spawning rover [%s] at x=[%f], y=[%f], theta=[%f]", real_name.c_str(), x, y, angle);

  return real_name;
}

void RoverFrame::clear()
{
  int r = DEFAULT_BG_R;
  int g = DEFAULT_BG_G;
  int b = DEFAULT_BG_B;

  private_nh_.param("background_r", r, r);
  private_nh_.param("background_g", g, g);
  private_nh_.param("background_b", b, b);

  path_image_.fill(qRgb(r, g, b));
  update();
}

void RoverFrame::onUpdate()
{
  ros::spinOnce();

  updateRovers();

  if (!ros::ok())
  {
    close();
  }
}

void RoverFrame::paintEvent(QPaintEvent*)
{
  QPainter painter(this);

  painter.drawImage(QPoint(0, 0), path_image_);

  M_Rover::iterator it = rovers_.begin();
  M_Rover::iterator end = rovers_.end();
  for (; it != end; ++it)
  {
    it->second->paint(painter);
  }
}

void RoverFrame::updateRovers()
{
  if (last_rover_update_.isZero())
  {
    last_rover_update_ = ros::WallTime::now();
    return;
  }

  bool modified = false;
  M_Rover::iterator it = rovers_.begin();
  M_Rover::iterator end = rovers_.end();
  for (; it != end; ++it)
  {
    modified |= it->second->update(0.001 * update_timer_->interval(), path_painter_, path_image_, width_in_meters_, height_in_meters_);
  }
  if (modified)
  {
    update();
  }

  ++frame_count_;
}


bool RoverFrame::clearCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  ROS_INFO("Clearing roversim.");
  clear();
  return true;
}

bool RoverFrame::resetCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  ROS_INFO("Resetting roversim.");
  rovers_.clear();
  id_counter_ = 0;
  // spawnRover("", width_in_meters_ / 2.0, height_in_meters_ / 2.0, 0);
  clear();
  return true;
}

}
