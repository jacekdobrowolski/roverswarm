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

#include "roversim/rover.h"

#include <QColor>
#include <QRgb>

#define DEFAULT_PEN_R 0xdf
#define DEFAULT_PEN_G 0x6a
#define DEFAULT_PEN_B 0x4d

namespace roversim
{

Rover::Rover(const ros::NodeHandle& nh, const QImage& rover_image, const QPointF& pos, float orient)
: nh_(nh)
, rover_image_(rover_image)
, pos_(pos)
, orient_(orient)
, lin_vel_x_(0.0)
, lin_vel_y_(0.0)
, ang_vel_(0.0)
, pen_on_(true)
, pen_(QColor(DEFAULT_PEN_R, DEFAULT_PEN_G, DEFAULT_PEN_B))
{
  pen_.setWidth(100); // TODO: READ FROM ROS PARAMS
  pen_.setCapStyle(Qt::RoundCap);

  velocity_sub_ = nh_.subscribe("cmd_vel", 1, &Rover::velocityCallback, this);
  pose_pub_ = nh_.advertise<Pose>("pose", 1);
  color_pub_ = nh_.advertise<Color>("color_sensor", 1);
  teleport_relative_srv_ = nh_.advertiseService("teleport_relative", &Rover::teleportRelativeCallback, this);
  teleport_absolute_srv_ = nh_.advertiseService("teleport_absolute", &Rover::teleportAbsoluteCallback, this);

  meter_ = rover_image_.height();
  rotateImage();
}


void Rover::velocityCallback(const geometry_msgs::Twist::ConstPtr& vel)
{
  last_command_time_ = ros::WallTime::now();
  lin_vel_x_ = vel->linear.x;
  lin_vel_y_ = vel->linear.y;
  ang_vel_ = vel->angular.z;
}

bool Rover::teleportRelativeCallback(roversim::TeleportRelative::Request& req, roversim::TeleportRelative::Response&)
{
  teleport_requests_.push_back(TeleportRequest(0, 0, req.angular, req.linear, true));
  return true;
}

bool Rover::teleportAbsoluteCallback(roversim::TeleportAbsolute::Request& req, roversim::TeleportAbsolute::Response&)
{
  teleport_requests_.push_back(TeleportRequest(req.x, req.y, req.theta, 0, false));
  return true;
}

void Rover::rotateImage()
{
  QTransform transform;
  transform.rotate(-orient_ * 180.0 / PI);
  rover_rotated_image_ = rover_image_.transformed(transform);
}

bool Rover::update(double dt, QPainter& path_painter, const QImage& path_image, qreal canvas_width, qreal canvas_height)
{
  bool modified = false;
  qreal old_orient = orient_;

  // first process any teleportation requests, in order
  V_TeleportRequest::iterator it = teleport_requests_.begin();
  V_TeleportRequest::iterator end = teleport_requests_.end();
  for (; it != end; ++it)
  {
    const TeleportRequest& req = *it;

    QPointF old_pos = pos_;
    if (req.relative)
    {
      orient_ += req.theta;
      pos_.rx() += std::cos(orient_) * req.linear;
      pos_.ry() += - std::sin(orient_) * req.linear;
    }
    else
    {
      pos_.setX(req.pos.x());
      pos_.setY(std::max(0.0, static_cast<double>(canvas_height - req.pos.y())));
      orient_ = req.theta;
    }

    if (pen_on_)
    {
      path_painter.setPen(pen_);
      path_painter.drawLine(pos_ * meter_, old_pos * meter_);
    }
    modified = true;
  }

  teleport_requests_.clear();

  if (ros::WallTime::now() - last_command_time_ > ros::WallDuration(1.0))
  {
    lin_vel_x_ = 0.0;
    lin_vel_y_ = 0.0;
    ang_vel_ = 0.0;
  }

  QPointF old_pos = pos_;

  orient_ = orient_ + ang_vel_ * dt;
  // Keep orient_ between -pi and +pi
  orient_ -= 2*PI * std::floor((orient_ + PI)/(2*PI));
  pos_.rx() += std::cos(orient_) * lin_vel_x_ * dt
             - std::sin(orient_) * lin_vel_y_ * dt;
  pos_.ry() -= std::cos(orient_) * lin_vel_y_ * dt
             + std::sin(orient_) * lin_vel_x_ * dt;

  // Clamp to screen size
  if (pos_.x() < 0 || pos_.x() > canvas_width ||
      pos_.y() < 0 || pos_.y() > canvas_height)
  {
    ROS_WARN("Oh no! I hit the wall! (Clamping from [x=%f, y=%f])", pos_.x(), pos_.y());
  }

  pos_.setX(std::min(std::max(static_cast<double>(pos_.x()), 0.0), static_cast<double>(canvas_width)));
  pos_.setY(std::min(std::max(static_cast<double>(pos_.y()), 0.0), static_cast<double>(canvas_height)));

  // Publish pose of the rover
  Pose p;
  p.x = pos_.x();
  p.y = canvas_height - pos_.y();
  p.theta = orient_;
  p.linear_velocity = std::sqrt(lin_vel_x_ * lin_vel_x_ + lin_vel_y_ * lin_vel_y_);
  p.angular_velocity = ang_vel_;
  pose_pub_.publish(p);

  // Figure out (and publish) the color underneath the rover
  {
    Color color;
    QRgb pixel = path_image.pixel((pos_ * meter_).toPoint());
    color.r = qRed(pixel);
    color.g = qGreen(pixel);
    color.b = qBlue(pixel);
    color_pub_.publish(color);
  }

  ROS_DEBUG("[%s]: pos_x: %f pos_y: %f theta: %f", nh_.getNamespace().c_str(), pos_.x(), pos_.y(), orient_);

  if (orient_ != old_orient)
  {
    rotateImage();
    modified = true;
  }
  if (pos_ != old_pos)
  {
    if (pen_on_)
    {
      path_painter.setPen(pen_);
      path_painter.drawLine(pos_ * meter_, old_pos * meter_);
    }
    modified = true;
  }

  return modified;
}

void Rover::paint(QPainter& painter)
{
  QPointF p = pos_ * meter_;
  p.rx() -= 0.5 * rover_rotated_image_.width();
  p.ry() -= 0.5 * rover_rotated_image_.height();
  painter.drawImage(p, rover_rotated_image_);
}

}
