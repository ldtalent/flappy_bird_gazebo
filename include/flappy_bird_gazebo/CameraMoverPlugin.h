//
// MIT License
//
// Copyright (c) 2023 Leander Stephen D'Souza
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


#ifndef FLAPPY_BIRD_GAZEBO_CAMERAMOVERPLUGIN_H
#define FLAPPY_BIRD_GAZEBO_CAMERAMOVERPLUGIN_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/rendering/UserCamera.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Visual.hh>


class CameraMoverPlugin : public gazebo::VisualPlugin
{
 public:
  CameraMoverPlugin();

  void pubHome();
  void pubVel();
  void onUpdate();
  void loadParameters();

  void Load(gazebo::rendering::VisualPtr visual,
    sdf::ElementPtr sdf) override;

 private:
  ros::NodeHandle nh_;
  ros::Publisher twist_pub_;

  double bird_velocity_;
  double camera_velocity_;
  ignition::math::Pose3d home_pose_;

  bool crashed = false;

  gazebo::rendering::UserCameraPtr m_camera;

  // Connection to the update event
  gazebo::event::ConnectionPtr m_update_connection;
};

#endif  // FLAPPY_BIRD_GAZEBO_CAMERAMOVERPLUGIN_H
