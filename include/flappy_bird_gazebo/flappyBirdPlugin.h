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


#ifndef FLAPPY_BIRD_GAZEBO_FLAPPYBIRDPLUGIN_H
#define FLAPPY_BIRD_GAZEBO_FLAPPYBIRDPLUGIN_H

#include <ros/ros.h>
#include <ros/package.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <gazebo_msgs/ContactsState.h>
#include <geometry_msgs/Twist.h>
#include <yaml-cpp/yaml.h>

#include <queue>
#include <random>
#include <vector>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportIface.hh>


class flappyBirdPlugin: public gazebo::WorldPlugin
{
 private:
  ros::NodeHandle nh_;
  ros::Subscriber contacts_sub_;

  gazebo::physics::WorldPtr world_;
  gazebo::physics::ModelPtr robot_model_;

  gazebo::event::ConnectionPtr updateConnection_;
  gazebo::transport::NodePtr node_;
  sdf::SDF pipe_sdf_, bg_sdf_;
  sdf::ElementPtr pipe_sdf_model_, bg_sdf_model_;

  ignition::math::Pose3d home_pose_;
  ignition::math::Pose3d background_pose_;

  bool crashed_;
  int pipe_count_, bg_count_;
  int clearing_safety_buffer_;
  int rendering_pipe_count_;
  double px_;
  double starting_distance_;
  double interpipe_distance_;
  double pipe_gap_;
  std::vector<double> pipe_separation_limits_;
  double vertical_clipping_distance_;
  double pipe_length_;
  double background_length_;


 public:
  void Init();
  void setVariables();
  void loadParameters();
  void spawnRobotAtHome();
  void spawnPipe(int idx, double x, double z, bool invert = false);
  void spawnPipePair(double separation);
  void deletePipes();
  void spawnBackground();
  void contactsCallback(const gazebo_msgs::ContactsState::ConstPtr& msg);
  void Load(gazebo::physics::WorldPtr _parent, sdf::ElementPtr _sdf);
  void OnUpdate();
};

#endif  // FLAPPY_BIRD_GAZEBO_FLAPPYBIRDPLUGIN_H
