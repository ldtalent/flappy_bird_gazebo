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


#include <flappy_bird_gazebo/CameraMoverPlugin.h>


GZ_REGISTER_VISUAL_PLUGIN(CameraMoverPlugin)


CameraMoverPlugin::CameraMoverPlugin()
{
  ros::init(
    ros::M_string(), "camera_mover_node",
    ros::init_options::NoSigintHandler);

  twist_pub_ = nh_.advertise<geometry_msgs::Twist>(
    "/cmd_vel", 10);

  loadParameters();
}

void CameraMoverPlugin::loadParameters()
{
  bird_velocity_ = 2.0;
  camera_velocity_ = 0.03;
  home_pose_ = ignition::math::Pose3d(0.55, -19.169, 7.15, 0, 0, M_PI/2);
}

void CameraMoverPlugin::pubVel()
{
  // Add linear velocity to x-axis
  ignition::math::Vector3d trans = {camera_velocity_, 0, 0};

  // get current pose
  ignition::math::Pose3d pose = m_camera->WorldPose();

  // add velocity to pose
  pose.Pos() += trans;

  // set new pose
  m_camera->SetWorldPose(pose);
}

void CameraMoverPlugin::pubHome()
{
  // set home pose
  m_camera->SetWorldPose(home_pose_);
}

void CameraMoverPlugin::Load(
  gazebo::rendering::VisualPtr visual, sdf::ElementPtr sdf)
{
  // Load the Plugin
  gazebo::rendering::ScenePtr scene = visual->GetScene();
  if (scene == nullptr)
  {
    gzerr << "Scene is null. CameraMoverPlugin won't load." << std::endl;
    return;
  }
  m_camera = scene->GetUserCamera(0);
  if (m_camera == nullptr)
  {
    gzerr << "Camera is null. CameraMoverPlugin won't load." << std::endl;
    return;
  }
  // Listen to the update event. This event is broadcast every sim iteration.
  m_update_connection = gazebo::event::Events::ConnectPreRender(
    boost::bind(&CameraMoverPlugin::onUpdate, this));
}

void CameraMoverPlugin::onUpdate()
{
  ros::param::get("/crashed", crashed);

  if (crashed)
  {
    pubHome();
    geometry_msgs::Twist msg;
    twist_pub_.publish(msg);
  }
  else
  {
    pubVel();
    geometry_msgs::Twist msg;
    msg.linear.x = bird_velocity_;
    twist_pub_.publish(msg);
  }
}
