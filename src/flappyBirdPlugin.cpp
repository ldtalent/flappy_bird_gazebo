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

#include <flappy_bird_gazebo/flappyBirdPlugin.h>
#include <string>


void flappyBirdPlugin::Init()
{
  node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  node_->Init("flappy bird gazebo");

  ros::init(
    ros::M_string(), "flappy_bird_gazebo_node",
    ros::init_options::NoSigintHandler);

  contacts_sub_ = nh_.subscribe(
    "/robot_bumper", 1, &flappyBirdPlugin::contactsCallback, this);

  loadParameters();
  setVariables();
}

void flappyBirdPlugin::loadParameters()
{
  // load parameters from yaml file
  std::string yaml_path =
    ros::package::getPath("flappy_bird_gazebo") + "/config/sim_params.yaml";

  YAML::Node config = YAML::LoadFile(yaml_path);

  starting_distance_ = config["starting_distance"].as<double>();
  interpipe_distance_ = config["interpipe_distance"].as<double>();
  pipe_gap_ = config["pipe_gap"].as<double>();
  pipe_separation_limits_ =
    config["pipe_separation_limits"].as<std::vector<double>>();

  // non configurable
  home_pose_ = ignition::math::Pose3d(0, 0, 5.0, 0, 0, 0);
  background_pose_ = ignition::math::Pose3d(0, 1, 11, M_PI/2, 0, 0);
  pipe_length_ = 10.0;
  background_length_ = 28.0;
  vertical_clipping_distance_ = 15.0;
  clearing_safety_buffer_ = 10;
  rendering_pipe_count_ = 5;
}

void flappyBirdPlugin::setVariables()
{
  // open the model file and save it to a string
  std::string model_string, line, model_path;

  // fill pipe sdf
  model_path =
    ros::package::getPath("flappy_bird_gazebo") + "/models/pipe/model.sdf";

  std::ifstream pipe_model_file(model_path.c_str());
  while (std::getline(pipe_model_file, line))
  {
    model_string += line + "\n";
  }
  pipe_sdf_.SetFromString(model_string);
  pipe_sdf_model_ = pipe_sdf_.Root()->GetElement("model");

  // fill background sdf
  model_path =
    ros::package::getPath("flappy_bird_gazebo") + "/models/flappy_bg/model.sdf";

  std::ifstream bg_model_file(model_path.c_str());
  model_string = "";
  while (std::getline(bg_model_file, line))
  {
    model_string += line + "\n";
  }
  bg_sdf_.SetFromString(model_string);
  bg_sdf_model_ = bg_sdf_.Root()->GetElement("model");

  pipe_count_ = 0;
  bg_count_ = 0;
  px_ = starting_distance_;

  crashed_ = false;
  ros::param::set("/crashed", crashed_);
}

void flappyBirdPlugin::spawnRobotAtHome()
{
  robot_model_ = world_->ModelByName("flappy_bird_gazebo");

  if (!robot_model_)
  {
    return;
  }
  robot_model_->SetWorldPose(home_pose_);
}

void flappyBirdPlugin::spawnPipe(int idx, double x, double z, bool invert)
{
  // spawn pipes
  if (!invert)
  {
    pipe_sdf_model_->GetElement("pose")->Set(
      ignition::math::Pose3d(x, 0, z, 0, 0, 0));
  }
  else
  {
    pipe_sdf_model_->GetElement("pose")->Set(
      ignition::math::Pose3d(x, 0, z, 0, M_PI, 0));
  }
  pipe_sdf_model_->GetAttribute("name")->SetFromString(
    "pipe" + std::to_string(idx));

  world_->InsertModelSDF(pipe_sdf_);
}

void flappyBirdPlugin::spawnBackground()
{
  // append multiplier
  background_pose_.Pos().X() = background_length_ * bg_count_;
  bg_sdf_model_->GetElement("pose")->Set(background_pose_);
  bg_sdf_model_->GetAttribute("name")->SetFromString(
    "bg" + std::to_string(bg_count_));

  world_->InsertModelSDF(bg_sdf_);
  bg_count_++;
}

void flappyBirdPlugin::spawnPipePair(double separation)
{
  // spawn pipes
  spawnPipe(pipe_count_, px_, -pipe_length_/2 + separation);
  spawnPipe(pipe_count_ + 1, px_, pipe_length_/2 + separation +
    pipe_gap_, true);

  px_ += interpipe_distance_;
  pipe_count_+= 2;
}

void flappyBirdPlugin::deletePipes()
{
  // add safety buffer
  for (int i = 0; i < pipe_count_ + clearing_safety_buffer_; i++)
  {
    gazebo::physics::ModelPtr delete_pipe =
      world_->ModelByName("pipe" + std::to_string(i));

    if (!delete_pipe)
    {
      continue;
    }
    else
    {
      world_->RemoveModel(delete_pipe);
    }
  }
  // reset pipe count
  pipe_count_ = 0;
  px_ = starting_distance_;
  sleep(1);  // cooldown
}

void flappyBirdPlugin::contactsCallback(
  const gazebo_msgs::ContactsState::ConstPtr& msg)
{
  if (msg->states.size() > 0)
  {
    ros::param::set("/crashed", true);
    spawnRobotAtHome();
    deletePipes();
    ros::param::set("/crashed", false);
  }
}

void flappyBirdPlugin::Load(
  gazebo::physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
  world_ = _parent;
  updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    boost::bind(&flappyBirdPlugin::OnUpdate, this));
}


void flappyBirdPlugin::OnUpdate()
{
  robot_model_ = world_->ModelByName("flappy_bird_gazebo");

  if (!robot_model_)
  {
    return;
  }

  // get the position of the robot
  ignition::math::Pose3d pose = robot_model_->WorldPose();
  double robot_x = pose.Pos().X();
  double robot_z = pose.Pos().Z();

  // spawn background if robot has crossed half of the current background
  if (robot_x > background_length_/2 * bg_count_)
  {
    if (bg_count_ == 0)
    {
      spawnBackground();
    }
    spawnBackground();
  }

  if ((robot_x > 1) && (robot_x >
    px_- rendering_pipe_count_ * interpipe_distance_))
  {
    // limit the robot's height
    if (robot_z > vertical_clipping_distance_)
    {
      ros::param::set("/crashed", true);
      spawnRobotAtHome();
      deletePipes();
      ros::param::set("/crashed", false);
    }
    else
    {
      unsigned int seed =
        std::chrono::system_clock::now().time_since_epoch().count();
      std::default_random_engine generator(seed);
      std::uniform_real_distribution<double> distribution(
        pipe_separation_limits_[0], pipe_separation_limits_[1]);
      double separation = distribution(generator);

      spawnPipePair(separation);
    }
  }
}
GZ_REGISTER_WORLD_PLUGIN(flappyBirdPlugin)
