/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Ryan Sinnet
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ryan Sinnet */

#include <initializer_list>
#include <memory>
#include <utility>

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include "multi_robot_state_publisher/joint_state_listener.h"

namespace multi_robot_state_publisher
{
class MultiRobotStatePublisherConfig
{
public:
  inline static const std::string CONFIG_PATH{ "~robots" };

  static std::vector<std::string> getRobotNamesFromROSParam();

  template <typename T, typename>
  static std::vector<T> getRobotConfigsFromROSParam();
};

std::vector<std::string> MultiRobotStatePublisherConfig::getRobotNamesFromROSParam()
{
  std::vector<std::string> robots;
  XmlRpc::XmlRpcValue data;

  std::stringstream ss;
  ss << "Loading robot configs from `" << CONFIG_PATH << "' for the following robots: ";

  ros::NodeHandle nh{ "~" };
  ROS_INFO_STREAM("NodeHandle namespace: " << nh.getNamespace());
  nh.getParam("robots", data);
  for (auto it{ data.begin() }; it != data.end(); ++it)
  {
    static constexpr const char* msg{ "Invalid parameter `%s' found polluting namespace `%s'." };
    ROS_ASSERT_MSG(it->second.getType() == XmlRpc::XmlRpcValue::TypeStruct, msg, it->first.c_str(),
                   CONFIG_PATH.c_str());
    robots.push_back(it->first);
    ss << "\n  " << robots.back();
  }
  if (robots.empty())
  {
    auto msg{ "No robots loaded from param server!" };
    ROS_ERROR_STREAM(msg);
    throw std::invalid_argument{ msg };
  }
  ROS_INFO_STREAM(ss.str().c_str());
  return robots;
}
}  // namespace multi_robot_state_publisher

using multi_robot_state_publisher::JointStateListener;
using multi_robot_state_publisher::MultiRobotStatePublisherConfig;

class MultiRobotStatePublisher
{
public:
  //! \brief Construct a new MultiRobotStatePublisher.
  MultiRobotStatePublisher();

  //! \brief Publish all frames from all robots in one message.
  //!
  //! \param time The time to be used in published headers.
  //! \parmm publish_tf_static If true, publish static frames instead.
  void publish(const ros::Time& time, bool publish_tf_static = false);

private:
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

  std::vector<std::unique_ptr<JointStateListener>> listeners_;
  std::vector<std::unique_ptr<JointStateListener>> static_non_listeners_;

  std::vector<geometry_msgs::TransformStamped> transforms_;
  std::vector<geometry_msgs::TransformStamped> static_transforms_;

  void addRobots(const std::vector<std::string>& robots);
};

MultiRobotStatePublisher::MultiRobotStatePublisher()
{
  const auto robots{ MultiRobotStatePublisherConfig::getRobotNamesFromROSParam() };
  addRobots(robots);
}

void MultiRobotStatePublisher::addRobots(const std::vector<std::string>& robots)
{
  for (const auto& robot : robots)
  {
    const auto path{ MultiRobotStatePublisherConfig::CONFIG_PATH + "/" + robot };
    auto listener{ std::make_unique<JointStateListener>(ros::NodeHandle{}, ros::NodeHandle{ path }) };
    auto& container{ listener->isStatic() ? this->static_non_listeners_ : this->listeners_ };
    container.emplace_back(std::move(listener));
  }
}

void MultiRobotStatePublisher::publish(const ros::Time& time, bool publish_tf_static)
{
  const auto& listeners{ publish_tf_static ? this->static_non_listeners_ : this->listeners_ };
  auto& transforms{ publish_tf_static ? this->static_transforms_ : this->transforms_ };

  transforms.clear();
  for (const auto& listener : listeners)
  {
    listener->getTransforms(time, &transforms);
  }

  if (transforms.empty())
  {
    return;
  }

  if (publish_tf_static)
  {
    this->static_tf_broadcaster_.sendTransform(transforms);
  }
  else
  {
    this->tf_broadcaster_.sendTransform(transforms);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "multi_robot_state_publisher");
  ros::NodeHandle np{ "~" };

  auto publishing_rate{ [&np]() {
    double publishing_rate;
    np.param<double>("publishing_rate", publishing_rate, 50.);
    return ros::Rate{ publishing_rate };
  }() };

  MultiRobotStatePublisher robot_state_publisher;
  robot_state_publisher.publish(ros::Time::now(), true);
  while (ros::ok())
  {
    robot_state_publisher.publish(ros::Time::now());
    ros::spinOnce();
    publishing_rate.sleep();
  }
  return 0;
}
