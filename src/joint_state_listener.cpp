/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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

/* Author: Wim Meeussen */

#include "multi_robot_state_publisher/joint_state_listener.h"

#include <iomanip>
#include <ios>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/TransformStamped.h>
#include <kdl/frames_io.hpp>
#include <kdl/tree.hpp>
#include <ros/ros.h>
#include <tf2_kdl/tf2_kdl.h>
#include <tf2_ros/transform_broadcaster.h>
#include <urdf/model.h>

#include "multi_robot_state_publisher/robot_state_publisher.h"

namespace multi_robot_state_publisher
{
JointStateListener::JointStateListener(ros::NodeHandle nh, ros::NodeHandle np)
  : JointStateListener{ loadModelFromROS(np), std::move(nh), std::move(np) }
{
}
JointStateListener::JointStateListener(urdf::Model model, ros::NodeHandle nh, ros::NodeHandle np)
  : nh_{ std::move(nh) }
  , np_{ std::move(np) }
  , model_{ std::move(model) }
  , mimic_{ this->buildMimicMap(this->model_) }
  , state_publisher_{ RobotStatePublisher{ &this->model_ } }

{
  // set publish frequency
  double publish_freq;
  this->np_.param("publish_frequency", publish_freq, 50.0);
  // If true, use the /tf_static latched static transform broadcaster
  this->np_.param("use_tf_static", this->use_tf_static_, true);
  // If true, joint_states messages are accepted, no matter their timestamp.
  this->np_.param("ignore_timestamp", this->ignore_timestamp_, false);
  std::string tf_prefix_key;
  this->np_.searchParam("tf_prefix", tf_prefix_key);
  // If true, set a socket option to avoid bundling incoming joint_states messages to reduce latency.
  this->np_.param("tcp_nodelay", this->tcp_nodelay_, false);
  this->np_.param(tf_prefix_key, tf_prefix_, std::string{ "" });
  this->publish_interval_ = ros::Duration{ 1.0 / std::max(publish_freq, 1.0) };

  ROS_DEBUG_STREAM(std::boolalpha                                           //
                   << std::setprecision(1)                                  //
                   << "Configuring JointStateListener:"                     //
                   << "\n\tNamespace:        " << this->np_.getNamespace()  //
                   << "\n\tModel Name:       " << this->model_.getName()    //
                   << "\n\tPublish Rate:     " << publish_freq << " Hz"     //
                   << "\n\tStatic TF:        " << this->use_tf_static_      //
                   << "\n\tIgnore Timestamp: " << this->ignore_timestamp_   //
                   << "\n\tTCP_NODELAY:      " << this->tcp_nodelay_);

  if (!this->use_tf_static_)
  {
    ros::TransportHints transport_hints;
    // Setting tcpNoDelay here tells the publisher to set the TCP_NODELAY socket option to true which disables the
    // Nagle algorithm. Doing so causes segments to be sent as soon as they are ready instead of bundling them. This
    // means there will be more frames with smaller packets which translates to poorer network utilization but lower
    // latency for the joint_states topic, which may be important for real-time control applications.
    //
    // See also: tcp(7)
    transport_hints.tcpNoDelay(this->tcp_nodelay_);
    this->joint_state_sub_ =
        this->nh_.subscribe("joint_states", 1, &JointStateListener::callbackJointState, this, transport_hints);
  }
}

MimicMap JointStateListener::buildMimicMap(const urdf::Model& model)
{
  MimicMap mimic;
  for (const auto& [name, joint] : model.joints_)
  {
    if (joint->mimic)
    {
      mimic.insert(std::make_pair(name, joint->mimic));
    }
  }
  return mimic;
}

urdf::Model JointStateListener::loadModelFromROS(const ros::NodeHandle& np)
{
  urdf::Model model;
  if (!model.initParamWithNodeHandle("robot_description", np))
  {
    ROS_ERROR("Unable to load robot description from param server.");
    throw std::invalid_argument(np.getNamespace() + "/robot_description");
  }
  return model;
}

JointStateListener::~JointStateListener()
{
}

void JointStateListener::callbackJointState(const JointStateConstPtr& state)
{
  if (state->name.size() != state->position.size())
  {
    if (state->position.empty())
    {
      const int throttle_seconds{ 300 };
      ROS_WARN_THROTTLE(throttle_seconds,
                        "Robot state publisher ignored a JointState message about joint(s) "
                        "\"%s\"(,...) whose position member was empty. This message will "
                        "not reappear for %d seconds.",
                        state->name[0].c_str(), throttle_seconds);
    }
    else
    {
      ROS_ERROR("Robot state publisher ignored an invalid JointState message");
    }
    return;
  }

  // check if we moved backwards in time (e.g. when playing a bag file)
  auto now{ ros::Time::now() };
  if (this->last_callback_time_ > now)
  {
    // force re-publish of joint transforms
    ROS_WARN("Moved backwards in time probably because ROS clock was reset), re-publishing joint transforms!");
    this->last_update_time_.clear();
  }
  ros::Duration warning_threshold{ 30.0 };
  if ((state->header.stamp + warning_threshold) < now)
  {
    ROS_WARN_THROTTLE(10, "Received JointState is %f seconds old.", (now - state->header.stamp).toSec());
  }
  this->last_callback_time_ = now;

  // determine least recently published joint
  auto last_published{ now };
  for (unsigned int i{ 0 }; i < state->name.size(); i++)
  {
    auto t{ this->last_update_time_[state->name[i]] };
    last_published = (t < last_published) ? t : last_published;
  }
  // note: if a joint was seen for the first time,
  //       then last_published is zero.

  // check if we need to publish
  auto next_publishing_time{ last_published + this->publish_interval_ };
  if (this->ignore_timestamp_ || state->header.stamp >= next_publishing_time)
  {
    // get joint positions from state message
    this->joint_positions_.clear();
    for (unsigned int i{ 0 }; i < state->name.size(); i++)
    {
      this->joint_positions_.emplace(std::make_pair(state->name[i], state->position[i]));
    }

    for (const auto& [name, mimic] : mimic_)
    {
      if (this->joint_positions_.find(mimic->joint_name) != this->joint_positions_.end())
      {
        double pos{ this->joint_positions_[mimic->joint_name] * mimic->multiplier + mimic->offset };
        this->joint_positions_.insert(std::make_pair(name, pos));
      }
    }

    for (const auto& name : state->name)
    {
      this->last_update_time_[name] = state->header.stamp;
    }
  }
}

void JointStateListener::getTransforms(const ros::Time& time, std::vector<geometry_msgs::TransformStamped>* destination)
{
  if (use_tf_static_)
  {
    state_publisher_.getFixedTransforms(this->tf_prefix_, time, destination);
  }
  else
  {
    state_publisher_.getTransforms(this->joint_positions_, this->tf_prefix_, time, destination);
  }
}
}  // namespace multi_robot_state_publisher
