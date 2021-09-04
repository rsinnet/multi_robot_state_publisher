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

#ifndef MULTI_ROBOT_STATE_PUBLISHER_JOINT_STATE_LISTENER_H
#define MULTI_ROBOT_STATE_PUBLISHER_JOINT_STATE_LISTENER_H

#include <map>
#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>

#include <geometry_msgs/TransformStamped.h>
#include <kdl/tree.hpp>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <urdf/model.h>

#include "multi_robot_state_publisher/robot_state_publisher.h"

namespace multi_robot_state_publisher
{
typedef boost::shared_ptr<sensor_msgs::JointState const> JointStateConstPtr;
typedef std::map<std::string, urdf::JointMimicSharedPtr> MimicMap;

//! \brief Listens to joint states for a robot to compute frames.
class JointStateListener
{
public:
  //! \brief Construct a new JointStateListener.
  //!
  //! \param model Model of the robot.
  //! \param nh Handle to the public namespace.
  //! \param np Handle to the node's private namespace.
  JointStateListener(urdf::Model model, ros::NodeHandle nh = {}, ros::NodeHandle np = { "~" });

  //! \brief Construct a new JointStateListener.
  //!
  //! \param nh Handle to the public namespace.
  //! \param np Handle to the node's private namespace.
  explicit JointStateListener(ros::NodeHandle nh = {}, ros::NodeHandle np = { "~" });

  //! \brief Destroy the JointStateListener.
  ~JointStateListener();

  //! \brief Return the associated RobotStatePublisher.
  //!
  //! \return The underlying RobotStatePublisher.
  inline constexpr const RobotStatePublisher& getRobotStatePublisher() const
  {
    return this->state_publisher_;
  }

  //! \brief Return true if configured to use tf_static.
  //!
  //! \brief True if configured to use tf_static.
  [[nodiscard]] inline bool isStatic() const
  {
    return use_tf_static_;
  }

  //! \brief Update destination with calculated transforms.
  //!
  //! \param time The time at which the transforms are valid.
  //! \param destination A place to store the calculated transforms.
  void getTransforms(const ros::Time& time, std::vector<geometry_msgs::TransformStamped>* destination);

protected:
  virtual void callbackJointState(const JointStateConstPtr& state);

  ros::NodeHandle nh_;
  ros::NodeHandle np_;
  urdf::Model model_;
  MimicMap mimic_;

  std::string tf_prefix_;
  ros::Duration publish_interval_;
  RobotStatePublisher state_publisher_;
  ros::Subscriber joint_state_sub_;
  ros::Time last_callback_time_;
  std::map<std::string, ros::Time> last_update_time_;
  bool use_tf_static_;
  bool ignore_timestamp_;
  bool tcp_nodelay_;

private:
  std::map<std::string, double> joint_positions_;

  [[nodiscard]] static urdf::Model loadModelFromROS(const ros::NodeHandle& np);
  [[nodiscard]] static MimicMap buildMimicMap(const urdf::Model& model);
};
}  // namespace multi_robot_state_publisher

#endif  // MULTI_ROBOT_STATE_PUBLISHER_JOINT_STATE_LISTENER_H
