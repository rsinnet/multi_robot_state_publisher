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

#include "multi_robot_state_publisher/robot_state_publisher.h"

#include <iterator>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/TransformStamped.h>
#include <kdl/frames_io.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <ros/ros.h>
#include <tf2_kdl/tf2_kdl.h>

namespace multi_robot_state_publisher
{
RobotStatePublisher::RobotStatePublisher(urdf::Model* model) : model_{ *model }
{
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(this->model_, tree))
  {
    ROS_ERROR("Failed to extract KDL Tree from URDF.");
    throw std::invalid_argument{ "model" };
  }
  // walk the tree and add segments to segments_
  this->addChildren(tree.getRootSegment());
}

// add children to correct maps
void RobotStatePublisher::addChildren(const KDL::SegmentMap::const_iterator segment)
{
  const auto& root{ GetTreeElementSegment(segment->second).getName() };

  const auto& children{ GetTreeElementChildren(segment->second) };
  for (unsigned int i{ 0 }; i < children.size(); i++)
  {
    const auto& child{ GetTreeElementSegment(children[i]->second) };
    SegmentPair s{ GetTreeElementSegment(children[i]->second), root, child.getName() };
    if (child.getJoint().getType() == KDL::Joint::None)
    {
      if (model_.getJoint(child.getJoint().getName()) &&
          model_.getJoint(child.getJoint().getName())->type == urdf::Joint::FLOATING)
      {
        ROS_INFO("Floating joint. Not adding segment from %s to %s. This TF can not be published based on joint_states "
                 "info",
                 root.c_str(), child.getName().c_str());
      }
      else
      {
        segments_fixed_.insert(std::make_pair(child.getJoint().getName(), s));
        ROS_DEBUG("Adding fixed segment from %s to %s", root.c_str(), child.getName().c_str());
      }
    }
    else
    {
      segments_.insert(std::make_pair(child.getJoint().getName(), s));
      ROS_DEBUG("Adding moving segment from %s to %s", root.c_str(), child.getName().c_str());
    }
    addChildren(children[i]);
  }
}

geometry_msgs::TransformStamped RobotStatePublisher::getTransform(const RobotStatePublisher::SegmentPair& pair,
                                                                  const KDL::Frame& frame, const std::string& tf_prefix,
                                                                  const ros::Time& time)
{
  auto tf_transform{ tf2::kdlToTransform(frame) };
  tf_transform.header.stamp = time;
  tf_transform.header.frame_id = tf::resolve(tf_prefix, pair.root);
  tf_transform.child_frame_id = tf::resolve(tf_prefix, pair.tip);
  return tf_transform;
}

void RobotStatePublisher::getTransforms(const std::map<std::string, double>& joint_positions,
                                        const std::string& tf_prefix, const ros::Time& time,
                                        std::vector<geometry_msgs::TransformStamped>* destination)
{
  for (const auto& [name, position] : joint_positions)
  {
    if (auto seg{ segments_.find(name) }; seg != segments_.end())
    {
      destination->emplace_back(getTransform(seg->second, seg->second.segment.pose(position), tf_prefix, time));
    }
    else
    {
      ROS_WARN_THROTTLE(10, "Joint state with name: \"%s\" was received but not found in URDF", name.c_str());
    }
  }
  this->getFixedTransforms(tf_prefix, time, destination);
}

void RobotStatePublisher::getFixedTransforms(const std::string& tf_prefix, const ros::Time& time,
                                             std::vector<geometry_msgs::TransformStamped>* destination) const
{
  for (const auto& [name, segment] : this->segments_fixed_)
  {
    destination->emplace_back(getTransform(segment, segment.segment.pose(0), tf_prefix, time));
  }
}
}  // namespace multi_robot_state_publisher
