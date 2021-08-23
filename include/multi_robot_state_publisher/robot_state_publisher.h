
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

#ifndef MULTI_ROBOT_STATE_PUBLISHER_ROBOT_STATE_PUBLISHER_H
#define MULTI_ROBOT_STATE_PUBLISHER_ROBOT_STATE_PUBLISHER_H

#include <iterator>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <boost/scoped_ptr.hpp>

#include <geometry_msgs/TransformStamped.h>
#include <kdl/frames.hpp>
#include <kdl/segment.hpp>
#include <kdl/tree.hpp>
#include <ros/ros.h>
#include <tf/tf.h>
#include <urdf/model.h>
namespace multi_robot_state_publisher
{
class RobotStatePublisher
{
  class SegmentPair
  {
  public:
    inline SegmentPair(const KDL::Segment& p_segment, const std::string& p_root, const std::string& p_tip)
      : segment{ p_segment }, root{ p_root }, tip{ p_tip }
    {
    }

    KDL::Segment segment;
    std::string root, tip;
  };

public:
  explicit RobotStatePublisher(urdf::Model* model);

  ~RobotStatePublisher(){};

  void getFixedTransforms(const std::string& tf_prefix, const ros::Time& time,
                          std::vector<geometry_msgs::TransformStamped>* destination) const;

  void getTransforms(const std::map<std::string, double>& joint_positions, const std::string& tf_prefix,
                     const ros::Time& time, std::vector<geometry_msgs::TransformStamped>* destination);

protected:
  virtual void addChildren(const KDL::SegmentMap::const_iterator segment);

  std::map<std::string, SegmentPair> segments_, segments_fixed_;
  urdf::Model& model_;

private:
  // Reusable memory.
  mutable std::vector<geometry_msgs::TransformStamped> tf_transforms_;

  static geometry_msgs::TransformStamped getTransform(const SegmentPair& pair, const KDL::Frame& frame,
                                                      const std::string& tf_prefix, const ros::Time& time);
};
}  // namespace multi_robot_state_publisher
#endif  // MULTI_ROBOT_STATE_PUBLISHER_ROBOT_STATE_PUBLISHER_H
