
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
//! \brief Computes robot frame data to publish to the TF tree.
class RobotStatePublisher
{
  class SegmentPair
  {
  public:
    //! \brief Construct a new SegmentPair.
    //!
    //! \param p_segment A kinematic segment connecting p_root to p_tip.
    //! \param p_root The root of p_segment.
    //! \param p_tip The tip of p_segment.
    inline SegmentPair(const KDL::Segment& p_segment, const std::string& p_root, const std::string& p_tip)
      : segment{ p_segment }, root{ p_root }, tip{ p_tip }
    {
    }

    //! \brief An associated segment.
    KDL::Segment segment;

    //! \brief The root frame of the segment.
    std::string root;

    //! \brief The tip frame of the segment.
    std::string tip;
  };

public:
  //! \brief Construct a new RobotStatePublisher.
  //!
  //! \param model A model of a robot to publish.
  explicit RobotStatePublisher(urdf::Model* model);

  //! \brief Destroy the RobotStatePublisher.
  ~RobotStatePublisher(){};

  //! \brief Update destination with fixed transforms.
  //!
  //! \param tf_prefix A prefix for the /tf topic on the rosgraph.
  //! \param time The time to set for the returned transforms.
  //! \param destination A location to place the resultant transforms.
  void getFixedTransforms(const std::string& tf_prefix, const ros::Time& time,
                          std::vector<geometry_msgs::TransformStamped>* destination) const;

  //! \brief Update destination with all  transforms.
  //!
  //! \param joint_positions Joint positions on the robot for non-static transforms.
  //! \param tf_prefix A prefix for the /tf topic on the rosgraph.
  //! \param time The time to set for the returned transforms.
  //! \param destination A location to place the resultant transforms.
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
