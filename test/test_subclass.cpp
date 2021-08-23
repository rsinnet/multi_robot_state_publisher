/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Open Source Robotics Foundation, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 */

#include <map>
#include <memory>
#include <string>
#include <utility>

#include <kdl_parser/kdl_parser.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <urdf/model.h>

#include "multi_robot_state_publisher/joint_state_listener.h"
#include "multi_robot_state_publisher/robot_state_publisher.h"
#include <gtest/gtest.h>

using multi_robot_state_publisher::JointStateListener;
using multi_robot_state_publisher::MimicMap;

namespace multi_robot_state_publisher_test
{
class AccessibleJointStateListener : public JointStateListener
{
public:
  AccessibleJointStateListener(std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster,
                               std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster,
                               const KDL::Tree& tree, MimicMap mimic_map, urdf::Model model)
    : JointStateListener{ tf_broadcaster, static_tf_broadcaster, tree, std::move(mimic_map), std::move(model) }
  {
  }

  bool usingTfStatic() const
  {
    return use_tf_static_;
  }
};

class AccessibleRobotStatePublisher : public multi_robot_state_publisher::RobotStatePublisher
{
public:
  AccessibleRobotStatePublisher(std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster,
                                std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster,
                                const KDL::Tree& tree, urdf::Model model)
    : multi_robot_state_publisher::RobotStatePublisher{ tf_broadcaster, static_tf_broadcaster, std::move(tree),
                                                        std::move(model) }
  {
  }

  const urdf::Model& getModel() const
  {
    return model_;
  }
};
}  // namespace multi_robot_state_publisher_test

TEST(TestRobotStatePubSubclass, robot_state_pub_subclass)
{
  auto tf_broadcaster{ std::make_shared<tf2_ros::TransformBroadcaster>() };
  auto static_tf_broadcaster{ std::make_shared<tf2_ros::StaticTransformBroadcaster>() };
  urdf::Model model;
  model.initParam("robot_description");
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree))
  {
    ROS_ERROR("Failed to extract kdl tree from xml robot description");
    FAIL();
  }

  MimicMap mimic;

  for (const auto& [name, joint] : model.joints_)
  {
    if (joint->mimic)
    {
      mimic.insert(std::make_pair(name, joint->mimic));
    }
  }

  multi_robot_state_publisher_test::AccessibleRobotStatePublisher state_pub{ tf_broadcaster, static_tf_broadcaster,
                                                                             tree, model };
  EXPECT_EQ(model.name_, state_pub.getModel().name_);
  EXPECT_EQ(model.root_link_, state_pub.getModel().root_link_);

  multi_robot_state_publisher_test::AccessibleJointStateListener state_listener{ tf_broadcaster, static_tf_broadcaster,
                                                                                 tree, mimic, model };
  EXPECT_TRUE(state_listener.usingTfStatic());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_subclass");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
