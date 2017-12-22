/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
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
 *   * Neither the name of the author nor the names of its
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

/* Author: Marcus Ebner */

/*
a1: -0.753815352917
a2: 1.1734058857
a3: 0.112757593393
a4: -1.68315970898
a5: -0.736448764801
a6: -1.34951746464
a7: 0.104109719396
*/


#include <iimoveit/robot_interface.h>


namespace hanoi {

class HanoiGripper : public iimoveit::RobotInterface {
public:
  HanoiGripper(ros::NodeHandle* node_handle, const std::string& planning_group, const std::string& base_frame)
      :  RobotInterface(node_handle, planning_group, base_frame),
         base_pose_jointspace_{-0.753815352917, 1.1734058857, 0.112757593393, -1.68315970898, -0.736448764801, -1.34951746464, 0.104109719396} {
    // Get endeffector pose from joint space goal
    robot_state_.setJointGroupPositions(joint_model_group_, base_pose_jointspace_);
    const Eigen::Affine3d& end_effector_state = robot_state_.getGlobalLinkTransform(move_group_.getEndEffectorLink());
    Eigen::Vector3d t(end_effector_state.translation());
    Eigen::Quaterniond q(end_effector_state.rotation());
    base_pose_.position.x = t[0];
    base_pose_.position.y = t[1];
    base_pose_.position.z = t[2];
    base_pose_.orientation.x = q.x();
    base_pose_.orientation.y = q.y();
    base_pose_.orientation.z = q.z();
    base_pose_.orientation.w = q.w();

    // Reset robot state to current state
    updateRobotState();
  }

  void moveToBasePose() {
    planAndMove(base_pose_jointspace_, std::string("base_pose_jointspace"), true);
  }

private:
  std::vector<double> base_pose_jointspace_;
  geometry_msgs::Pose base_pose_;
};
} // namespace hanoi

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_pose_follower");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  hanoi::HanoiGripper hanoi_gripper(&node_handle, "manipulator", "world");
  hanoi_gripper.moveToBasePose();

  ros::Rate rate(10);
  while(ros::ok()) {
    rate.sleep();
  }
  ros::shutdown();
  return 0;
}