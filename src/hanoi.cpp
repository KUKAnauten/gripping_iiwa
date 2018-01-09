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
#include <tf/LinearMath/Quaternion.h>


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

  void moveToBaseRelativePose(const geometry_msgs::Pose& relativePose, bool approvalRequired) {
    tf::Quaternion base_quaternion(base_pose_.orientation.x, base_pose_.orientation.y, base_pose_.orientation.z, base_pose_.orientation.w);
    tf::Quaternion next_quaternion(relativePose.orientation.x, relativePose.orientation.y, relativePose.orientation.z, relativePose.orientation.w);
    tf::Quaternion result_quaternion = next_quaternion * base_quaternion;
    result_quaternion.normalize();

    geometry_msgs::Pose target_pose = base_pose_;
    target_pose.position.x += relativePose.position.x;
    target_pose.position.y += relativePose.position.y;
    target_pose.position.z += relativePose.position.z;
    target_pose.orientation.x = result_quaternion.getX();
    target_pose.orientation.y = result_quaternion.getY();
    target_pose.orientation.z = result_quaternion.getZ();
    target_pose.orientation.w = result_quaternion.getW();

    planAndMove(target_pose, std::string("relative pose"), approvalRequired);
  }

  void moveToBaseRelativePosition(const geometry_msgs::Point relativePosition, bool approvalRequired) {
    geometry_msgs::Pose target_pose = base_pose_;
    target_pose.position.x += relativePosition.x;
    target_pose.position.y += relativePosition.y;
    target_pose.position.z += relativePosition.z;

    planAndMove(target_pose, std::string("relative pose"), approvalRequired);
  }

  void moveToBaseRelativePosition(double x, double y, double z, bool approvalRequired) {
    geometry_msgs::Pose target_pose = base_pose_;
    target_pose.position.x += x;
    target_pose.position.y += y;
    target_pose.position.z += z;

    planAndMove(target_pose, std::string("relative pose"), approvalRequired);
  }

  void buttonEventCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Der Schmartie funktioniert: %s", msg->data.c_str());
    if(msg->data == "pose_get_pressed") {
      geometry_msgs::PoseStamped current_pose = getPose();
      ROS_INFO("Current Position = (%f, %f, %f)", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
    }
  }

  // These might come from iimoveit or a similar package
  void openGripper() {}
  void closeGripper() {}

  void grabFromLeftTower() {}
  void putToLeftTower() {}
  void grabFromCenterTower() {}
  void putToCenterTower() {}
  void grabFromRightTower() {}
  void putToRightTower(){}

private:
  std::vector<double> base_pose_jointspace_;
  geometry_msgs::Pose base_pose_;
  geometry_msgs::Pose left_tower_relPose_;
  geometry_msgs::Pose right_tower_relPose_;
  geometry_msgs::Pose center_tower_relPose_;
  double slice_height_;
  int nslices_left_;
  int nslices_center_;
  int nslices_right_;
  bool grapping_;
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
  double sdl = 0.15;
  hanoi_gripper.moveToBaseRelativePosition(0.0, sdl, 0.0, true);
  hanoi_gripper.moveToBaseRelativePosition(sdl, sdl, 0.0, false);
  hanoi_gripper.moveToBaseRelativePosition(sdl, 0.0, 0.0, false);
  hanoi_gripper.moveToBaseRelativePosition(sdl, 0.0, sdl, false);
  hanoi_gripper.moveToBaseRelativePosition(0.0, 0.0, sdl, false);
  hanoi_gripper.moveToBaseRelativePosition(0.0, sdl, sdl, false);
  hanoi_gripper.moveToBaseRelativePosition(sdl, sdl, sdl, false);
  hanoi_gripper.moveToBaseRelativePosition(0.0, 0.0, 0.0, false);

  /*
  ros::Rate rate(10);
  while(ros::ok()) {
    rate.sleep();
  }
  */
  ros::shutdown();
  return 0;
}
