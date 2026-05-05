#include <memory>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "scara_move",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto const logger = rclcpp::get_logger("scara_move");

  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface move_group(node, "arm");

  move_group.setEndEffectorLink("Link_ee");
  move_group.setPoseReferenceFrame("base_link");
  move_group.setGoalJointTolerance(0.01);
  move_group.allowReplanning(true);
  move_group.setNumPlanningAttempts(5);
  move_group.setPlanningTime(5.0);

  RCLCPP_INFO(logger, "Planning frame:  %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "EE link:         %s", move_group.getEndEffectorLink().c_str());
  RCLCPP_INFO(logger, "Joint names:");
  for (auto const & j : move_group.getJointNames())
    RCLCPP_INFO(logger, "  - %s", j.c_str());

  // SCARA "arm" has 2 revolute joints (Link_1_joint, Link_2_joint).
  // Joint-space target — guaranteed reachable, no IK ambiguity.
  std::map<std::string, double> joint_target = {
    {"Link_1_joint",  30.0 * M_PI / 180.0},
    {"Link_2_joint", -45.0 * M_PI / 180.0},
  };

  move_group.setStartStateToCurrentState();
  move_group.setJointValueTarget(joint_target);

  MoveGroupInterface::Plan plan;
  if (static_cast<bool>(move_group.plan(plan)))
  {
    RCLCPP_INFO(logger, "Plan succeeded — executing.");
    move_group.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Plan failed.");
    rclcpp::shutdown();
    return 1;
  }

  // Print resulting EE pose for reference.
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  auto const ee_pose = move_group.getCurrentPose("Link_ee").pose;
  RCLCPP_INFO(logger, "Link_ee pose: x=%.3f y=%.3f z=%.3f",
              ee_pose.position.x, ee_pose.position.y, ee_pose.position.z);

  rclcpp::shutdown();
  return 0;
}
