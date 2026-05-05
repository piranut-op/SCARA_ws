#include <memory>
#include <thread>
#include <vector>
#include <map>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <visualization_msgs/msg/marker_array.hpp>

using moveit::planning_interface::MoveGroupInterface;
using JointMap = std::map<std::string, double>;

struct Step {
  std::string label;
  JointMap joints;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "scara_pick_and_place",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  auto const logger = rclcpp::get_logger("scara_pick_and_place");

  MoveGroupInterface move_group(node, "arm");
  move_group.setEndEffectorLink("Link_ee");
  move_group.setPoseReferenceFrame("base_link");
  move_group.setGoalJointTolerance(0.01);
  move_group.allowReplanning(true);
  move_group.setNumPlanningAttempts(5);
  move_group.setPlanningTime(5.0);
  move_group.setMaxVelocityScalingFactor(0.4);
  move_group.setMaxAccelerationScalingFactor(0.4);

  auto marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/scara_pick_place_markers", rclcpp::QoS(1).transient_local());

  // Joint-space waypoints — guaranteed reachable, no IK in the loop.
  // ee_joint URDF limits: [-0.045, 0.065] m → 110 mm total travel.
  // Use the bulk of the down-stroke for a practical pick/place dip while
  // keeping a safety margin to the URDF upper limit.
  double const ee_up = 0.0;
  double const ee_dn = 0.060;  // 60 mm downward stroke (5 mm under URDF max)

  // Pick configuration: front-right of workspace.
  double const pick_j1 =  0.8;
  double const pick_j2 =  1.0;

  // Place configuration: back-left, opposite quadrant. Going from pick to
  // place rotates Link_1_joint by 1.6 rad and Link_2_joint by 2.0 rad.
  double const place_j1 = -0.8;
  double const place_j2 = -1.0;

  auto J = [&](double j1, double j2, double ee) -> JointMap {
    return {{"Link_1_joint", j1}, {"Link_2_joint", j2}, {"ee_joint", ee}};
  };

  // Sequence: from initial pose, move arm to pick (ee stays at rest),
  // descend 35 mm, ascend 35 mm, swing arm to place, descend, ascend, home.
  std::vector<Step> sequence = {
    {"approach_pick", J(pick_j1,  pick_j2,  ee_up)},
    {"engage_pick",   J(pick_j1,  pick_j2,  ee_dn)},
    {"retreat_pick",  J(pick_j1,  pick_j2,  ee_up)},
    {"approach_place",J(place_j1, place_j2, ee_up)},
    {"engage_place",  J(place_j1, place_j2, ee_dn)},
    {"retreat_place", J(place_j1, place_j2, ee_up)},
    {"home_return",   J(0.0,      0.0,      ee_up)},
  };

  // ── Run the sequence ─────────────────────────────────────────────────────
  size_t reached = 0;
  for (auto const & step : sequence)
  {
    RCLCPP_INFO(logger, "[%s] j1=%.2f j2=%.2f ee=%.3f",
                step.label.c_str(),
                step.joints.at("Link_1_joint"),
                step.joints.at("Link_2_joint"),
                step.joints.at("ee_joint"));

    move_group.setStartStateToCurrentState();
    move_group.clearPoseTargets();
    move_group.setJointValueTarget(step.joints);

    MoveGroupInterface::Plan plan;
    if (!static_cast<bool>(move_group.plan(plan)))
    {
      RCLCPP_ERROR(logger, "[%s] plan failed.", step.label.c_str());
      rclcpp::shutdown(); spinner.join(); return 1;
    }
    move_group.execute(plan);
    ++reached;

    // Publish a marker at the resulting EE position so RViz shows where
    // pick / place actually landed.
    if (step.label == "engage_pick" || step.label == "engage_place")
    {
      rclcpp::sleep_for(std::chrono::milliseconds(200));
      auto p = move_group.getCurrentPose("Link_ee").pose;
      visualization_msgs::msg::MarkerArray ma;
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "base_link";
      m.header.stamp = node->now();
      m.ns = step.label;
      m.id = 0;
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose = p;
      // Place the cube on the world floor (z=0 relative to base_link),
      // sitting on top of a 25 mm cube, instead of floating at the EE.
      m.pose.position.z = 0.0125;
      m.pose.orientation.x = 0.0;
      m.pose.orientation.y = 0.0;
      m.pose.orientation.z = 0.0;
      m.pose.orientation.w = 1.0;
      m.scale.x = m.scale.y = m.scale.z = 0.025;
      if (step.label == "engage_pick") {
        m.color.r = 0.0f; m.color.g = 1.0f; m.color.b = 0.0f;
      } else {
        m.color.r = 1.0f; m.color.g = 0.5f; m.color.b = 0.0f;
      }
      m.color.a = 1.0f;
      ma.markers.push_back(m);
      marker_pub->publish(ma);
      RCLCPP_INFO(logger, "  -> EE landed at x=%.3f y=%.3f z=%.3f",
                  p.position.x, p.position.y, p.position.z);
    }

    // Dwell 1 second after the EE is engaged at pick or place; just a
    // short settle delay between every other step.
    auto const dwell = (step.label == "engage_pick" ||
                        step.label == "engage_place")
                       ? std::chrono::milliseconds(1000)
                       : std::chrono::milliseconds(200);
    if (step.label == "engage_pick" || step.label == "engage_place")
      RCLCPP_INFO(logger, "  -> dwell 1.0 s before retreat");
    rclcpp::sleep_for(dwell);
  }

  RCLCPP_INFO(logger, "Pick-and-place complete (%zu/%zu steps).",
              reached, sequence.size());
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
