// Camera-driven SCARA pick-and-place using MoveIt.
//
// Subscribes to /bottle_cap/detections (geometry_msgs/PointStamped in the
// camera optical frame, expressed in metres) published by
// SCARA_pkg/detect_bottle_cap.py — the detector deprojects the locked
// pixel using real RealSense intrinsics + measured depth.
//
// We then transform that point into base_link via tf2 (the launch file
// publishes a static TF from base_link → camera_color_optical_frame), and
// use MoveIt's KDL solver via setPoseTarget on the "arm" group (tip =
// Link_ee).
//
// The arm group spans base_link -> Link_ee, so the same pose target drives
// both the planar joints (Link_1_joint, Link_2_joint) AND the prismatic
// ee_joint via the chain — descending = lower z on the same pose, lifting
// = raise it.

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using moveit::planning_interface::MoveGroupInterface;
using JointMap = std::map<std::string, double>;


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("scara_cam_pick_and_place");

  // ── Parameters ─────────────────────────────────────────────────────────
  auto declare_d = [&](const std::string & n, double dflt) {
    node->declare_parameter<double>(n, dflt);
    return node->get_parameter(n).as_double();
  };

  // Cartesian z of Link_ee in base_link frame (metres). Tune to your URDF.
  double z_travel_m             = declare_d("z_travel_m",        0.10);
  double z_pick_m               = declare_d("z_pick_m",          0.04);

  double place_x_m              = declare_d("place_x_m",         0.10);
  double place_y_m              = declare_d("place_y_m",         0.25);

  double cooldown_s             = declare_d("cooldown_s",        2.0);
  double dwell_engage_s         = declare_d("dwell_engage_s",    1.0);

  // ── Spin executor on a side thread (MoveGroupInterface needs spinning) ──
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  auto const logger = rclcpp::get_logger("scara_cam_pick_and_place");

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
    "/scara_cam_pick_place_markers", rclcpp::QoS(1).transient_local());

  // ── tf2 buffer + listener for camera_optical_frame → base_link ────────
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf2_ros::TransformListener tf_listener(*tf_buffer);

  // ── Target intake ──────────────────────────────────────────────────────
  // The detector publishes a 3-D point in the camera optical frame
  // (deprojected from pixel + measured depth). We just tf2-transform to
  // base_link and hand the (x, y) to the planner; z is overridden by the
  // travel/pick heights.
  enum class Phase { IDLE, BUSY };
  std::atomic<Phase> phase{Phase::IDLE};

  std::optional<std::pair<double, double>> ready_target;
  std::mutex ready_mu;
  std::condition_variable ready_cv;

  auto sub = node->create_subscription<geometry_msgs::msg::PointStamped>(
    "/bottle_cap/detections", 10,
    [&](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
      if (phase.load() != Phase::IDLE) return;
      geometry_msgs::msg::PointStamped in_base;
      try {
        in_base = tf_buffer->transform(*msg, "base_link",
                                        tf2::durationFromSec(0.5));
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(logger, "TF transform failed: %s", ex.what());
        return;
      }
      RCLCPP_INFO(logger,
        "%s=(%.3f, %.3f, %.3f) → base_link=(%.3f, %.3f, %.3f) m",
        msg->header.frame_id.c_str(),
        msg->point.x, msg->point.y, msg->point.z,
        in_base.point.x, in_base.point.y, in_base.point.z);
      {
        std::lock_guard<std::mutex> rl(ready_mu);
        ready_target = std::make_pair(in_base.point.x, in_base.point.y);
      }
      ready_cv.notify_one();
    });

  // ── Helpers ────────────────────────────────────────────────────────────
  auto pose_at = [&](double x, double y, double z) {
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = "base_link";
    p.pose.position.x = x;
    p.pose.position.y = y;
    p.pose.position.z = z;
    p.pose.orientation.w = 1.0;
    return p;
  };

  auto plan_and_execute_pose = [&](const std::string & label,
                                    double x, double y, double z) -> bool {
    RCLCPP_INFO(logger, "[%s] target xyz=(%.3f, %.3f, %.3f)",
                label.c_str(), x, y, z);
    move_group.setStartStateToCurrentState();
    move_group.clearPoseTargets();
    move_group.setPoseTarget(pose_at(x, y, z), "Link_ee");
    MoveGroupInterface::Plan plan;
    if (!static_cast<bool>(move_group.plan(plan))) {
      RCLCPP_ERROR(logger, "[%s] plan failed.", label.c_str());
      return false;
    }
    move_group.execute(plan);
    return true;
  };

  auto plan_and_execute_joints = [&](const std::string & label,
                                      const JointMap & joints) -> bool {
    std::ostringstream os;
    for (auto & kv : joints) os << kv.first << "=" << kv.second << " ";
    RCLCPP_INFO(logger, "[%s] joints: %s", label.c_str(), os.str().c_str());
    move_group.setStartStateToCurrentState();
    move_group.clearPoseTargets();
    move_group.setJointValueTarget(joints);
    MoveGroupInterface::Plan plan;
    if (!static_cast<bool>(move_group.plan(plan))) {
      RCLCPP_ERROR(logger, "[%s] plan failed.", label.c_str());
      return false;
    }
    move_group.execute(plan);
    return true;
  };

  auto publish_marker = [&](const std::string & ns, double r, double g, double b) {
    auto p = move_group.getCurrentPose("Link_ee").pose;
    visualization_msgs::msg::MarkerArray ma;
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "base_link";
    m.header.stamp = node->now();
    m.ns = ns;
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose = p;
    m.pose.position.z = 0.0125;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 0.025;
    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1.0f;
    ma.markers.push_back(m);
    marker_pub->publish(ma);
  };

  RCLCPP_INFO(logger,
    "scara_cam_pick_and_place ready  |  z_travel=%.3f  z_pick=%.3f  "
    "place=(%.3f, %.3f)  |  expecting TF base_link → camera_color_optical_frame",
    z_travel_m, z_pick_m, place_x_m, place_y_m);

  // ── Main loop ──────────────────────────────────────────────────────────
  while (rclcpp::ok()) {
    std::pair<double, double> target;
    {
      std::unique_lock<std::mutex> lk(ready_mu);
      ready_cv.wait(lk, [&] { return ready_target.has_value() || !rclcpp::ok(); });
      if (!rclcpp::ok()) break;
      target = *ready_target;
      ready_target.reset();
    }
    phase.store(Phase::BUSY);

    RCLCPP_INFO(logger, "cap stable at (%.3f, %.3f) m — starting pick.",
                target.first, target.second);

    bool ok = true;
    ok = ok && plan_and_execute_pose("approach_pick",
                                      target.first, target.second, z_travel_m);
    ok = ok && plan_and_execute_pose("engage_pick",
                                      target.first, target.second, z_pick_m);
    if (ok) {
      publish_marker("engage_pick", 0.0f, 1.0f, 0.0f);
      rclcpp::sleep_for(std::chrono::milliseconds(
        static_cast<int>(dwell_engage_s * 1000)));
    }
    ok = ok && plan_and_execute_pose("retreat_pick",
                                      target.first, target.second, z_travel_m);

    ok = ok && plan_and_execute_pose("approach_place",
                                      place_x_m, place_y_m, z_travel_m);
    ok = ok && plan_and_execute_pose("engage_place",
                                      place_x_m, place_y_m, z_pick_m);
    if (ok) {
      publish_marker("engage_place", 1.0f, 0.5f, 0.0f);
      rclcpp::sleep_for(std::chrono::milliseconds(
        static_cast<int>(dwell_engage_s * 1000)));
    }
    ok = ok && plan_and_execute_pose("retreat_place",
                                      place_x_m, place_y_m, z_travel_m);

    ok = ok && plan_and_execute_joints("home_return",
      {{"Link_1_joint", 0.0}, {"Link_2_joint", 0.0}, {"ee_joint", 0.0}});

    RCLCPP_INFO(logger, "cycle complete (ok=%d) — cooldown %.1fs",
                static_cast<int>(ok), cooldown_s);
    rclcpp::sleep_for(std::chrono::milliseconds(
      static_cast<int>(cooldown_s * 1000)));

    phase.store(Phase::IDLE);
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
