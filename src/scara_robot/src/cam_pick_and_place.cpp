// Camera-driven SCARA pick-and-place using MoveIt.
//
// Mirrors src/pick_and_place.cpp but takes the pick (x, y) from the
// /bottle_cap/workspace_position topic published by the YOLO bottle-cap
// detector in SCARA_pkg/detect_bottle_cap.py, and uses MoveIt's KDL solver
// (via setPoseTarget on the "arm" group, tip = Link_ee) instead of the
// custom analytical IK in SCARA_pkg/ikpos.py.
//
// The arm group spans base_link -> Link_ee, so the same pose target drives
// both the planar joints (Link_1_joint, Link_2_joint) AND the prismatic
// ee_joint via the chain — descending = lower z on the same pose, lifting
// = raise it.

#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

using moveit::planning_interface::MoveGroupInterface;
using JointMap = std::map<std::string, double>;

namespace {

// ── Minimal JSON helpers (the detector publishes std_msgs/String JSON). ──
// We don't pull in a full JSON dep; we only need a few numeric fields per
// detection. This is fragile by design — keep it in lock-step with the
// localizer's output schema.
struct Detection {
  std::string cls;
  double confidence{0.0};
  double robot_x_cm{0.0};
  double robot_y_cm{0.0};
};

bool find_number(const std::string & s, const std::string & key, double & out) {
  auto k = s.find("\"" + key + "\":");
  if (k == std::string::npos) return false;
  k += key.size() + 3;
  while (k < s.size() && (s[k] == ' ' || s[k] == ':')) ++k;
  std::size_t end = k;
  while (end < s.size() && (std::isdigit(s[end]) || s[end] == '.' ||
                            s[end] == '-' || s[end] == '+' || s[end] == 'e'))
    ++end;
  if (end == k) return false;
  try { out = std::stod(s.substr(k, end - k)); }
  catch (...) { return false; }
  return true;
}

bool find_class(const std::string & s, std::string & out) {
  auto k = s.find("\"class\":");
  if (k == std::string::npos) return false;
  k = s.find('"', k + 8);
  if (k == std::string::npos) return false;
  auto e = s.find('"', k + 1);
  if (e == std::string::npos) return false;
  out = s.substr(k + 1, e - k - 1);
  return true;
}

// Crude split on top-level "},{" — assumes no nested braces in detection
// objects beyond what the localizer emits today (intrinsics is a flat dict
// nested inside, so we instead split on `}, {` only at depth 0).
std::vector<std::string> split_detections(const std::string & json_array) {
  std::vector<std::string> out;
  int depth = 0;
  std::size_t start = std::string::npos;
  for (std::size_t i = 0; i < json_array.size(); ++i) {
    char c = json_array[i];
    if (c == '{') {
      if (depth == 0) start = i;
      ++depth;
    } else if (c == '}') {
      --depth;
      if (depth == 0 && start != std::string::npos) {
        out.emplace_back(json_array.substr(start, i - start + 1));
        start = std::string::npos;
      }
    }
  }
  return out;
}

std::vector<Detection> parse_localized_json(const std::string & s) {
  std::vector<Detection> out;
  for (const auto & blob : split_detections(s)) {
    Detection d;
    if (!find_class(blob, d.cls)) continue;
    find_number(blob, "confidence", d.confidence);
    // robot.x_cm / robot.y_cm appear inside a nested {"robot": {...}} dict.
    // Our key search matches the inner keys directly because we don't
    // descend into nested dicts — but x_cm / y_cm appear *only* under
    // "robot" in the localizer schema, so this is safe.
    find_number(blob, "x_cm", d.robot_x_cm);
    find_number(blob, "y_cm", d.robot_y_cm);
    out.push_back(d);
  }
  return out;
}

}  // namespace


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "scara_cam_pick_and_place",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // ── Parameters ─────────────────────────────────────────────────────────
  auto declare_d = [&](const std::string & n, double dflt) {
    node->declare_parameter<double>(n, dflt);
    return node->get_parameter(n).as_double();
  };
  auto declare_i = [&](const std::string & n, int64_t dflt) {
    node->declare_parameter<int>(n, static_cast<int>(dflt));
    return node->get_parameter(n).as_int();
  };
  auto declare_s = [&](const std::string & n, const std::string & dflt) {
    node->declare_parameter<std::string>(n, dflt);
    return node->get_parameter(n).as_string();
  };

  std::string class_filter      = declare_s("class_filter",      "bottle_cap");
  double      min_confidence    = declare_d("min_confidence",    0.50);

  // workspace-centre offset in SCARA base frame (metres)
  double base_off_x             = declare_d("base_to_workspace_x_m", 0.0);
  double base_off_y             = declare_d("base_to_workspace_y_m", 0.20);

  int    stable_frames          = declare_i("stable_frames",     8);
  double stable_deadband_m      = declare_d("stable_deadband_m", 0.01);

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

  // ── Stability tracker (only consumed in IDLE) ──────────────────────────
  enum class Phase { IDLE, BUSY };
  std::atomic<Phase> phase{Phase::IDLE};

  std::mutex stable_mu;
  std::optional<std::pair<double, double>> stable_xy;
  int stable_count = 0;

  std::optional<std::pair<double, double>> ready_target;  // set when stable
  std::mutex ready_mu;
  std::condition_variable ready_cv;

  auto sub = node->create_subscription<std_msgs::msg::String>(
    "/bottle_cap/workspace_position", 10,
    [&](const std_msgs::msg::String::SharedPtr msg) {
      if (phase.load() != Phase::IDLE) return;
      auto dets = parse_localized_json(msg->data);

      // Best by confidence, with class/conf filter.
      const Detection * best = nullptr;
      for (auto & d : dets) {
        if (!class_filter.empty() && d.cls != class_filter) continue;
        if (d.confidence < min_confidence) continue;
        if (!best || d.confidence > best->confidence) best = &d;
      }
      std::lock_guard<std::mutex> lock(stable_mu);
      if (!best) {
        stable_xy.reset();
        stable_count = 0;
        return;
      }

      double x = best->robot_x_cm / 100.0 + base_off_x;
      double y = best->robot_y_cm / 100.0 + base_off_y;

      if (!stable_xy.has_value()) {
        stable_xy = std::make_pair(x, y);
        stable_count = 1;
        return;
      }
      double dx = x - stable_xy->first;
      double dy = y - stable_xy->second;
      if (std::sqrt(dx * dx + dy * dy) <= stable_deadband_m) {
        stable_count += 1;
        stable_xy = std::make_pair(
          0.5 * (stable_xy->first  + x),
          0.5 * (stable_xy->second + y));
      } else {
        stable_xy = std::make_pair(x, y);
        stable_count = 1;
      }

      if (stable_count >= stable_frames) {
        {
          std::lock_guard<std::mutex> rl(ready_mu);
          ready_target = stable_xy;
        }
        stable_xy.reset();
        stable_count = 0;
        ready_cv.notify_one();
      }
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
    "scara_cam_pick_and_place ready  |  class='%s'  "
    "offset=(%.3f, %.3f) m  z_travel=%.3f  z_pick=%.3f  "
    "place=(%.3f, %.3f)",
    class_filter.c_str(), base_off_x, base_off_y,
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
