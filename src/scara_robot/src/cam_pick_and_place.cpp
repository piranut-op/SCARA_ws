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
#include <cmath>
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
#include <std_msgs/msg/int32.hpp>
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

  // Screw/unscrew motion at engage points: rotate ee CCW screw_revs, dwell,
  // rotate CW screw_revs back. Time budget per leg = screw_wait_s — must be
  // long enough at the current MKS speed_rpm to cover screw_revs revolutions.
  double screw_revs             = declare_d("screw_revs",        7.0);
  double screw_dwell_s          = declare_d("screw_dwell_s",     1.0);
  double screw_wait_s           = declare_d("screw_wait_s",      3.0);

  // Z of the workspace surface (where caps sit) in base_link, metres.
  // The detector publishes a unit-Z ray in the camera optical frame; we
  // transform that ray into base_link and intersect with this plane to
  // get the cap's (x, y). This bypasses RealSense depth entirely —
  // matters because matte-black workspaces give bad depth.
  double workspace_z_m          = declare_d("workspace_z_m",     0.0);

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

  // MKS relative-counts publisher (1 rev = 16384 counts; CCW = negative).
  auto ee_rel_pub = node->create_publisher<std_msgs::msg::Int32>(
    "/mks/ee_relative_counts", 10);
  constexpr int MKS_COUNTS_PER_REV = 16384;

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

      // The detector publishes a ray in the camera optical frame, expressed
      // as the point at Z=1 along that ray (X = (px-ppx)/fx, Y = (py-ppy)/fy,
      // Z = 1). Transform two points into base_link — the camera origin and
      // the ray endpoint — then intersect with z = workspace_z_m.
      geometry_msgs::msg::PointStamped origin_in_cam, end_in_cam;
      origin_in_cam.header = msg->header;
      origin_in_cam.point.x = 0.0;
      origin_in_cam.point.y = 0.0;
      origin_in_cam.point.z = 0.0;
      end_in_cam.header = msg->header;
      end_in_cam.point = msg->point;

      geometry_msgs::msg::PointStamped origin_b, end_b;
      try {
        origin_b = tf_buffer->transform(origin_in_cam, "base_link",
                                        tf2::durationFromSec(0.5));
        end_b    = tf_buffer->transform(end_in_cam,    "base_link",
                                        tf2::durationFromSec(0.5));
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(logger, "TF transform failed: %s", ex.what());
        return;
      }

      double dx = end_b.point.x - origin_b.point.x;
      double dy = end_b.point.y - origin_b.point.y;
      double dz = end_b.point.z - origin_b.point.z;
      if (std::abs(dz) < 1e-6) {
        RCLCPP_WARN(logger,
          "Ray nearly parallel to workspace plane (dz=%.6f); skipping.", dz);
        return;
      }
      double t = (workspace_z_m - origin_b.point.z) / dz;
      if (t <= 0.0) {
        RCLCPP_WARN(logger,
          "Ray points away from workspace (t=%.3f); skipping.", t);
        return;
      }
      double bx = origin_b.point.x + t * dx;
      double by = origin_b.point.y + t * dy;

      RCLCPP_INFO(logger,
        "ray=(%.4f, %.4f, 1.0) cam_origin_base=(%.3f, %.3f, %.3f) "
        "→ base_link cap=(%.3f, %.3f, %.3f) m  [plane z=%.3f]",
        msg->point.x, msg->point.y,
        origin_b.point.x, origin_b.point.y, origin_b.point.z,
        bx, by, workspace_z_m, workspace_z_m);
      {
        std::lock_guard<std::mutex> rl(ready_mu);
        ready_target = std::make_pair(bx, by);
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

  // Analytical SCARA IK — bypasses MoveIt's KDL solver, which can't handle
  // an under-actuated 3-DOF chain. Geometry from arm.xacro:
  //   Link_1_joint at base_link xy=(0, 0), z=0.16, axis (0,0,-1)
  //   Link_2_joint at Link_1   xy=( 0.14912, 0.00021), axis (0,0,+1)
  //   ee_joint origin in Link_2 xy=( 0.14661, -0.03577), z=-0.105
  //   ee_joint axis (0,0,-1), travel q_ee in [-0.045, 0.065]
  //   z_chain = 0.16 + 0.1565 - 0.105 = 0.2115, so Link_ee z = 0.2115 - q_ee
  constexpr double SHOULDER_X = 0.0;
  constexpr double SHOULDER_Y = 0.0;
  constexpr double L1         =  0.14912;
  constexpr double L2_OFF_X   =  0.13661;    // was 0.14661 — physical L2 is ~10 mm shorter than URDF.
  constexpr double L2_OFF_Y   =  0.0;        // was -0.035766 — physical arm has no lateral offset.
  const     double L2         = L2_OFF_X;
  const     double BETA       = 0.0;
  constexpr double Z_CHAIN    =  0.2115;
  constexpr double Q_EE_LO    = -0.045;
  constexpr double Q_EE_HI    =  0.065;

  auto solve_ik = [&](double x, double y, double z,
                      double & q1, double & q2, double & q_ee) -> bool {
    double tx = x - SHOULDER_X;
    double ty = y - SHOULDER_Y;
    double r2 = tx * tx + ty * ty;
    double cos_alpha = (r2 - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);
    if (cos_alpha < -1.0 || cos_alpha > 1.0) {
      RCLCPP_WARN(logger,
        "IK out of reach: target xy=(%.3f, %.3f) r=%.3f L1+L2=%.3f",
        x, y, std::sqrt(r2), L1 + L2);
      return false;
    }
    // Elbow-up branch (negative alpha). Flip sign for elbow-down if needed.
    double alpha = -std::acos(cos_alpha);  // = q2 + BETA in our chain
    double phi1  = std::atan2(ty, tx)
                 - std::atan2(L2 * std::sin(alpha), L1 + L2 * std::cos(alpha));
    q1   = -phi1;            // joint axis is (0,0,-1)
    q2   = alpha - BETA;     // remove L2 phase offset
    q_ee = Z_CHAIN - z;
    // EE disabled for planar-only testing — only q1/q2 are commanded.
    // if (q_ee < Q_EE_LO || q_ee > Q_EE_HI) {
    //   RCLCPP_WARN(logger,
    //     "IK z out of reach: z=%.3f → q_ee=%.3f (limits [%.3f, %.3f])",
    //     z, q_ee, Q_EE_LO, Q_EE_HI);
    //   return false;
    // }
    return true;
  };

  auto plan_and_execute_pose = [&](const std::string & label,
                                    double x, double y, double z) -> bool {
    RCLCPP_INFO(logger, "[%s] target xyz=(%.3f, %.3f, %.3f)",
                label.c_str(), x, y, z);
    double q1, q2, q_ee;
    if (!solve_ik(x, y, z, q1, q2, q_ee)) {
      RCLCPP_ERROR(logger, "[%s] IK failed.", label.c_str());
      return false;
    }
    RCLCPP_INFO(logger,
      "[%s] IK → Link_1_joint=%.3f  Link_2_joint=%.3f  ee_joint=%.3f",
      label.c_str(), q1, q2, q_ee);
    move_group.setStartStateToCurrentState();
    move_group.clearPoseTargets();
    move_group.setJointValueTarget(JointMap{
      {"Link_1_joint", q1},
      {"Link_2_joint", q2},
      // EE disabled for planar-only testing.
      // {"ee_joint",     q_ee},
    });
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

  // Screw / unscrew motion on the MKS ee motor while the arm is parked at
  // an engage pose. CCW screw_revs → dwell → CW screw_revs back. Single-
  // threaded blocking sleeps ensure joints 1 & 2 hold position throughout.
  auto run_screw_cycle = [&](const std::string & label) {
    const int delta = static_cast<int>(std::round(screw_revs * MKS_COUNTS_PER_REV));
    const int wait_ms = static_cast<int>(screw_wait_s * 1000);
    const int dwell_ms = static_cast<int>(screw_dwell_s * 1000);
    std_msgs::msg::Int32 m;

    // Sign convention here is from the user's viewing angle (not the MKS
    // datasheet's "from the shaft end"). Positive count = CCW as seen by
    // the operator. Flip if your mount orientation is opposite.
    RCLCPP_INFO(logger, "[%s] screw CCW %.1f rev (Δ=%+d counts)",
                label.c_str(), screw_revs, +delta);
    m.data = +delta;
    ee_rel_pub->publish(m);
    rclcpp::sleep_for(std::chrono::milliseconds(wait_ms));

    RCLCPP_INFO(logger, "[%s] dwell %.1fs", label.c_str(), screw_dwell_s);
    rclcpp::sleep_for(std::chrono::milliseconds(dwell_ms));

    RCLCPP_INFO(logger, "[%s] unscrew CW %.1f rev (Δ=%+d counts)",
                label.c_str(), screw_revs, -delta);
    m.data = -delta;
    ee_rel_pub->publish(m);
    rclcpp::sleep_for(std::chrono::milliseconds(wait_ms));
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

  // Move to the named SRDF state "begin_cam_pick" before accepting any
  // detections — this rotates the arm out of the camera's view so the
  // workspace is unobstructed when YOLO begins locking on a cap.
  {
    RCLCPP_INFO(logger, "[startup] moving to named state 'begin_cam_pick'");
    move_group.setStartStateToCurrentState();
    move_group.clearPoseTargets();
    move_group.setNamedTarget("begin_cam_pick");
    MoveGroupInterface::Plan startup_plan;
    if (static_cast<bool>(move_group.plan(startup_plan))) {
      move_group.execute(startup_plan);
    } else {
      RCLCPP_WARN(logger,
        "[startup] plan to 'begin_cam_pick' failed; continuing from current pose.");
    }
  }

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
      run_screw_cycle("engage_pick");
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
      run_screw_cycle("engage_place");
    }
    ok = ok && plan_and_execute_pose("retreat_place",
                                      place_x_m, place_y_m, z_travel_m);

    // Return to the same SRDF state we started from so the arm clears the
    // camera's view of the workspace before the next detection cycle.
    if (ok) {
      RCLCPP_INFO(logger, "[home_return] moving to named state 'begin_cam_pick'");
      move_group.setStartStateToCurrentState();
      move_group.clearPoseTargets();
      move_group.setNamedTarget("begin_cam_pick");
      MoveGroupInterface::Plan home_plan;
      if (static_cast<bool>(move_group.plan(home_plan))) {
        move_group.execute(home_plan);
      } else {
        RCLCPP_WARN(logger, "[home_return] plan to 'begin_cam_pick' failed.");
        ok = false;
      }
    }

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
