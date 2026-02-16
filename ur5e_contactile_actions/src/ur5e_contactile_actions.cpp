#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <thread>
#include <vector>
#include <string>
#include <chrono>
#include <memory>
#include <limits>
#include <cmath>

using namespace std::chrono_literals;

#ifndef ENABLE_GRIPPER
#define ENABLE_GRIPPER 0
#endif

#if ENABLE_GRIPPER
  #include <contactile_gripper_ros2/srv/gripper_command.hpp>
  #include <contactile_gripper_ros2/srv/gripper_serial_port.hpp>
#endif

static bool importMoveItDescriptionsFromMoveGroup(const rclcpp::Node::SharedPtr& node)
{
  auto params = std::make_shared<rclcpp::SyncParametersClient>(node, "/move_group");

  if (!params->wait_for_service(10s)) {
    RCLCPP_ERROR(node->get_logger(), "Param service on /move_group not available");
    return false;
  }

  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    const auto values = params->get_parameters({"robot_description", "robot_description_semantic"});

    if (values.size() == 2 &&
        values[0].get_type() == rclcpp::ParameterType::PARAMETER_STRING &&
        values[1].get_type() == rclcpp::ParameterType::PARAMETER_STRING)
    {
      const std::string urdf = values[0].as_string();
      const std::string srdf = values[1].as_string();

      if (!urdf.empty() && !srdf.empty()) {
        node->declare_parameter<std::string>("robot_description", urdf);
        node->declare_parameter<std::string>("robot_description_semantic", srdf);
        RCLCPP_INFO(node->get_logger(),
                    "Imported robot_description and robot_description_semantic from /move_group");
        return true;
      }
    }

    rclcpp::sleep_for(200ms);
  }

  RCLCPP_ERROR(node->get_logger(),
               "robot_description / robot_description_semantic not ready on /move_group (timeout)");
  return false;
}

static bool waitForCurrentState(
  const rclcpp::Node::SharedPtr& node,
  moveit::planning_interface::MoveGroupInterface& mgi,
  std::chrono::milliseconds timeout = 8000ms)
{
  auto start = std::chrono::steady_clock::now();
  while (rclcpp::ok()) {
    auto state = mgi.getCurrentState(0.1);
    if (state) return true;

    if (std::chrono::steady_clock::now() - start > timeout) {
      RCLCPP_ERROR(node->get_logger(),
                   "Timeout waiting for current robot state (joint_states).");
      return false;
    }
    rclcpp::sleep_for(100ms);
  }
  return false;
}

static void logGroupJointOrder(
  const rclcpp::Node::SharedPtr& node,
  moveit::planning_interface::MoveGroupInterface& mgi)
{
  const auto names = mgi.getJointNames();
  RCLCPP_INFO(node->get_logger(), "MoveGroupInterface joint order (%zu joints):", names.size());
  for (size_t i = 0; i < names.size(); ++i) {
    RCLCPP_INFO(node->get_logger(), "  [%zu] %s", i, names[i].c_str());
  }
}

static bool goToJointTarget(
  const rclcpp::Node::SharedPtr& node,
  moveit::planning_interface::MoveGroupInterface& mgi,
  const std::vector<std::string>& joint_names,
  const std::vector<double>& joint_angles)
{
  mgi.setStartStateToCurrentState();
  mgi.clearPoseTargets();

  if (!mgi.setJointValueTarget(joint_names, joint_angles)) {
    RCLCPP_ERROR(node->get_logger(),
                 "setJointValueTarget(names, angles) rejected target (wrong names/size?)");
    return false;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto plan_rc = mgi.plan(plan);
  if (plan_rc != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(),
                 "Planning failed to joint target (code=%d)", plan_rc.val);
    return false;
  }

  auto exec_rc = mgi.execute(plan);
  if (exec_rc != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(),
                 "Execution failed (code=%d)", exec_rc.val);
    return false;
  }

  return true;
}

static bool goToTarget(
  const rclcpp::Node::SharedPtr& node,
  moveit::planning_interface::MoveGroupInterface& mgi,
  const geometry_msgs::msg::Pose& target,
  const std::string& ref_frame,
  const std::string& eef_link)
{
  mgi.setPoseReferenceFrame(ref_frame);

  mgi.setStartStateToCurrentState();
  mgi.clearPoseTargets();
  mgi.setPoseTarget(target, eef_link);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto plan_rc = mgi.plan(plan);
  if (plan_rc != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Planning failed to target pose (code=%d)", plan_rc.val);
    return false;
  }

  auto exec_rc = mgi.execute(plan);
  if (exec_rc != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Execution failed to target pose (code=%d)", exec_rc.val);
    return false;
  }

  return true;
}

static bool buildShakeWaypointsAroundEEFZ(
  moveit::planning_interface::MoveGroupInterface& mgi,
  const std::string& eef_link,
  std::vector<geometry_msgs::msg::Pose>& waypoints,
  double angle_delta_rad,
  int num_shakes,
  double z_override = std::numeric_limits<double>::quiet_NaN())
{
  geometry_msgs::msg::Pose start_pose = mgi.getCurrentPose(eef_link).pose;
  waypoints.clear();
  waypoints.push_back(start_pose);

  tf2::Quaternion q_current;
  tf2::fromMsg(start_pose.orientation, q_current);

  for (int i = 0; i < num_shakes; ++i) {
    geometry_msgs::msg::Pose p = start_pose;

    if (!std::isnan(z_override)) {
      p.position.z = z_override;
    }

    tf2::Quaternion q_delta;
    q_delta.setRPY(0.0, 0.0, (i % 2 == 0) ? angle_delta_rad : -angle_delta_rad);

    tf2::Quaternion q_new = q_current * q_delta;
    q_new.normalize();
    p.orientation = tf2::toMsg(q_new);

    waypoints.push_back(p);
  }

  return true;
}

static bool executeCartesianWaypoints(
  const rclcpp::Node::SharedPtr& node,
  moveit::planning_interface::MoveGroupInterface& mgi,
  const std::vector<geometry_msgs::msg::Pose>& waypoints,
  double eef_step = 0.005,
  double jump_thresh = 0.0)
{
  moveit_msgs::msg::RobotTrajectory traj;
  double fraction = mgi.computeCartesianPath(waypoints, eef_step, jump_thresh, traj);

  RCLCPP_INFO(node->get_logger(), "Cartesian path achieved: %.2f%%", fraction * 100.0);
  if (fraction < 0.8) {
    RCLCPP_WARN(node->get_logger(), "Low Cartesian fraction; executing anyway.");
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = traj;

  auto exec_rc = mgi.execute(plan);
  if (exec_rc != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Cartesian execute failed (code=%d)", exec_rc.val);
    return false;
  }
  return true;
}

#if ENABLE_GRIPPER
static bool setGripperPort(
  const rclcpp::Node::SharedPtr& node,
  const std::string& port)
{
  auto client = node->create_client<contactile_gripper_ros2::srv::GripperSerialPort>(
    "/contactile_gripper_port_set");

  if (!client->wait_for_service(5s)) {
    RCLCPP_ERROR(node->get_logger(), "Service /contactile_gripper_port_set not available");
    return false;
  }

  auto req = std::make_shared<contactile_gripper_ros2::srv::GripperSerialPort::Request>();
  req->port = port;

  auto fut = client->async_send_request(req);
  if (rclcpp::spin_until_future_complete(node, fut, 5s) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Call to /contactile_gripper_port_set failed");
    return false;
  }

  auto resp = fut.get();
  RCLCPP_INFO(node->get_logger(), "Port set success=%s", resp->success ? "true" : "false");
  return resp->success;
}

static bool callGripperCommandSync(
  const rclcpp::Node::SharedPtr& node,
  const std::string& cmd,
  double arg = 0.0,
  bool use_arg = false,
  std::chrono::seconds timeout = 10s)
{
  auto client = node->create_client<contactile_gripper_ros2::srv::GripperCommand>(
    "/contactile_gripper_comms");

  if (!client->wait_for_service(5s)) {
    RCLCPP_ERROR(node->get_logger(), "Service /contactile_gripper_comms not available");
    return false;
  }

  auto req = std::make_shared<contactile_gripper_ros2::srv::GripperCommand::Request>();
  req->command_name = cmd;
  if (use_arg) req->command_argument = arg;

  auto fut = client->async_send_request(req);
  if (rclcpp::spin_until_future_complete(node, fut, timeout) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Gripper cmd '%s' timed out / failed", cmd.c_str());
    return false;
  }

  auto resp = fut.get();
  RCLCPP_INFO(node->get_logger(), "Gripper cmd '%s' -> result_enum=%d", cmd.c_str(), resp->result_enum);
  return true;
}
#endif

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ur5e_acquisition");

  const bool enable_gripper = node->declare_parameter<bool>("enable_gripper", false);
  const std::string planning_group = node->declare_parameter<std::string>("planning_group", "ur_manipulator");
  const std::string eef_link       = node->declare_parameter<std::string>("eef_link", "tool0");
  const std::string ref_frame      = node->declare_parameter<std::string>("ref_frame", "base_link");

  const std::vector<std::string> joint_names = {
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
  };

  std::vector<double> home_joint_values = node->declare_parameter<std::vector<double>>(
    "home_joint_values",
    std::vector<double>{ 0.0, -1.57, 0.0, -1.57, 0.0, 0.0 }
  );

  std::vector<double> grasp_joint_values = node->declare_parameter<std::vector<double>>(
    "grasp_joint_values",
    home_joint_values
  );

  const double shake_angle_delta = node->declare_parameter<double>("shake_angle_delta", 0.5);
  const int num_shakes           = node->declare_parameter<int>("num_shakes", 6);

  if (!importMoveItDescriptionsFromMoveGroup(node)) {
    RCLCPP_ERROR(node->get_logger(),
                 "Failed to import robot_description(_semantic) from /move_group.");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&]() { exec.spin(); });

  moveit::planning_interface::MoveGroupInterface mgi(node, planning_group);

  mgi.setPlanningPipelineId("ompl");
  mgi.setPlannerId("RRTConnect");
  mgi.setMaxVelocityScalingFactor(0.5);
  mgi.setMaxAccelerationScalingFactor(0.2);

  mgi.startStateMonitor();
  logGroupJointOrder(node, mgi);

  if (!waitForCurrentState(node, mgi, 8000ms)) {
    RCLCPP_ERROR(node->get_logger(), "No current state available, aborting.");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Going to home...");
  if (!goToJointTarget(node, mgi, joint_names, home_joint_values)) {
    RCLCPP_ERROR(node->get_logger(), "Failed going home.");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Going to grasp joint target...");
  (void)goToJointTarget(node, mgi, joint_names, grasp_joint_values);

  rclcpp::sleep_for(500ms);

  {
    auto pose = mgi.getCurrentPose(eef_link).pose;
    pose.position.y += 0.10;
    RCLCPP_INFO(node->get_logger(), "Going to pose target...");
    (void)goToTarget(node, mgi, pose, ref_frame, eef_link);
  }

#if ENABLE_GRIPPER
  if (enable_gripper) {
    // (void)setGripperPort(node, "/dev/contactile_gripper");

    RCLCPP_INFO(node->get_logger(), "Gripper DF_GRIP...");
    (void)callGripperCommandSync(node, "DF_GRIP");
    rclcpp::sleep_for(2s);

    RCLCPP_INFO(node->get_logger(), "Gripper RELEASE...");
    (void)callGripperCommandSync(node, "RELEASE");
    rclcpp::sleep_for(1s);

    RCLCPP_INFO(node->get_logger(), "Gripper DF_GRIP...");
    (void)callGripperCommandSync(node, "DF_GRIP");
    rclcpp::sleep_for(1s);
  } else {
    RCLCPP_INFO(node->get_logger(), "Gripper disabled by parameter.");
  }
#else
  (void)enable_gripper;
  RCLCPP_INFO(node->get_logger(), "Gripper support not compiled.");
#endif

  {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    (void)buildShakeWaypointsAroundEEFZ(mgi, eef_link, waypoints, shake_angle_delta, num_shakes);

    RCLCPP_INFO(node->get_logger(), "Executing shake cartesian path...");
    (void)executeCartesianWaypoints(node, mgi, waypoints, 0.005, 0.0);
  }

#if ENABLE_GRIPPER
  if (enable_gripper) {
    RCLCPP_INFO(node->get_logger(), "Gripper RELEASE...");
    (void)callGripperCommandSync(node, "RELEASE");
    rclcpp::sleep_for(2s);

    RCLCPP_INFO(node->get_logger(), "Gripper PC_MOVE_TO_WIDTH 100...");
    (void)callGripperCommandSync(node, "PC_MOVE_TO_WIDTH", 100.0, true);
    rclcpp::sleep_for(1s);
  }
#endif

  RCLCPP_INFO(node->get_logger(), "Returning home...");
  (void)goToJointTarget(node, mgi, joint_names, home_joint_values);

  RCLCPP_INFO(node->get_logger(), "Done.");
  rclcpp::shutdown();
  spinner.join();
  return 0;
}

