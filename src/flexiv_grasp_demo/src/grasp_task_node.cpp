// Pick-and-place state machine for the Flexiv Rizon grasp demo.
//
// The node owns the whole sequence: move to the observe pose, ask the perception
// node for a target, plan and execute the approach, close the gripper, lift,
// move to the place pose, release and retreat. Gripper commands go through the
// existing flexiv_gripper action servers, arm motion through MoveIt.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <sstream>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <flexiv_msgs/action/grasp.hpp>
#include <flexiv_msgs/action/move.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include "flexiv_grasp_demo/action/pick_place.hpp"
#include "flexiv_grasp_demo/msg/grasp_target.hpp"
#include "flexiv_grasp_demo/srv/detect.hpp"

namespace flexiv_grasp_demo
{

namespace
{
constexpr char kGripperMoveAction[] = "/flexiv_gripper_node/move";
constexpr char kGripperGraspAction[] = "/flexiv_gripper_node/grasp";
constexpr char kDetectService[] = "/grasp_demo/detect";

// Poll a future without blocking the executor that owns the callbacks.
template <typename FutureT>
bool wait_for(FutureT & future, double timeout_s)
{
    const auto deadline = std::chrono::steady_clock::now()
        + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(timeout_s));
    while (rclcpp::ok())
    {
        if (future.wait_for(std::chrono::milliseconds(20)) == std::future_status::ready)
        {
            return true;
        }
        if (std::chrono::steady_clock::now() >= deadline)
        {
            return false;
        }
    }
    return false;
}

geometry_msgs::msg::Pose make_pose(const std::vector<double> & xyz, const std::vector<double> & rpy)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = xyz.size() > 0 ? xyz[0] : 0.0;
    pose.position.y = xyz.size() > 1 ? xyz[1] : 0.0;
    pose.position.z = xyz.size() > 2 ? xyz[2] : 0.0;
    tf2::Quaternion q;
    q.setRPY(
        rpy.size() > 0 ? rpy[0] : 0.0,
        rpy.size() > 1 ? rpy[1] : 0.0,
        rpy.size() > 2 ? rpy[2] : 0.0);
    pose.orientation = tf2::toMsg(q);
    return pose;
}
}  // namespace

class GraspTaskNode : public rclcpp::Node
{
public:
    using PickPlace = flexiv_grasp_demo::action::PickPlace;
    using GoalHandle = rclcpp_action::ServerGoalHandle<PickPlace>;
    using GripperMove = flexiv_msgs::action::Move;
    using GripperGrasp = flexiv_msgs::action::Grasp;
    using Detect = flexiv_grasp_demo::srv::Detect;

    GraspTaskNode();
    ~GraspTaskNode() override;

    // Creates the MoveIt interface once the node is being spun.
    void initialize();

private:
    // ---- action plumbing ----
    rclcpp_action::GoalResponse handle_goal(const std::shared_ptr<const PickPlace::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel();
    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
    void worker_loop();
    void execute(const std::shared_ptr<GoalHandle> & goal_handle);

    // ---- capabilities ----
    bool ensure_move_group(std::string & error);
    bool fetch_robot_model_params(std::string & urdf, std::string & srdf, std::string & error);
    bool move_to_pose(const geometry_msgs::msg::Pose & target, const std::string & stage,
        std::string & error);
    bool cartesian_move(
        const std::vector<geometry_msgs::msg::Pose> & waypoints, const std::string & stage,
        std::string & error);
    bool descend_with_contact_stop(
        const geometry_msgs::msg::Pose & from, const geometry_msgs::msg::Pose & to,
        std::string & error, geometry_msgs::msg::Pose & last_reached, double & contact_force);
    bool gripper_move(double width, std::string & error);
    bool gripper_grasp(double force, std::string & error, double & measured_width);
    bool detect(const std::string & object_id, msg::GraspTarget & target, std::string & error);
    bool detect_robust(
        const std::string & object_id, msg::GraspTarget & target, std::string & error);

    // ---- helpers ----
    geometry_msgs::msg::Pose compute_grasp_pose(const msg::GraspTarget & target) const;
    bool canceled(const std::shared_ptr<GoalHandle> & goal_handle) const;
    void feedback(
        const std::shared_ptr<GoalHandle> & goal_handle, const std::string & stage, double progress,
        const std::string & detail = "");

    // ---- parameters ----
    std::string robot_sn_;
    std::string planning_group_;
    std::string base_frame_;
    std::string tcp_frame_;
    std::string detect_mode_;
    geometry_msgs::msg::Pose observe_pose_;
    geometry_msgs::msg::Pose place_pose_;
    std::vector<double> mock_tag_xyz_;
    std::vector<double> mock_tag_rpy_;
    int mock_tag_id_{0};
    double approach_height_{0.10};
    double grasp_offset_{0.0};
    double grasp_yaw_offset_{0.0};
    double lift_height_{0.10};
    double place_release_height_{0.05};
    double grasp_force_{20.0};
    double min_grasp_width_{0.030};
    double min_grasp_force_{3.0};
    std::string descent_mode_{"single"};
    double open_width_{0.09};
    double close_width_{0.01};
    double gripper_velocity_{0.1};
    double gripper_max_force_{20.0};
    double velocity_scaling_{0.1};
    double acceleration_scaling_{0.1};
    double planning_time_{5.0};
    double cartesian_step_{0.005};
    double min_cartesian_fraction_{0.9};
    double settle_time_s_{0.5};
    double base_timeout_s_{2.0};
    int detect_retries_{3};
    int detect_samples_{3};
    bool contact_stop_{true};
    double contact_force_n_{5.0};
    double descent_step_{0.010};
    double contact_settle_{0.10};
    std::atomic<double> latest_force_{0.0};
    double last_grasp_force_{0.0};
    std::atomic<double> wrench_stamp_{-1.0};
    std::string place_mode_{"relative"};
    std::vector<double> place_offset_{0.0, 0.10, 0.0};
    std::string place_height_mode_{"grasp"};
    double place_z_offset_{0.0};
    std::string grasp_height_mode_{"table"};
    double object_height_{0.040};
    double grasp_height_offset_{0.0};
    double pre_grasp_dwell_{0.5};
    double table_z_{-0.05021};
    double min_tcp_clearance_{0.010};
    bool enable_table_guard_{true};
    int move_attempts_{2};

    // ---- state ----
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::mutex move_group_mutex_;

    rclcpp_action::Server<PickPlace>::SharedPtr action_server_;
    rclcpp_action::Client<GripperMove>::SharedPtr gripper_move_client_;
    rclcpp_action::Client<GripperGrasp>::SharedPtr gripper_grasp_client_;
    rclcpp::Client<Detect>::SharedPtr detect_client_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_subscription_;

    std::thread worker_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::deque<std::shared_ptr<GoalHandle>> queue_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> busy_{false};
    std::atomic<bool> cancel_requested_{false};
};

GraspTaskNode::GraspTaskNode()
: Node("grasp_task")
{
    robot_sn_ = this->declare_parameter<std::string>("robot_sn", "");
    if (robot_sn_.empty())
    {
        RCLCPP_ERROR(
            get_logger(), "Parameter 'robot_sn' is empty: frame and planning group defaults are unusable");
    }
    // Flexiv keeps the serial number verbatim in link names, hyphens included
    // (e.g. Rizon4s-123456_base_link), so no sanitising here.
    planning_group_ = this->declare_parameter<std::string>("planning_group", "");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "");
    tcp_frame_ = this->declare_parameter<std::string>("tcp_frame", "");
    if (planning_group_.empty())
    {
        planning_group_ = robot_sn_ + "_arm";
    }
    if (base_frame_.empty())
    {
        base_frame_ = robot_sn_ + "_base_link";
    }
    if (tcp_frame_.empty())
    {
        tcp_frame_ = robot_sn_ + "_grav_tcp";
    }

    detect_mode_ = this->declare_parameter<std::string>("detect_mode", "vision");

    observe_pose_ = make_pose(
        this->declare_parameter<std::vector<double>>("observe_pose_xyz", {0.55, 0.0, 0.45}),
        this->declare_parameter<std::vector<double>>("observe_pose_rpy", {3.14159, 0.0, 0.0}));
    place_pose_ = make_pose(
        this->declare_parameter<std::vector<double>>("place_pose_xyz", {0.45, 0.25, 0.15}),
        this->declare_parameter<std::vector<double>>("place_pose_rpy", {3.14159, 0.0, 0.0}));
    mock_tag_xyz_ = this->declare_parameter<std::vector<double>>("mock_tag_xyz", {0.45, 0.0, 0.02});
    mock_tag_rpy_ = this->declare_parameter<std::vector<double>>("mock_tag_rpy", {0.0, 0.0, 0.0});
    mock_tag_id_ = this->declare_parameter<int>("mock_tag_id", 0);

    approach_height_ = this->declare_parameter<double>("approach_height", 0.10);
    grasp_offset_ = this->declare_parameter<double>("grasp_offset", 0.0);
    grasp_yaw_offset_ = this->declare_parameter<double>("grasp_yaw_offset", 0.0);
    lift_height_ = this->declare_parameter<double>("lift_height", 0.10);
    place_release_height_ = this->declare_parameter<double>("place_release_height", 0.05);
    grasp_force_ = this->declare_parameter<double>("grasp_force", 20.0);
    min_grasp_width_ = this->declare_parameter<double>("min_grasp_width", 0.030);
    min_grasp_force_ = this->declare_parameter<double>("min_grasp_force", 3.0);
    descent_mode_ = this->declare_parameter<std::string>("descent_mode", "single");
    open_width_ = this->declare_parameter<double>("open_width", 0.09);
    close_width_ = this->declare_parameter<double>("close_width", 0.01);
    gripper_velocity_ = this->declare_parameter<double>("gripper_velocity", 0.1);
    gripper_max_force_ = this->declare_parameter<double>("gripper_max_force", 20.0);
    velocity_scaling_ = this->declare_parameter<double>("velocity_scaling", 0.1);
    acceleration_scaling_ = this->declare_parameter<double>("acceleration_scaling", 0.1);
    planning_time_ = this->declare_parameter<double>("planning_time", 5.0);
    cartesian_step_ = this->declare_parameter<double>("cartesian_step", 0.005);
    min_cartesian_fraction_ = this->declare_parameter<double>("min_cartesian_fraction", 0.9);
    settle_time_s_ = this->declare_parameter<double>("settle_time", 0.5);
    base_timeout_s_ = this->declare_parameter<double>("client_timeout", 2.0);
    detect_retries_ = this->declare_parameter<int>("detect_retries", 3);
    detect_samples_ = this->declare_parameter<int>("detect_samples", 3);
    contact_stop_ = this->declare_parameter<bool>("contact_stop", true);
    contact_force_n_ = this->declare_parameter<double>("contact_force_n", 5.0);
    descent_step_ = this->declare_parameter<double>("descent_step", 0.010);
    contact_settle_ = this->declare_parameter<double>("contact_settle", 0.10);
    place_mode_ = this->declare_parameter<std::string>("place_mode", "relative");
    place_offset_ = this->declare_parameter<std::vector<double>>(
        "place_offset_from_grasp", {0.0, 0.10, 0.0});
    place_height_mode_ = this->declare_parameter<std::string>("place_height_mode", "grasp");
    place_z_offset_ = this->declare_parameter<double>("place_z_offset", 0.0);
    grasp_height_mode_ = this->declare_parameter<std::string>("grasp_height_mode", "table");
    object_height_ = this->declare_parameter<double>("object_height", 0.040);
    grasp_height_offset_ = this->declare_parameter<double>("grasp_height_offset", 0.0);
    pre_grasp_dwell_ = this->declare_parameter<double>("pre_grasp_dwell", 0.5);
    table_z_ = this->declare_parameter<double>("table_z", -0.05021);
    min_tcp_clearance_ = this->declare_parameter<double>("min_tcp_clearance", 0.010);
    enable_table_guard_ = this->declare_parameter<bool>("enable_table_guard", true);
    move_attempts_ = this->declare_parameter<int>("move_attempts", 2);

    // MoveGroupInterface builds its own RobotModel from these parameters, so the
    // values are copied over from /move_group at runtime instead of duplicating
    // the xacro invocation of the bringup launch file.
    this->declare_parameter<std::string>("robot_description", "");
    this->declare_parameter<std::string>("robot_description_semantic", "");

    gripper_move_client_ = rclcpp_action::create_client<GripperMove>(this, kGripperMoveAction);
    gripper_grasp_client_ = rclcpp_action::create_client<GripperGrasp>(this, kGripperGraspAction);
    detect_client_ = this->create_client<Detect>(kDetectService);

    if (contact_stop_)
    {
        // Gravity-compensated external wrench: it reads ~0 while free and rises as soon
        // as the open fingers touch the table.
        // ROS 2 topic names cannot contain hyphens, so the driver replaces them with
        // underscores when it builds its topics; do the same here.
        std::string topic_sn = robot_sn_;
        std::replace(topic_sn.begin(), topic_sn.end(), '-', '_');
        const std::string wrench_topic = "/" + topic_sn + "/external_wrench_in_tcp";
        wrench_subscription_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            wrench_topic, rclcpp::SensorDataQoS().keep_last(1),
            [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
                const auto & f = msg->wrench.force;
                latest_force_ = std::sqrt(f.x * f.x + f.y * f.y + f.z * f.z);
                wrench_stamp_ = std::chrono::duration<double>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
            });
        RCLCPP_INFO(get_logger(), "Contact stop armed on %s (threshold %.1f N)",
            wrench_topic.c_str(), contact_force_n_);
    }

    action_server_ = rclcpp_action::create_server<PickPlace>(
        this, "/grasp_demo/pick_place",
        [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const PickPlace::Goal> goal) {
            return handle_goal(goal);
        },
        [this](const std::shared_ptr<GoalHandle>) { return handle_cancel(); },
        [this](const std::shared_ptr<GoalHandle> goal_handle) { handle_accepted(goal_handle); });

    worker_ = std::thread(&GraspTaskNode::worker_loop, this);

    RCLCPP_INFO(
        get_logger(),
        "grasp_task ready: group='%s' base='%s' tcp='%s' detect_mode='%s'",
        planning_group_.c_str(), base_frame_.c_str(), tcp_frame_.c_str(), detect_mode_.c_str());
}

GraspTaskNode::~GraspTaskNode()
{
    stop_ = true;
    queue_cv_.notify_all();
    if (worker_.joinable())
    {
        worker_.join();
    }
}

void GraspTaskNode::initialize()
{
    std::string error;
    if (!ensure_move_group(error))
    {
        RCLCPP_WARN(
            get_logger(),
            "MoveIt not available yet (%s); the first goal will retry the connection", error.c_str());
    }
}

rclcpp_action::GoalResponse GraspTaskNode::handle_goal(
    const std::shared_ptr<const PickPlace::Goal> /*goal*/)
{
    if (busy_.load())
    {
        RCLCPP_WARN(get_logger(), "Rejecting goal: another pick-and-place is running");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GraspTaskNode::handle_cancel()
{
    cancel_requested_ = true;
    RCLCPP_WARN(get_logger(), "Cancel requested; stopping after the current motion finishes");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void GraspTaskNode::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
    busy_ = true;
    cancel_requested_ = false;
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        queue_.push_back(goal_handle);
    }
    queue_cv_.notify_one();
}

void GraspTaskNode::worker_loop()
{
    while (rclcpp::ok() && !stop_.load())
    {
        std::shared_ptr<GoalHandle> goal_handle;
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait_for(lock, std::chrono::milliseconds(200), [this] {
                return stop_.load() || !queue_.empty();
            });
            if (stop_.load() || queue_.empty())
            {
                continue;
            }
            goal_handle = queue_.front();
            queue_.pop_front();
        }
        try
        {
            execute(goal_handle);
        }
        catch (const std::exception & e)
        {
            auto result = std::make_shared<PickPlace::Result>();
            result->success = false;
            result->message = std::string("INTERNAL_ERROR: ") + e.what();
            goal_handle->abort(result);
            RCLCPP_ERROR(get_logger(), "Goal aborted: %s", e.what());
        }
        busy_ = false;
        cancel_requested_ = false;
    }
}

bool GraspTaskNode::canceled(const std::shared_ptr<GoalHandle> & goal_handle) const
{
    return cancel_requested_.load() || goal_handle->is_canceling();
}

void GraspTaskNode::feedback(
    const std::shared_ptr<GoalHandle> & goal_handle, const std::string & stage, double progress,
    const std::string & detail)
{
    auto msg = std::make_shared<PickPlace::Feedback>();
    msg->stage = stage;
    msg->progress = progress;
    msg->detail = detail;
    goal_handle->publish_feedback(msg);
    RCLCPP_INFO(get_logger(), "[%s] %.2f %s", stage.c_str(), progress, detail.c_str());
}

bool GraspTaskNode::fetch_robot_model_params(
    std::string & urdf, std::string & srdf, std::string & error)
{
    auto client = std::make_shared<rclcpp::AsyncParametersClient>(shared_from_this(), "/move_group");
    if (!client->wait_for_service(std::chrono::seconds(5)))
    {
        error = "/move_group parameter service is not available";
        return false;
    }
    auto future = client->get_parameters({"robot_description", "robot_description_semantic"});
    if (!wait_for(future, 5.0))
    {
        error = "timed out reading robot_description from /move_group";
        return false;
    }
    const auto params = future.get();
    if (params.size() != 2
        || params[0].get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET
        || params[1].get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
    {
        error = "/move_group does not expose robot_description and robot_description_semantic";
        return false;
    }
    urdf = params[0].as_string();
    srdf = params[1].as_string();
    return true;
}

bool GraspTaskNode::ensure_move_group(std::string & error)
{
    std::lock_guard<std::mutex> lock(move_group_mutex_);
    if (move_group_)
    {
        return true;
    }
    try
    {
        std::string urdf;
        std::string srdf;
        if (!fetch_robot_model_params(urdf, srdf, error))
        {
            return false;
        }
        // RobotModelLoader reads these from the node the interface is built on.
        this->set_parameter(rclcpp::Parameter("robot_description", urdf));
        this->set_parameter(rclcpp::Parameter("robot_description_semantic", srdf));
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), planning_group_, nullptr, rclcpp::Duration::from_seconds(5.0));
        move_group_->setEndEffectorLink(tcp_frame_);
        if (move_group_->getEndEffectorLink() != tcp_frame_)
        {
            RCLCPP_WARN(
                get_logger(), "TCP frame '%s' is unknown to MoveIt; planning uses '%s' instead",
                tcp_frame_.c_str(), move_group_->getEndEffectorLink().c_str());
        }
        move_group_->setPoseReferenceFrame(base_frame_);
        move_group_->setPlanningTime(planning_time_);
        move_group_->setNumPlanningAttempts(3);
        move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
        move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_);
        RCLCPP_INFO(get_logger(), "Connected to move_group for planning group '%s'", planning_group_.c_str());
        return true;
    }
    catch (const std::exception & e)
    {
        error = e.what();
        move_group_.reset();
        return false;
    }
}

bool GraspTaskNode::move_to_pose(
    const geometry_msgs::msg::Pose & target, const std::string & stage, std::string & error)
{
    if (!ensure_move_group(error))
    {
        error = "PLAN_FAILED: MoveIt unavailable (" + error + ")";
        return false;
    }
    std::lock_guard<std::mutex> lock(move_group_mutex_);

    for (int attempt = 1; attempt <= move_attempts_; ++attempt)
    {
        move_group_->setStartStateToCurrentState();
        move_group_->clearPoseTargets();
        move_group_->setPoseTarget(target);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        const auto plan_code = move_group_->plan(plan);
        if (plan_code != moveit::core::MoveItErrorCode::SUCCESS)
        {
            error = "PLAN_FAILED: could not plan to stage " + stage;
            RCLCPP_WARN(get_logger(), "%s (attempt %d/%d)", error.c_str(), attempt, move_attempts_);
            continue;
        }
        const auto exec_code = move_group_->execute(plan);
        move_group_->clearPoseTargets();
        if (exec_code == moveit::core::MoveItErrorCode::SUCCESS)
        {
            return true;
        }
        error = "EXEC_FAILED: controller rejected the trajectory for stage " + stage;
        RCLCPP_WARN(get_logger(), "%s (attempt %d/%d)", error.c_str(), attempt, move_attempts_);
    }
    return false;
}

bool GraspTaskNode::cartesian_move(
    const std::vector<geometry_msgs::msg::Pose> & waypoints, const std::string & stage,
    std::string & error)
{
    if (!ensure_move_group(error))
    {
        error = "PLAN_FAILED: MoveIt unavailable (" + error + ")";
        return false;
    }
    std::lock_guard<std::mutex> lock(move_group_mutex_);

    move_group_->setStartStateToCurrentState();
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double fraction = move_group_->computeCartesianPath(
        waypoints, cartesian_step_, 0.0, trajectory, true);
    if (fraction < min_cartesian_fraction_)
    {
        error = "PLAN_FAILED: cartesian path for stage " + stage + " reached only "
            + std::to_string(fraction);
        return false;
    }
    const auto exec_code = move_group_->execute(trajectory);
    if (exec_code != moveit::core::MoveItErrorCode::SUCCESS)
    {
        error = "EXEC_FAILED: controller rejected the cartesian trajectory for stage " + stage;
        return false;
    }
    return true;
}

bool GraspTaskNode::descend_with_contact_stop(
    const geometry_msgs::msg::Pose & from, const geometry_msgs::msg::Pose & to,
    std::string & error, geometry_msgs::msg::Pose & last_reached, double & contact_force)
{
    const auto now = []() {
        return std::chrono::duration<double>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    };

    // No force data (fake hardware, driver stopped): a straight move is all we can do.
    if (wrench_stamp_.load() < 0.0 || now() - wrench_stamp_.load() > 2.0)
    {
        RCLCPP_WARN(
            get_logger(),
            "No fresh /external_wrench_in_tcp; descending without contact stop");
        last_reached = to;
        return cartesian_move({from, to}, "APPROACH", error);
    }

    const double dx = to.position.x - from.position.x;
    const double dy = to.position.y - from.position.y;
    const double dz = to.position.z - from.position.z;
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    const int steps = std::max(1, static_cast<int>(std::ceil(distance / descent_step_)));

    geometry_msgs::msg::Pose previous = from;
    for (int i = 1; i <= steps; ++i)
    {
        const double t = static_cast<double>(i) / steps;
        geometry_msgs::msg::Pose waypoint = from;
        waypoint.position.x += dx * t;
        waypoint.position.y += dy * t;
        waypoint.position.z += dz * t;

        const geometry_msgs::msg::Pose step_start = previous;
        if (!cartesian_move({previous, waypoint}, "APPROACH", error))
        {
            return false;
        }
        last_reached = waypoint;

        std::this_thread::sleep_for(std::chrono::duration<double>(contact_settle_));
        contact_force = latest_force_.load();
        if (contact_force > contact_force_n_)
        {
            // Report the bracket, not just the final point: the surface is somewhere
            // between the start of this step and where the force tripped, and that range
            // is what the operator needs to fill into table_z.
            std::ostringstream message;
            message << "touched between z=" << step_start.position.z << " and z="
                    << waypoint.position.z << " m with " << contact_force
                    << " N; the surface is inside that range, set table_z there";
            error = message.str();
            last_reached = step_start;
            return false;
        }
        previous = waypoint;
    }
    return true;
}

bool GraspTaskNode::gripper_move(double width, std::string & error)
{
    if (!gripper_move_client_->wait_for_action_server(
            std::chrono::duration<double>(base_timeout_s_)))
    {
        error = "PRECHECK_FAILED: gripper move action server unavailable";
        return false;
    }
    GripperMove::Goal goal;
    goal.width = width;
    goal.velocity = gripper_velocity_;
    goal.max_force = gripper_max_force_;
    auto goal_future = gripper_move_client_->async_send_goal(goal);
    if (!wait_for(goal_future, base_timeout_s_))
    {
        error = "GRASP_FAILED: gripper move goal was not accepted";
        return false;
    }
    auto result_future = gripper_move_client_->async_get_result(goal_future.get());
    if (!wait_for(result_future, base_timeout_s_ * 5.0))
    {
        error = "GRASP_FAILED: gripper move timed out";
        return false;
    }
    const auto wrapped = result_future.get();
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED)
    {
        error = "GRASP_FAILED: gripper move did not succeed";
        return false;
    }
    if (!wrapped.result->success)
    {
        error = "GRASP_FAILED: " + wrapped.result->error;
        return false;
    }
    return true;
}

bool GraspTaskNode::gripper_grasp(double force, std::string & error, double & measured_width)
{
    if (!gripper_grasp_client_->wait_for_action_server(
            std::chrono::duration<double>(base_timeout_s_)))
    {
        error = "PRECHECK_FAILED: gripper grasp action server unavailable";
        return false;
    }
    GripperGrasp::Goal goal;
    goal.force = force;

    // The grasp result carries no width, so track it through feedback.
    rclcpp_action::Client<GripperGrasp>::SendGoalOptions options;
    double measured_force = 0.0;
    options.feedback_callback = [&measured_width, &measured_force](
        rclcpp_action::Client<GripperGrasp>::GoalHandle::SharedPtr,
        const std::shared_ptr<const GripperGrasp::Feedback> feedback) {
            measured_width = feedback->current_width;
            measured_force = feedback->current_force;
        };
    auto goal_future = gripper_grasp_client_->async_send_goal(goal, options);
    if (!wait_for(goal_future, base_timeout_s_))
    {
        error = "GRASP_FAILED: gripper grasp goal was not accepted";
        return false;
    }
    auto goal_handle = goal_future.get();
    auto result_future = gripper_grasp_client_->async_get_result(goal_handle);
    if (!wait_for(result_future, base_timeout_s_ * 5.0))
    {
        error = "GRASP_FAILED: gripper grasp timed out";
        return false;
    }
    const auto wrapped = result_future.get();
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result->success)
    {
        error = "GRASP_FAILED: " + (wrapped.result ? wrapped.result->error : "no result");
        return false;
    }
    if (measured_width <= 0.0)
    {
        measured_width = close_width_;
    }
    RCLCPP_INFO(
        get_logger(), "Gripper closed to %.1f mm with %.1f N",
        measured_width * 1000.0, measured_force);
    last_grasp_force_ = measured_force;
    return true;
}

bool GraspTaskNode::detect(
    const std::string & object_id, msg::GraspTarget & target, std::string & error)
{
    if (detect_mode_ == "mock")
    {
        target = msg::GraspTarget();
        target.id = static_cast<uint16_t>(mock_tag_id_);
        target.pose.header.frame_id = base_frame_;
        target.pose.pose = make_pose(mock_tag_xyz_, mock_tag_rpy_);
        target.size.x = target.size.y = 0.026;
        target.size.z = 0.0;
        target.area = 1.0;
        target.reprojection_error = 0.0;
        RCLCPP_INFO(
            get_logger(), "Using mock detection (id=%u) at [%.3f, %.3f, %.3f]", target.id,
            target.pose.pose.position.x, target.pose.pose.position.y,
            target.pose.pose.position.z);
        return true;
    }

    if (!detect_client_->wait_for_service(std::chrono::duration<double>(base_timeout_s_)))
    {
        error = "DETECT_FAILED: perception service unavailable";
        return false;
    }

    auto request = std::make_shared<Detect::Request>();
    request->object_id = object_id;
    auto future = detect_client_->async_send_request(request);
    if (!wait_for(future.future, base_timeout_s_ * 2.0))
    {
        error = "DETECT_FAILED: perception service timed out";
        return false;
    }
    const auto response = future.future.get();
    if (response->image_stale)
    {
        error = "DETECT_FAILED: camera frame is stale";
        return false;
    }
    if (response->targets.empty())
    {
        // Keep the perception node's wording: "no marker detected" and "none faces up"
        // need completely different fixes, and they used to look identical here.
        error = "DETECT_FAILED: "
            + (response->message.empty() ? std::string("no visible marker") : response->message);
        return false;
    }
    target = response->targets.front();
    return true;
}

bool GraspTaskNode::detect_robust(
    const std::string & object_id, msg::GraspTarget & target, std::string & error)
{
    // A single frame is occasionally off by tens of millimetres (planar marker
    // ambiguity, a side face slipping through, motion blur). The arm is parked while
    // detecting, so take a few frames and keep the median: cheap outlier rejection.
    if (detect_mode_ == "mock" || detect_samples_ <= 1)
    {
        return detect(object_id, target, error);
    }

    std::vector<msg::GraspTarget> samples;
    for (int i = 0; i < detect_samples_; ++i)
    {
        msg::GraspTarget sample;
        std::string sample_error;
        if (detect(object_id, sample, sample_error))
        {
            samples.push_back(sample);
        }
        else
        {
            error = sample_error;
        }
        if (i + 1 < detect_samples_)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(80));
        }
    }
    if (samples.empty())
    {
        return false;
    }
    if (samples.size() == 1)
    {
        target = samples.front();
        return true;
    }

    const auto component_median = [&samples](auto getter) {
        std::vector<double> values;
        values.reserve(samples.size());
        for (const auto & sample : samples)
        {
            values.push_back(getter(sample));
        }
        std::sort(values.begin(), values.end());
        return values[values.size() / 2];
    };
    const double mx = component_median([](const auto & t) { return t.pose.pose.position.x; });
    const double my = component_median([](const auto & t) { return t.pose.pose.position.y; });
    const double mz = component_median([](const auto & t) { return t.pose.pose.position.z; });

    // Use the sample closest to the median so the orientation stays a real measurement.
    size_t best = 0;
    double best_distance = std::numeric_limits<double>::max();
    for (size_t i = 0; i < samples.size(); ++i)
    {
        const auto & p = samples[i].pose.pose.position;
        const double distance = std::hypot(p.x - mx, p.y - my, p.z - mz);
        if (distance < best_distance)
        {
            best_distance = distance;
            best = i;
        }
    }
    target = samples[best];
    target.pose.pose.position.x = mx;
    target.pose.pose.position.y = my;
    target.pose.pose.position.z = mz;

    RCLCPP_INFO(
        get_logger(), "Detection median of %zu samples, worst deviation %.1f mm",
        samples.size(), best_distance * 1000.0);
    return true;
}

geometry_msgs::msg::Pose GraspTaskNode::compute_grasp_pose(const msg::GraspTarget & target) const
{
    tf2::Quaternion q_tag;
    tf2::fromMsg(target.pose.pose.orientation, q_tag);
    q_tag.normalize();

    const tf2::Vector3 normal = tf2::quatRotate(q_tag, tf2::Vector3(0.0, 0.0, 1.0));
    const tf2::Vector3 tag_center(
        target.pose.pose.position.x, target.pose.pose.position.y, target.pose.pose.position.z);

    // Approach along the marker normal; the tool z axis points into the object.
    const tf2::Vector3 grasp_position = tag_center + normal * grasp_offset_;

    tf2::Quaternion q_flip;
    q_flip.setRPY(M_PI, 0.0, 0.0);
    tf2::Quaternion q_yaw;
    q_yaw.setRPY(0.0, 0.0, grasp_yaw_offset_);
    tf2::Quaternion q_grasp = q_tag * q_flip * q_yaw;
    q_grasp.normalize();

    geometry_msgs::msg::Pose pose;
    pose.position.x = grasp_position.x();
    pose.position.y = grasp_position.y();
    pose.position.z = grasp_position.z();
    pose.orientation = tf2::toMsg(q_grasp);
    return pose;
}

void GraspTaskNode::execute(const std::shared_ptr<GoalHandle> & goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<PickPlace::Result>();

    const auto abort_with = [&](const std::string & code, const std::string & detail) {
        result->success = false;
        result->message = detail.empty() ? code : code + ": " + detail;
        RCLCPP_ERROR(get_logger(), "Pick-and-place failed: %s", result->message.c_str());
        goal_handle->abort(result);
    };

    // ---- OBSERVE ----
    feedback(goal_handle, "OBSERVE", 0.05, "moving to the observe pose");
    std::string error;
    if (!move_to_pose(observe_pose_, "OBSERVE", error))
    {
        abort_with("EXEC_FAILED", error);
        return;
    }
    if (canceled(goal_handle))
    {
        result->message = "CANCELED";
        goal_handle->canceled(result);
        return;
    }
    std::this_thread::sleep_for(std::chrono::duration<double>(settle_time_s_));

    // ---- DETECT ----
    feedback(goal_handle, "DETECT", 0.20, "querying the perception node");
    msg::GraspTarget target;
    bool detected = false;
    for (int attempt = 1; attempt <= detect_retries_ && rclcpp::ok(); ++attempt)
    {
        if (detect_robust(goal->object_id, target, error))
        {
            detected = true;
            break;
        }
        RCLCPP_WARN(get_logger(), "Detection attempt %d/%d failed: %s", attempt, detect_retries_,
            error.c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
    if (!detected)
    {
        abort_with("DETECT_FAILED", error);
        return;
    }

    // ---- GRASP POSE ----
    auto grasp_pose = compute_grasp_pose(target);

    // Planar marker detection gives good x/y but a poor depth: 1-2 cm at this marker
    // size, and it varies with the viewing pose. The table height and the cube size are
    // known, so the grasp height is derived from geometry instead of the detection.
    // x/y still come from the marker, where the error is only a few millimetres.
    if (grasp_height_mode_ == "table")
    {
        grasp_pose.position.z = table_z_ + object_height_ * 0.5 + grasp_height_offset_;
        RCLCPP_INFO(
            get_logger(),
            "Grasp height from table geometry: %.4f m (table %.4f + half height %.4f), "
            "marker reported %.4f m",
            grasp_pose.position.z, table_z_, object_height_ * 0.5, target.pose.pose.position.z);
    }

    // The detected height drifts by a centimetre or so at this marker size. Without a
    // guard that drift can put the fingertips into the table, so keep the TCP at least
    // min_tcp_clearance above the known table plane.
    if (enable_table_guard_ && grasp_pose.position.z < table_z_ + min_tcp_clearance_)
    {
        RCLCPP_WARN(
            get_logger(),
            "Grasp height %.4f m is below table %.4f m + clearance %.4f m; clamping",
            grasp_pose.position.z, table_z_, min_tcp_clearance_);
        grasp_pose.position.z = table_z_ + min_tcp_clearance_;
    }
    result->grasp_pose_used.header.frame_id = base_frame_;
    result->grasp_pose_used.pose = grasp_pose;

    tf2::Quaternion q_grasp;
    tf2::fromMsg(grasp_pose.orientation, q_grasp);
    const tf2::Vector3 approach_dir = tf2::quatRotate(q_grasp, tf2::Vector3(0.0, 0.0, 1.0));
    const double approach = goal->approach_height > 0.0 ? goal->approach_height : approach_height_;

    geometry_msgs::msg::Pose pre_grasp = grasp_pose;
    pre_grasp.position.x -= approach_dir.x() * approach;
    pre_grasp.position.y -= approach_dir.y() * approach;
    pre_grasp.position.z -= approach_dir.z() * approach;

    // ---- OPEN ----
    // The gripper keeps whatever width the previous run left it at (a failed run or a
    // manual test can leave it closed). Command the open width explicitly before the
    // approach, otherwise the fingers are driven into the cube.
    feedback(goal_handle, "APPROACH", 0.30, "opening the gripper before the approach");
    const double open_width = open_width_;
    if (!gripper_move(open_width, error))
    {
        abort_with("GRASP_FAILED", error);
        return;
    }
    // Give the planning scene a moment to pick up the new finger positions; planning
    // from a stale (nearly closed) gripper state can be rejected as self-collision.
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    if (canceled(goal_handle))
    {
        result->message = "CANCELED";
        goal_handle->canceled(result);
        return;
    }

    // ---- APPROACH ----
    feedback(goal_handle, "APPROACH", 0.35, "planning to the pre-grasp pose");
    if (!move_to_pose(pre_grasp, "APPROACH", error))
    {
        abort_with("PLAN_FAILED", error);
        return;
    }
    if (canceled(goal_handle))
    {
        result->message = "CANCELED";
        goal_handle->canceled(result);
        return;
    }
    // Let the open gripper sit still above the cube before descending: the operator can
    // see that the fingers clear the object, and the arm has stopped wobbling when the
    // straight descent starts.
    if (pre_grasp_dwell_ > 0.0)
    {
        std::this_thread::sleep_for(std::chrono::duration<double>(pre_grasp_dwell_));
    }
    feedback(goal_handle, "APPROACH", 0.45, "descending to the grasp pose");
    geometry_msgs::msg::Pose last_reached = pre_grasp;
    double contact_force = 0.0;
    bool descended = false;
    if (descent_mode_ == "single")
    {
        // One straight move: no stop-and-go, which the operator sees as stuttering.
        // Contact is still checked right after the move, so a bad height is caught, just
        // one step later.
        descended = cartesian_move({pre_grasp, grasp_pose}, "APPROACH", error);
        last_reached = grasp_pose;
    }
    else
    {
        descended = descend_with_contact_stop(
            pre_grasp, grasp_pose, error, last_reached, contact_force);
    }
    if (!descended)
    {
        if (error.rfind("touched between", 0) == 0)
        {
            // Release the pressure first, then report: the height is now measured.
            RCLCPP_ERROR(
                get_logger(),
                "Contact at z=%.4f m (%.1f N); retreating instead of pressing on",
                last_reached.position.z, contact_force);
            geometry_msgs::msg::Pose retreat = last_reached;
            retreat.position.z += 0.03;
            std::string retreat_error;
            cartesian_move({last_reached, retreat}, "RETREAT", retreat_error);
            abort_with("CONTACT_DETECTED", error);
        }
        else
        {
            abort_with("EXEC_FAILED", error);
        }
        return;
    }
    if (descent_mode_ == "single" && contact_stop_)
    {
        contact_force = latest_force_.load();
        if (contact_force > contact_force_n_)
        {
            geometry_msgs::msg::Pose retreat = grasp_pose;
            retreat.position.z += 0.03;
            std::string retreat_error;
            cartesian_move({grasp_pose, retreat}, "RETREAT", retreat_error);
            abort_with(
                "CONTACT_DETECTED",
                "force " + std::to_string(contact_force) + " N right after the descent; "
                "the fingers are pressing the surface, raise table_z");
            return;
        }
    }

    // ---- GRASP ----
    const double force = goal->grasp_force > 0.0 ? goal->grasp_force : grasp_force_;
    feedback(goal_handle, "GRASP", 0.55, "closing the gripper with " + std::to_string(force) + " N");
    double measured_width = 0.0;
    if (!gripper_grasp(force, error, measured_width))
    {
        abort_with("GRASP_FAILED", error);
        return;
    }
    result->measured_width = measured_width;
    // The gripper reports how far it actually closed. On a 40 mm cube it must stop around
    // 40 mm; closing to "nothing" means the fingers missed the object entirely, which
    // used to be reported as a success.
    const bool width_ok = measured_width >= min_grasp_width_;
    const bool force_ok = last_grasp_force_ >= min_grasp_force_;
    if (!width_ok && !force_ok)
    {
        abort_with(
            "GRASP_FAILED",
            "closed to " + std::to_string(static_cast<int>(measured_width * 1000.0))
                + " mm / " + std::to_string(static_cast<int>(last_grasp_force_))
                + " N, below both thresholds, nothing between the fingers; "
                "lower table_z or check the pose");
        return;
    }
    if (canceled(goal_handle))
    {
        result->message = "CANCELED";
        goal_handle->canceled(result);
        return;
    }

    // ---- LIFT ----
    geometry_msgs::msg::Pose lifted = grasp_pose;
    lifted.position.x -= approach_dir.x() * lift_height_;
    lifted.position.y -= approach_dir.y() * lift_height_;
    lifted.position.z -= approach_dir.z() * lift_height_;
    feedback(goal_handle, "LIFT", 0.65, "lifting the object");
    if (!cartesian_move({grasp_pose, lifted}, "LIFT", error))
    {
        abort_with("EXEC_FAILED", error);
        return;
    }

    // ---- PLACE ----
    const bool goal_has_place_pose = goal->place_pose.pose.position.x != 0.0
        || goal->place_pose.pose.position.y != 0.0 || goal->place_pose.pose.position.z != 0.0;

    geometry_msgs::msg::Pose place_pose;
    if (goal_has_place_pose)
    {
        place_pose = goal->place_pose.pose;
    }
    else if (place_mode_ == "relative")
    {
        // Place relative to where the cube was actually picked up. Orientation is
        // inherited from the grasp so the wrist does not twist while carrying, and the
        // height comes from the configured place pose (the table is flat, the detected
        // depth is not accurate enough to be trusted for the release height).
        place_pose = grasp_pose;
        if (place_offset_.size() >= 2)
        {
            place_pose.position.x += place_offset_[0];
            place_pose.position.y += place_offset_[1];
        }
        // The cube keeps its offset from the TCP once grasped, so releasing at the grasp
        // height puts it back on the same surface. A configured height would press it in.
        place_pose.position.z = (place_height_mode_ == "configured")
            ? place_pose_.position.z
            : grasp_pose.position.z + place_z_offset_;
        RCLCPP_INFO(
            get_logger(),
            "Placing at grasp + [%.3f, %.3f] m -> [%.3f, %.3f, %.3f]",
            place_offset_.size() > 0 ? place_offset_[0] : 0.0,
            place_offset_.size() > 1 ? place_offset_[1] : 0.0,
            place_pose.position.x, place_pose.position.y, place_pose.position.z);
    }
    else
    {
        place_pose = place_pose_;
    }
    feedback(goal_handle, "PLACE", 0.75, "moving to the place pose");
    geometry_msgs::msg::Pose pre_place = place_pose;
    pre_place.position.z += place_release_height_;
    if (!move_to_pose(pre_place, "PLACE", error))
    {
        abort_with("PLAN_FAILED", error);
        return;
    }
    feedback(goal_handle, "PLACE", 0.85, "descending to the release pose");
    if (!cartesian_move({pre_place, place_pose}, "PLACE", error))
    {
        abort_with("EXEC_FAILED", error);
        return;
    }

    // ---- RELEASE ----
    feedback(goal_handle, "RELEASE", 0.92, "opening the gripper");
    if (!gripper_move(open_width_, error))
    {
        abort_with("GRASP_FAILED", error);
        return;
    }

    // ---- RETREAT ----
    // Only cosmetic: the object is already released. A Cartesian retreat can fail
    // when the arm sits close to a joint limit at the place pose, so plan it with
    // OMPL and treat a failure as a warning rather than aborting the whole run.
    feedback(goal_handle, "RETREAT", 0.97, "retreating");
    if (!move_to_pose(pre_place, "RETREAT", error))
    {
        RCLCPP_WARN(
            get_logger(), "Retreat failed (%s); staying at the release pose", error.c_str());
        result->success = true;
        result->message = "ok (retreat skipped: " + error + ")";
        goal_handle->succeed(result);
        return;
    }

    result->success = true;
    result->message = "ok";
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Pick-and-place finished successfully");
}

}  // namespace flexiv_grasp_demo

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<flexiv_grasp_demo::GraspTaskNode>();

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });

    node->initialize();

    // The executor thread above does the spinning; block here until Ctrl-C.
    while (rclcpp::ok())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    rclcpp::shutdown();
    if (spinner.joinable())
    {
        spinner.join();
    }
    return 0;
}
