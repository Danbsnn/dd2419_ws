#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <optional>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

#include "robp_interfaces/srv/arm_command.hpp"

using namespace std::chrono_literals;
using ArmCommand = robp_interfaces::srv::ArmCommand;

// ─────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────
bool planExecute(moveit::planning_interface::MoveGroupInterface &group)
{
    const int MAX_ATTEMPTS = 10;

    for (int i = 0; i < MAX_ATTEMPTS; i++)
    {
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        if (group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_WARN(rclcpp::get_logger("planExecute"),
                        "Planning failed (attempt %d)", i + 1);
            continue;
        }

        if (group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS)
        {
            return true;
        }

        RCLCPP_WARN(rclcpp::get_logger("planExecute"),
                    "Execution failed (attempt %d)", i + 1);
    }

    return false;
}

bool moveNamed(moveit::planning_interface::MoveGroupInterface &group,
               const std::string &name)
{
    group.setStartStateToCurrentState();
    group.setNamedTarget(name);
    return planExecute(group);
}

// ─────────────────────────────────────────
// Cube structure
// ─────────────────────────────────────────
struct CubePose
{
    std::string color;
    double x, y, z;
};

std::optional<CubePose> extractCube(
    const visualization_msgs::msg::MarkerArray::SharedPtr &msg)
{
    for (const auto &marker : msg->markers)
    {
        if (marker.action == visualization_msgs::msg::Marker::DELETE)
            continue;
        if (marker.id % 2 != 0)
            continue;

        CubePose cp;
        cp.x = marker.pose.position.x;
        cp.y = marker.pose.position.y;
        cp.z = marker.pose.position.z;

        switch (marker.id)
        {
            case 0: cp.color = "red";      break;
            case 2: cp.color = "green";    break;
            case 4: cp.color = "blue";     break;
            default: cp.color = "unknown"; break;
        }
        return cp;
    }
    return std::nullopt;
}

// ─────────────────────────────────────────
// SERVICE NODE
// ─────────────────────────────────────────
class ArmServiceNode : public rclcpp::Node
{
public:
    ArmServiceNode() : Node("arm_service")
    {
        // Reentrant callback group so cube_callback can fire
        // even while handle_command is blocking inside the executor
        cb_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);

        rclcpp::SubscriptionOptions opts;
        opts.callback_group = cb_group_;

        sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "cubes", 10,
            std::bind(&ArmServiceNode::cube_callback, this, std::placeholders::_1),
            opts);

        service_ = this->create_service<ArmCommand>(
            "arm_command",
            std::bind(&ArmServiceNode::handle_command, this,
                      std::placeholders::_1,
                      std::placeholders::_2));

        init_timer_ = this->create_wall_timer(
            500ms,
            std::bind(&ArmServiceNode::init_moveit, this));

        RCLCPP_INFO(this->get_logger(), "Arm Service Node started");
    }

private:
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_;
    rclcpp::Service<ArmCommand>::SharedPtr service_;
    rclcpp::TimerBase::SharedPtr init_timer_;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_;

    std::optional<CubePose> latest_cube_;
    std::mutex cube_mutex_;

    // ─────────── CALLBACKS ───────────
    void cube_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
        auto result = extractCube(msg);
        std::lock_guard<std::mutex> lock(cube_mutex_);
        latest_cube_ = result;
    }

    void init_moveit()
    {
        if (arm_ != nullptr)
            return;

        RCLCPP_INFO(this->get_logger(), "Initializing MoveIt...");

        arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "arm");

        gripper_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "gripper");

        arm_->setMaxVelocityScalingFactor(0.5);
        arm_->setMaxAccelerationScalingFactor(0.5);
        arm_->setGoalPositionTolerance(0.001);

        gripper_->setMaxVelocityScalingFactor(0.6);
        gripper_->setMaxAccelerationScalingFactor(0.6);

        init_timer_->cancel();

        RCLCPP_INFO(this->get_logger(), "MoveIt ready");
    }

    void handle_command(
        const std::shared_ptr<ArmCommand::Request> req,
        std::shared_ptr<ArmCommand::Response> res)
    {
        if (!arm_ || !gripper_)
        {
            res->success = false;
            res->message = "MoveIt not ready";
            return;
        }

        if (req->command == "pick")
        {
            res->success = execute_pick();
            res->message = res->success ? "Pick done" : "Pick failed";
        }
        else if (req->command == "drop")
        {
            res->success = execute_drop();
            res->message = res->success ? "Drop done" : "Drop failed";
        }
        else
        {
            res->success = false;
            res->message = "Invalid command";
        }
    }

    // ─────────── FRESH CUBE DETECTION ───────────
    // Flushes stale data, then waits for target_msgs fresh detections.
    // cube_callback fires concurrently on the reentrant cb_group_ thread
    // via the MultiThreadedExecutor while this function sleeps.
    std::optional<CubePose> wait_for_fresh_cube(int target_msgs = 10)
    {
        // 1. Flush any stale data accumulated before the arm settled
        {
            std::lock_guard<std::mutex> lock(cube_mutex_);
            latest_cube_ = std::nullopt;
        }

        RCLCPP_INFO(this->get_logger(),
                    "Flushed stale cube data. Waiting for %d fresh detections...",
                    target_msgs);

        // 2. Sleep in small steps; cube_callback fires concurrently
        //    on the MultiThreadedExecutor and updates latest_cube_
        int fresh_count = 0;
        const int MAX_WAIT_MS = 5000; // give up after 5 s
        int elapsed_ms = 0;

        while (rclcpp::ok() && fresh_count < target_msgs && elapsed_ms < MAX_WAIT_MS)
        {
            std::this_thread::sleep_for(50ms);
            elapsed_ms += 50;

            std::lock_guard<std::mutex> lock(cube_mutex_);
            if (latest_cube_.has_value())
                fresh_count++;
        }

        RCLCPP_INFO(this->get_logger(),
                    "Collected %d/%d fresh detections in %d ms",
                    fresh_count, target_msgs, elapsed_ms);

        std::lock_guard<std::mutex> lock(cube_mutex_);
        return latest_cube_;
    }

    // ─────────── PICK ───────────
    bool execute_pick()
    {
        // 1. Open gripper
        moveNamed(*gripper_, "gripper_open");
        std::this_thread::sleep_for(500ms);

        // 2. Move arm to detection pose and let it physically settle
        moveNamed(*arm_, "detection");
        std::this_thread::sleep_for(2000ms);

        // 3. Collect fresh cube detections (flushes stale data first)
        auto cube = wait_for_fresh_cube(10);

        if (!cube.has_value())
        {
            RCLCPP_ERROR(this->get_logger(), "No cube detected — aborting pick");
            return false;
        }

        RCLCPP_INFO(this->get_logger(),
                    "Fresh cube [%s] at (%.4f, %.4f, %.4f)",
                    cube->color.c_str(), cube->x, cube->y, cube->z);

        // 4. Move to cube position
        double gx = cube->x;
        double gy = cube->y;
        double gz = cube->z + 0.03;

        arm_->setStartStateToCurrentState();
        arm_->setPositionTarget(gx, gy, gz, "grasping_frame");

        if (!planExecute(*arm_))
            return false;

        std::this_thread::sleep_for(500ms);

        // 5. Close gripper
        moveNamed(*gripper_, "gripper_closed");
        std::this_thread::sleep_for(500ms);

        // 6. Return to rest
        moveNamed(*arm_, "rest");

        return true;
    }

    // ─────────── DROP ───────────
    bool execute_drop()
    {

        moveNamed(*arm_, "drop");
        std::this_thread::sleep_for(500ms);
        if (!planExecute(*arm_))
            return false;

        std::this_thread::sleep_for(500ms);

        moveNamed(*gripper_, "gripper_open");
        std::this_thread::sleep_for(500ms);

        moveNamed(*arm_, "rest");

        return true;
    }
};

// ─────────────────────────────────────────
// MAIN — MultiThreadedExecutor allows cube_callback
// to fire concurrently while handle_command blocks
// ─────────────────────────────────────────
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmServiceNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}