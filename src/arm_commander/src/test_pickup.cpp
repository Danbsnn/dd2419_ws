#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <optional>
#include <mutex>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>

using namespace std::chrono_literals;

// ─────────────────────────────────────────────────────────────
// Pixel bounds
// ─────────────────────────────────────────────────────────────
constexpr double PX_X_MIN       = 240.0;
constexpr double PX_X_MAX       = 383.0;

constexpr double PX_Y_MIN       = 351.0;
constexpr double PX_Y_MAX       = 479.0;

constexpr int    MAX_ITER       = 60;

constexpr double GRASP_Z_OFFSET = 0.05;

constexpr double BASE_STEP      = 0.003;
constexpr double ELBOW_STEP     = 0.003;

// ─────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────
bool planExecute(
    moveit::planning_interface::MoveGroupInterface &group)
{
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    if (group.plan(plan) !=
        moveit::core::MoveItErrorCode::SUCCESS)
    {
        return false;
    }

    if (group.execute(plan) !=
        moveit::core::MoveItErrorCode::SUCCESS)
    {
        return false;
    }

    return true;
}

bool moveNamed(
    moveit::planning_interface::MoveGroupInterface &group,
    const std::string &name)
{
    group.setStartStateToCurrentState();

    group.setNamedTarget(name);

    return planExecute(group);
}

// ─────────────────────────────────────────────────────────────
// Cube structure
// ─────────────────────────────────────────────────────────────
struct CubePose
{
    std::string color;

    double x;
    double y;
    double z;

    double yaw;
};

double getYaw(
    const geometry_msgs::msg::Quaternion &q)
{
    return atan2(
        2.0 * (q.w * q.z),
        1.0 - 2.0 * (q.z * q.z));
}

std::optional<CubePose> extractCube(
    const visualization_msgs::msg::MarkerArray::SharedPtr &msg)
{
    for (const auto &marker : msg->markers)
    {
        if (marker.action ==
            visualization_msgs::msg::Marker::DELETE)
        {
            continue;
        }

        if (marker.id % 2 != 0)
        {
            continue;
        }

        CubePose cp;

        cp.x   = marker.pose.position.x;
        cp.y   = marker.pose.position.y;
        cp.z   = marker.pose.position.z;

        cp.yaw = getYaw(marker.pose.orientation);

        switch (marker.id)
        {
            case 0:
                cp.color = "red";
                break;

            case 2:
                cp.color = "green";
                break;

            case 4:
                cp.color = "blue";
                break;

            default:
                cp.color = "unknown";
                break;
        }

        return cp;
    }

    return std::nullopt;
}

// ─────────────────────────────────────────────────────────────
// CENTER CUBE
//
// STAGE 1:
// Adjust BASE only until X aligned
//
// STAGE 2:
// Adjust ELBOW only until Y aligned
//
// joints[0] -> base
// joints[3] -> elbow
// joints[4] -> wrist rotate fixed at zero
// ─────────────────────────────────────────────────────────────
bool centerCube(
    moveit::planning_interface::MoveGroupInterface &arm,
    rclcpp::Node::SharedPtr                        node,
    double                                         &px_x_ref,
    double                                         &px_y_ref,
    std::mutex                                     &px_mutex)
{
    // ─────────────────────────────────────────
    // Force wrist rotate joint to zero
    // joints[4]
    // ─────────────────────────────────────────
    {
        auto joints = arm.getCurrentJointValues();

        joints[4] = 0.0;

        arm.setStartStateToCurrentState();

        arm.setJointValueTarget(joints);

        if (!planExecute(arm))
        {
            RCLCPP_WARN(
                node->get_logger(),
                "Failed to zero wrist rotate joint.");
        }

        std::this_thread::sleep_for(500ms);
    }

    // ========================================================
    // STAGE 1
    // BASE ALIGNMENT
    // ========================================================
    RCLCPP_INFO(
        node->get_logger(),
        "Stage 1: Base alignment");

    for (int iter = 0; iter < MAX_ITER; ++iter)
    {
        double px;
        double py;

        {
            std::lock_guard<std::mutex> lock(px_mutex);

            px = px_x_ref;
            py = px_y_ref;
        }

        if (px < 0.0)
        {
            RCLCPP_WARN(
                node->get_logger(),
                "No pixel data.");

            std::this_thread::sleep_for(200ms);

            continue;
        }

        bool x_ok =
            (px > PX_X_MIN && px < PX_X_MAX);

        if (x_ok)
        {
            RCLCPP_INFO(
                node->get_logger(),
                "Base aligned! px = %.1f",
                px);

            break;
        }

        auto joints = arm.getCurrentJointValues();

        // keep wrist fixed
        joints[4] = 0.0;

        // ─────────────────────────────────────
        // Base correction only
        // joints[0]
        // ─────────────────────────────────────
        if (px >= PX_X_MAX)
        {
            RCLCPP_INFO(
                node->get_logger(),
                "[iter %d] px %.1f -> BASE RIGHT",
                iter,
                px);

            joints[0] += BASE_STEP;
        }
        else if (px <= PX_X_MIN)
        {
            RCLCPP_INFO(
                node->get_logger(),
                "[iter %d] px %.1f -> BASE LEFT",
                iter,
                px);

            joints[0] -= BASE_STEP;
        }

        arm.setStartStateToCurrentState();

        arm.setJointValueTarget(joints);

        if (!planExecute(arm))
        {
            RCLCPP_WARN(
                node->get_logger(),
                "Base adjustment failed.");
        }

        std::this_thread::sleep_for(350ms);
    }

    // ========================================================
    // STAGE 2
    // ELBOW ALIGNMENT
    // ========================================================
    RCLCPP_INFO(
        node->get_logger(),
        "Stage 2: Elbow alignment");

    for (int iter = 0; iter < MAX_ITER; ++iter)
    {
        double px;
        double py;

        {
            std::lock_guard<std::mutex> lock(px_mutex);

            px = px_x_ref;
            py = px_y_ref;
        }

        bool y_ok =
            (py > PX_Y_MIN && py < PX_Y_MAX);

        if (y_ok)
        {
            RCLCPP_INFO(
                node->get_logger(),
                "Elbow aligned! py = %.1f",
                py);

            return true;
        }

        auto joints = arm.getCurrentJointValues();

        // keep wrist fixed
        joints[4] = 0.0;

        // ─────────────────────────────────────
        // Elbow correction only
        // joints[3]
        // ─────────────────────────────────────
        if (py >= PX_Y_MAX)
        {
            RCLCPP_INFO(
                node->get_logger(),
                "[iter %d] py %.1f -> ELBOW DOWN",
                iter,
                py);

            joints[3] += ELBOW_STEP;
        }
        else if (py <= PX_Y_MIN)
        {
            RCLCPP_INFO(
                node->get_logger(),
                "[iter %d] py %.1f -> ELBOW UP",
                iter,
                py);

            joints[3] -= ELBOW_STEP;
        }

        arm.setStartStateToCurrentState();

        arm.setJointValueTarget(joints);

        if (!planExecute(arm))
        {
            RCLCPP_WARN(
                node->get_logger(),
                "Elbow adjustment failed.");
        }

        std::this_thread::sleep_for(350ms);
    }

    RCLCPP_ERROR(
        node->get_logger(),
        "Failed to center cube.");

    return false;
}

// ─────────────────────────────────────────────────────────────
// Main
// ─────────────────────────────────────────────────────────────
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<rclcpp::Node>("test_pick");

    rclcpp::executors::SingleThreadedExecutor executor;

    executor.add_node(node);

    std::thread([&executor]()
    {
        executor.spin();
    }).detach();

    // ─────────────────────────────────────────
    // Cube subscriber
    // ─────────────────────────────────────────
    std::optional<CubePose> latest_cube;

    std::mutex cube_mutex;

    auto cube_sub =
        node->create_subscription<
            visualization_msgs::msg::MarkerArray>(
            "cubes",
            10,
            [&](const visualization_msgs::msg::MarkerArray::SharedPtr msg)
            {
                auto result = extractCube(msg);

                std::lock_guard<std::mutex> lock(cube_mutex);

                latest_cube = result;
            });

    // ─────────────────────────────────────────
    // Pixel subscriber
    // ─────────────────────────────────────────
    double raw_px_x = -1.0;
    double raw_px_y = -1.0;

    std::mutex px_mutex;

    auto px_sub =
        node->create_subscription<geometry_msgs::msg::Point>(
            "center_raw",
            10,
            [&](const geometry_msgs::msg::Point::SharedPtr msg)
            {
                std::lock_guard<std::mutex> lock(px_mutex);

                raw_px_x = msg->x;
                raw_px_y = msg->y;
            });

    // ─────────────────────────────────────────
    // MoveIt groups
    // ─────────────────────────────────────────
    moveit::planning_interface::MoveGroupInterface arm(
        node,
        "arm");

    moveit::planning_interface::MoveGroupInterface gripper(
        node,
        "gripper");

    arm.setMaxVelocityScalingFactor(0.4);
    arm.setMaxAccelerationScalingFactor(0.4);

    arm.setGoalPositionTolerance(0.02);
    arm.setGoalJointTolerance(0.05);

    gripper.setMaxVelocityScalingFactor(0.5);
    gripper.setMaxAccelerationScalingFactor(0.5);

    // ─────────────────────────────────────────
    // Open gripper
    // ─────────────────────────────────────────
    moveNamed(gripper, "gripper_open");

    std::this_thread::sleep_for(500ms);

    // ─────────────────────────────────────────
    // Detection pose
    // ─────────────────────────────────────────
    moveNamed(arm, "detection");

    std::this_thread::sleep_for(1500ms);

    // ─────────────────────────────────────────
    // Get cube
    // ─────────────────────────────────────────
    std::optional<CubePose> cube;

    {
        std::lock_guard<std::mutex> lock(cube_mutex);

        cube = latest_cube;
    }

    if (!cube.has_value())
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "No cube detected.");

        rclcpp::shutdown();

        return 0;
    }

    RCLCPP_INFO(
        node->get_logger(),
        "Cube detected: %s (%.3f %.3f %.3f)",
        cube->color.c_str(),
        cube->x,
        cube->y,
        cube->z);

    // ─────────────────────────────────────────
    // Move near cube using IK
    // ─────────────────────────────────────────
    arm.setStartStateToCurrentState();

    arm.setPositionTarget(
        cube->x,
        cube->y,
        cube->z + GRASP_Z_OFFSET,
        "grasping_frame");

    if (!planExecute(arm))
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "Failed to move near cube.");

        rclcpp::shutdown();

        return 0;
    }

    std::this_thread::sleep_for(700ms);

    // ─────────────────────────────────────────
    // Fine alignment
    // ─────────────────────────────────────────
    if (!centerCube(
            arm,
            node,
            raw_px_x,
            raw_px_y,
            px_mutex))
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "Centering failed.");

        rclcpp::shutdown();

        return 0;
    }

    // ─────────────────────────────────────────
    // Close gripper
    // ─────────────────────────────────────────
    moveNamed(gripper, "gripper_closed");

    std::this_thread::sleep_for(500ms);

    // ─────────────────────────────────────────
    // Return to detection pose
    // ─────────────────────────────────────────
    moveNamed(arm, "detection");

    rclcpp::shutdown();

    return 0;
}