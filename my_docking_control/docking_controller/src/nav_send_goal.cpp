#include "controller.hpp"


void DockingController::send_goal(geometry_msgs::msg::Pose goal_pose, std::string command)
{
    auto nav_client = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    while (!nav_client->wait_for_action_server())
    {
        RCLCPP_INFO(get_logger(), "Waiting for action server...");
    }

    if (command == "cancel")
    {
        nav_client->async_cancel_all_goals();
    }
    else if (command == "send")
    {
        // Create goal
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.header.frame_id = "map";

        goal_msg.pose.pose.position.x = goal_pose.position.x;
        goal_msg.pose.pose.position.y = goal_pose.position.y;
        goal_msg.pose.pose.orientation.x = goal_pose.orientation.x;
        goal_msg.pose.pose.orientation.y = goal_pose.orientation.y;
        goal_msg.pose.pose.orientation.w = goal_pose.orientation.w;
        goal_msg.pose.pose.orientation.z = goal_pose.orientation.z;

        // Create goal optinos
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.feedback_callback = std::bind(&DockingController::navFeedbackCallback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&DockingController::navResultCallback, this, _1);
        
        // Send goal
        nav_client->async_send_goal(goal_msg, send_goal_options);
    }
}

void DockingController::navFeedbackCallback(GoalHandleNavigateToPose::SharedPtr,const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
    RCLCPP_INFO(get_logger(), "Distance remaining = %f", feedback->distance_remaining);
}

void DockingController::navResultCallback(const GoalHandleNavigateToPose::WrappedResult & result)
{
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Success!!!");
        goal_reached = true;
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(get_logger(), "Goal was aborted");
        return;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(get_logger(), "Goal was canceled");
        return;
    default:
        RCLCPP_ERROR(get_logger(), "Unknown result code");
        return;
    }
}


void DockingController::exit_dock()
{
    // Create waypoints
    geometry_msgs::msg::PoseStamped post_dock_1;
    post_dock_1.header.stamp = this->now();
    post_dock_1.header.frame_id = "map";
    post_dock_1.pose.position.x = 2;
    post_dock_1.pose.position.y = 2;
    post_dock_1.pose.orientation.z = 1;

    geometry_msgs::msg::PoseStamped post_dock_2;
    post_dock_2.header.stamp = this->now();
    post_dock_2.header.frame_id = "map";
    post_dock_2.pose.position.x = initial_pose.position.x;
    post_dock_2.pose.position.y = initial_pose.position.y;
    post_dock_2.pose.orientation.w = initial_pose.orientation.w;
    post_dock_2.pose.orientation.x = initial_pose.orientation.x;
    post_dock_2.pose.orientation.y = initial_pose.orientation.y;
    post_dock_2.pose.orientation.z = initial_pose.orientation.z;

    std::vector<geometry_msgs::msg::PoseStamped> waypoints = {post_dock_1, post_dock_2};

    // Create ation msg
    auto waypoint_msg = FollowWaypoints::Goal();
    waypoint_msg.poses = waypoints;
    
    // Create action client
    auto waypoint_client = rclcpp_action::create_client<FollowWaypoints>(this, "FollowWaypoints");

    // Wait for action server
    while (!waypoint_client->wait_for_action_server())
    {
        RCLCPP_INFO(get_logger(), "Waiting for action server...");
    }

    // Send goal to action servevr
    waypoint_client->async_send_goal(waypoint_msg);
}