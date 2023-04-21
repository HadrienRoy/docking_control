#include "controller.hpp"

void DockingController::docking_state_manager()
{
    if (docking_state == "start")
    {
        start_state_func();
    }

    if (docking_state == "searching")
    {
        searching_state_func();
    }

    if (docking_state == "approach")
    {
        approach_state_func();
    }

    if (docking_state == "final_approach")
    {
        final_approach_state_func();
    }

    if (docking_state == "docked")
    {
        docked_state_func();
    }
}

/*** Docking State Functions ***/
void DockingController::start_state_func()
{
    // Stop Turtlebot
    turtlebot_stop();

    if (ready_turtle_pose && battery_received)
    {
        // Call Queue Update Server to add new robot to queue 
        // and get docking instructions
        threads.push_back(std::thread(std::bind(&DockingController::queue_update_client, this, "add_new_robot")));

        // After response is valid
        // move to specified location

        set_docking_state("searching");
        battery_received = false;
    }
   
}

void DockingController::searching_state_func()
{
    if (!ready_tag_pose)
    {
        RCLCPP_INFO_ONCE(get_logger(),"Searching for AprilTag.");
        turtlebot_turn_velocity(vel_angular);
        return;
    }

    turtlebot_stop();

    // Get a good tag estimation
    if (tag_counter < 10)
    {
        RCLCPP_INFO_ONCE(get_logger(),"tag_x position: %f", tag_x);
        RCLCPP_INFO_ONCE(get_logger(),"tag_y position: %f", tag_y);
        return;
    }

    approach_goal_pose.position.x = tag_x - 1.0;
    approach_goal_pose.position.y = tag_y;
    RCLCPP_INFO(get_logger(), "approach_goal x: %f", approach_goal_pose.position.x);
    RCLCPP_INFO(get_logger(), "approach_goal y: %f", approach_goal_pose.position.y);

    final_approach_goal_pose.position.x = tag_x - 0.65;
    final_approach_goal_pose.position.y = tag_y;
    RCLCPP_INFO(get_logger(), "final_approach_goal x: %f", final_approach_goal_pose.position.x);
    RCLCPP_INFO(get_logger(), "final_approach_goal y: %f", final_approach_goal_pose.position.y);

    RCLCPP_INFO(get_logger(), "turtle x: %f", turtle_x);
    RCLCPP_INFO(get_logger(), "turtle y: %f", turtle_y);
    RCLCPP_INFO(get_logger(), "turtle y: %f", turtle_theta);

    set_docking_state("approach");

    ready_tag_pose = false;
}

void DockingController::approach_state_func()
{
    if (!ready_turtle_pose)
    {
        RCLCPP_INFO(get_logger(), "Turtle Pose not ready");
        return;
    }

    RCLCPP_INFO(get_logger(), "approach_distance goal: %f", distance(approach_goal_pose));

    if (distance(approach_goal_pose) >= approach_distance_tolerance)
    {
        vel_msg.linear.x = linear_velocity(approach_goal_pose);
        vel_msg.linear.y = 0.0;
        vel_msg.linear.z = 0.0;

        vel_msg.angular.x = 0.0;
        vel_msg.angular.y = 0.0;
        vel_msg.angular.z = angular_velocity(approach_goal_pose);

        vel_publisher->publish(vel_msg);
    }
    else
    {
        turtlebot_stop();
        set_docking_state("final_approach");
    }

    ready_turtle_pose = false;
    ready_tag_pose = false;
}

void DockingController::final_approach_state_func()
{
    if (!ready_turtle_pose)
    {
        return;
    }
    

    if (abs(steering_angle(final_approach_goal_pose) - turtle_theta) >= angle_tolerance)
    {
        // RCLCPP_INFO(get_logger(),"robot theta position: %f", turtle_theta);
        
        vel_msg.angular.z = 2.5 * (steering_angle(final_approach_goal_pose)-turtle_theta);
        vel_publisher->publish(vel_msg);
    }
    else
    {
        // RCLCPP_INFO(get_logger(), "final_fapproach_distance goal: %f", distance(final_approach_goal_pose));

        if (distance(final_approach_goal_pose) >= final_approach_distance_tolerance)
        {
            vel_msg.angular.z = 0;
            vel_msg.linear.x = 0.1;
            vel_publisher->publish(vel_msg);
        }
        else
        {
            turtlebot_stop();
            set_docking_state("docked");
        }
    }

    ready_turtle_pose = false;
    ready_tag_pose = false;
}

void DockingController::docked_state_func()
{
    turtlebot_stop();

    // gazebo_charge_battery_client();

    threads.push_back(std::thread(std::bind(&DockingController::queue_update_client, this, "state_change")));

    start_tag_detection = false;
}

/*** Calculation Functions ***/
double DockingController::distance(geometry_msgs::msg::Pose goal_pose)
{
    return sqrt(pow((goal_pose.position.x - turtle_x), 2) +
                pow((goal_pose.position.y - turtle_y), 2));
}

double DockingController::linear_velocity(geometry_msgs::msg::Pose goal_pose)
{
    return 0.5 * distance(goal_pose);
}

double DockingController::angular_velocity(geometry_msgs::msg::Pose goal_pose)
{
    return 4.0 * (steering_angle(goal_pose) - turtle_theta);
}

double DockingController::steering_angle(geometry_msgs::msg::Pose goal_pose)
{
    return atan2(goal_pose.position.y - turtle_y, goal_pose.position.x - turtle_x);
}

/*** Turtlebot3 Movement Functions ***/
void DockingController::turtlebot_stop()
{
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;

    vel_publisher->publish(vel_msg);
}

void DockingController::turtlebot_move(double velocity, double radians)
{
    vel_msg.linear.x = velocity;
    vel_msg.angular.z = radians;

    vel_publisher->publish(vel_msg);
}

void DockingController::turtlebot_turn(double radians)
{
    // double time = radians / vel_angular; // time

    // if (tag_y < 0)
    // {
    //     vel_msg.angular.z = vel_angular;
    // }
    // else
    // {
    //     vel_msg.angular.z = -vel_angular;
    // }
    vel_msg.angular.z = radians;

    vel_publisher->publish(vel_msg);
}

void DockingController::turtlebot_turn_velocity(double angular_velocity)
{
    vel_msg.linear.x = 0;
    vel_msg.angular.z = angular_velocity;

    vel_publisher->publish(vel_msg);
}

void DockingController::turtlebot_forward(double velocity)
{
    vel_msg.linear.x = velocity;
    vel_msg.angular.z = 0;

    vel_publisher->publish(vel_msg);
}

/*** Set & Get Functions ***/
void DockingController::set_docking_state(std::string new_docking_state)
{

    if (docking_state != new_docking_state)
    {
        last_docking_state = docking_state;
        docking_state = new_docking_state;
        RCLCPP_INFO(get_logger(), "Docking state: %s, Last state: %s", docking_state.c_str(), last_docking_state.c_str());
    }
}

void DockingController::get_last_tag_pose()
{
    if (ready_tag_pose)
    {
        ready_tag_pose = false;     // Wait for next AprilTag Pose
    }
}

void DockingController::publish_state()
{
    docking_interfaces::msg::CurrentState current_state;

    current_state.docking_state = docking_state;
    state_publisher->publish(current_state);
}

void DockingController::gazebo_charge_battery_client()
{
    auto client = this->create_client<docking_interfaces::srv::GazeboChargeBattery>("gazebo_battery_state/charge_battery_service");
    while (!client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(this->get_logger(), "Waiting for Battery Charging server to be up...");
    }

    auto request = std::make_shared<docking_interfaces::srv::GazeboChargeBattery::Request>();
    request->command = "charge";

    auto future = client->async_send_request(request);

    try
    {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Charging Successful.");
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Service call failed");
    }
}

/*** Main Run Function ***/
void DockingController::run()
{
    // get_last_tag_pose();     // Get last pose of AprilTag
    docking_state_manager(); // Manage docking states
    publish_state();         // Publish state to /current_state topic
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DockingController>();

    rclcpp::Rate rate(30.0);

    node->set_docking_state("");

    while (rclcpp::ok())
    {
        node->run();
        rclcpp::spin_some(node);
        // rclcpp::spin(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}