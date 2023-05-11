#include "controller.hpp"

void DockingController::docking_state_manager()
{
    if (docking_state == "wait")
    {
        // do nothing
    }

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

    if (docking_state == "queue_approach")
    {
        queue_approach_state_func();
    }

    if (docking_state == "in_queue")
    {
        in_queue_state_func();
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
        is_docking = true; // used for priority rank update
    }
   
}

void DockingController::searching_state_func()
{
    // if Battery% < 27% -> FAIL
    if (!battery_received )
    {
        return;
    }
    if (battery_received && (current_percent < 27.00))
    {
        RCLCPP_INFO_ONCE(get_logger(), "### BATTERY FAIL: %f ###", current_percent);
        return;
    }

    // Get turtle pose
    if (!ready_turtle_pose)
    {
        RCLCPP_INFO(get_logger(), "Turtle Pose not ready");
        return;
    }
   
    // If no queue number recieved then can't start next state
    if (!queue_state_received)
    {
        return;
    }
    last_queue_num = queue_num;
    queue_state_received = false;

    if (queue_state == "Docking")
    {
        set_docking_state("approach");  // set next state
        calculate_goal(queue_num);      // get approach goal
    }
    else if (queue_state == "Queuing")
    {
        set_docking_state("queue_approach");    // set next state
        calculate_goal(queue_num);              // calculate queue goal
    }
    
    ready_tag_pose = false;
    battery_received = false;
}

void DockingController::approach_state_func()
{
    // if Battery% < 27% -> FAIL
    // if (!battery_received )
    // {
    //     return;
    // }
    if (battery_received && (current_percent < 27.00))
    {
        RCLCPP_INFO_ONCE(get_logger(), "### BATTERY FAIL: %f ###", current_percent);
        return;
    }
    if (!ready_turtle_pose)
    {
        // RCLCPP_INFO(get_logger(), "Turtle Pose not ready");
        return;
    }
    

    // Get current time passed
    steady_clock::time_point time_passed = steady_clock::now();
    approach_time_sim_passed = duration_cast<duration<float>>(time_passed - approach_time_start).count();
    // RCLCPP_INFO(get_logger(), "###  time: %f ###", approach_time_passed);

    
    // If the time needed to go to AG is passed, move to next state
    if ((approach_time_sim_passed) >=  time_real_to_approach_goal/sim_time_dilation)
    {
        // RCLCPP_INFO_ONCE(get_logger(), "Approach Real Time Passed: %f", (approach_time_real_passed/sim_time_dilation));
        // RCLCPP_INFO_ONCE(get_logger(), "Approach Percent Used: %f", (approach_time_real_passed/31.2237*0.032022));
        
        set_docking_state("final_approach");
        final_approach_time_start = steady_clock::now();    // start clock for final approach to be able to simulate time needed
    }

 
    ready_turtle_pose = false;
    battery_received = false;
}

void DockingController::final_approach_state_func()
{
    // if Battery% < 27% -> FAIL
    // if (!battery_received )
    // {
    //     return;
    // }
    if (battery_received && (current_percent < 27.00))
    {
        RCLCPP_INFO_ONCE(get_logger(), "### BATTERY FAIL: %f ###", current_percent);
        return;
    }
    if (!ready_turtle_pose)
    {
        // RCLCPP_INFO(get_logger(), "Turtle Pose not ready");
        return;
    }

    // Get current time passed
    steady_clock::time_point time_passed = steady_clock::now();
    final_approach_time_sim_passed = duration_cast<duration<float>>(time_passed - final_approach_time_start).count();

    // 31.2237 = time dilation
    if (final_approach_time_sim_passed >= time_real_to_final_approach_goal/sim_time_dilation)
    {
        set_docking_state("docked");
        docked_time_start = steady_clock::now();
    }

 
    ready_turtle_pose = false;
    battery_received = false;
}

void DockingController::docked_state_func()
{
    turtlebot_stop();

    if (is_docking)
    {
        threads.push_back(std::thread(std::bind(&DockingController::queue_update_client, this, "state_change")));
        is_docking = false; 
    }

    // Get current time passed
    steady_clock::time_point time_passed = steady_clock::now();
    docked_time_passed = duration_cast<duration<float>>(time_passed - docked_time_start).count();
    // RCLCPP_INFO(get_logger(), "###  time: %f ###", docked_time_passed);

    // wait 60 second to account for charging time
    // 31.2237 = time dilation
    if (docked_time_passed >= time_to_charge/sim_time_dilation)
    {
        // send state change to pop robot off queue
        threads.push_back(std::thread(std::bind(&DockingController::queue_update_client, this, "state_change")));

        set_docking_state("wait");
    }

}

void DockingController::queue_approach_state_func()
{
    // if Battery% < 27% -> FAIL
    // if (!battery_received )
    // {
    //     return;
    // }
    if (battery_received && (current_percent < 27.00))
    {
        RCLCPP_INFO_ONCE(get_logger(), "### BATTERY FAIL: %f ###", current_percent);
        return;
    }
    if (!ready_turtle_pose)
    {
        // RCLCPP_INFO(get_logger(), "Turtle Pose not ready");
        return;
    }
   
    // If no queue number recieved then can't start next state
    // if (!queue_state_received)
    // {
    //     return;
    // }

    // Get current time passed
    steady_clock::time_point time_passed = steady_clock::now();
    queue_approach_time_sim_passed = duration_cast<duration<float>>(time_passed - queue_approach_time_start).count();

    
    // If the time needed to go to AG is passed, move to next state
    if ((queue_approach_time_sim_passed) >=  time_real_to_queue_approach_goal/sim_time_dilation)
    {
        // RCLCPP_INFO_ONCE(get_logger(), "Queue Approach Time Passed: %f", (queue_approach_time_passed*31.2237));
        // RCLCPP_INFO_ONCE(get_logger(), "Queue Approach Percent Used: %f", (queue_approach_time_passed*31.2237*0.032022));
        set_docking_state("in_queue");
        is_in_queue = true;
        turtlebot_stop();
    }

    // Check if docking
    if (queue_state == "Docking")
    {
        // calculate current distance from goal
        // current_distance = queue_approach_distance - queue_approach_time_sim_passed*0.032022/0.3467;
        current_distance = queue_approach_distance - queue_approach_time_sim_passed*sim_time_dilation*0.1;

        set_docking_state("approach");  // set next state
        calculate_goal(queue_num);      // get approach goal
    }

    
    battery_received = false;
    ready_turtle_pose = false;
}

void DockingController::in_queue_state_func()
{
    // if Battery% < 27% -> FAIL
    // if (!battery_received )
    // {
    //     return;
    // }
    if (battery_received && (current_percent < 27.00))
    {
        RCLCPP_INFO_ONCE(get_logger(), "### BATTERY FAIL: %f ###", current_percent);
        return;
    }
    if (!ready_turtle_pose)
    {
        // RCLCPP_INFO(get_logger(), "Turtle Pose not ready");
        return;
    }
   
    // Once arrived in queue, change state
    if (is_in_queue)
    {
        threads.push_back(std::thread(std::bind(&DockingController::queue_update_client, this, "state_change")));
        is_in_queue = false;
    }

    // If queue number has changed, move to new position
    // if (queue_state_received && (queue_state != "Docking"))
    // {
    //     // Get current time passed
    //     steady_clock::time_point time_passed = steady_clock::now();
    //     queue_approach_time_sim_passed = duration_cast<duration<float>>(time_passed - queue_approach_time_start).count();

        
    //     // If the time needed to go to AG is passed, move to next state
    //     // 31.2237 = time dilation
    //     if ((queue_approach_time_sim_passed) >=  time_real_to_queue_approach_goal/sim_time_dilation)
    //     {
    //         // RCLCPP_INFO_ONCE(get_logger(), "Queue Approach Time Passed: %f", (queue_approach_time_passed*31.2237));
    //         // RCLCPP_INFO_ONCE(get_logger(), "Queue Approach Percent Used: %f", (queue_approach_time_passed*31.2237*0.032022));
    //         queue_state_received = false;
    //     }
    // }

    // Check if docking
    if (queue_state == "Docking")
    {
        set_docking_state("approach");  // set next state
        calculate_goal(queue_num);      // get approach goal
    }

   battery_received = false;
   ready_turtle_pose = false;
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

void DockingController::calculate_goal(int queue_num)
{
    // If place in queue hasn't change, do nothing
    // if (last_queue_num == queue_num)
    // {
    //     new_queue_num = false;
    //     return;
    // }
    
    // If docking
    if (queue_num == 0)
    {
        approach_goal_pose.position.x = 2.0;
        approach_goal_pose.position.y = 0;

        // Calculate time needed to go to approach goal

        // TO DO: get rid of if and only keep else
        // On physical robot the turtle pose will be updated but not on sim without movement
        float approach_distance;
        if (last_queue_state == "")
        {
            approach_distance = distance(approach_goal_pose);
        }
        else
        {
            approach_distance = sim_distance(approach_goal_pose);
        }
        //approach_distance = distance(approach_goal_pose);

        // 0.1 m/s assumed vel , 0.032022 is [%/s] with time , 0.32022 is [%/m] when vel=0.1m/s
        time_real_to_approach_goal = approach_distance/velocity;
        // percent_to_approach_goal = approach_distance*0.32022;

        // RCLCPP_INFO(get_logger(), "Approach distance: %f", approach_distance);
        // RCLCPP_INFO(get_logger(), "Time to approach goal: %f", time_real_to_approach_goal);
        // RCLCPP_INFO_ONCE(get_logger(), "Percent to approach goal: %f", percent_to_approach_goal);

        // start approach clock
        approach_time_start = std::chrono::steady_clock::now();
    }
    else // queueing or in queue
    {
        queue_goal_pose.position.x = 2.0; 
        queue_goal_pose.position.y = -1.0 * queue_num;

        // Calculate time needed to go to approach goal
        queue_approach_distance = distance(queue_goal_pose);
        
        // 0.1 m/s assumed vel , 0.032022 is [%/s] with time , 0.32022 is [%/m] when vel=0.1m/s
        time_real_to_queue_approach_goal = queue_approach_distance/velocity;
        // percent_to_queue_approach_goal = queue_approach_distance*0.32022;

        // RCLCPP_INFO_ONCE(get_logger(), "Queue Approach distance: %f", queue_approach_distance);
        // RCLCPP_INFO_ONCE(get_logger(), "Queue Time to queue approach goal: %f", time_to_queue_approach_goal);
        // RCLCPP_INFO_ONCE(get_logger(), "Queue Percent to queue approach goal: %f", percent_to_queue_approach_goal);

        // start approach clock
        queue_approach_time_start = std::chrono::steady_clock::now();
    }

    // new_queue_num_rcv = true;
    // last_queue_num = queue_num;
}

float DockingController::sim_distance(geometry_msgs::msg::Pose goal_pose)
{
    // This is for the simulation because the robot do not move 
    // their last simulated queue position will be used


    if (last_queue_state == "Queued")
    {
        return sqrt(pow((goal_pose.position.x - 2.0), 2) +
                pow((goal_pose.position.y - (-1.0 )), 2));
    }
    else
    {
        return current_distance;
    }
    
}

void DockingController::set_queue_info(int new_queue_num, std::string new_queue_state)
{
    if (queue_num != new_queue_num)
    {
        last_queue_num = queue_num;
        queue_num = new_queue_num;
        // RCLCPP_INFO(get_logger(), "### %s Queue num: %d, Last num: %d ###", robot_id.c_str(), queue_num, last_queue_num);
    }

    if (queue_state != new_queue_state)
    {
        last_queue_state = queue_state;
        queue_state = new_queue_state;
        // RCLCPP_INFO(get_logger(), "### %s Queue state: %s, Last state: %s ###", robot_id.c_str(), queue_state.c_str(), last_queue_state.c_str());
    }

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
        // RCLCPP_INFO(get_logger(), "### %s Docking state: %s, Last state: %s ###", robot_id.c_str(), docking_state.c_str(), last_docking_state.c_str());
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