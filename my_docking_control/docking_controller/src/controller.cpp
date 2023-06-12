#include "controller.hpp"

void DockingController::docking_state_manager()
{
    if (docking_state == "")
    {
        turtlebot_turn_velocity(0.1);
        set_docking_state("wait");
    }

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

    if (docking_state == "initial_approach")
    {
        initial_approach_state_func();
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

    if (ready_turtle_pose && ready_battery_data)
    {
        // Call Queue Update Server to add new robot to queue/get dock commands
        threads.push_back(std::thread(std::bind(&DockingController::queue_update_client, this, "add_new_robot")));

        // After response is valid move to specified location

        // Set initial pose to return to when charging finished
        initial_pose.position.x = turtle_x;
        initial_pose.position.y = turtle_y;
        initial_pose.orientation.w = 0;
        initial_pose.orientation.x = 0;
        initial_pose.orientation.y = 0;
        initial_pose.orientation.z = 1;

        set_docking_state("searching");
 
        call_send_goal = true; // make available for search goal or origin
        start_tag_detection = true; // start AprilTag lookup

        ready_turtle_pose = false;
        ready_battery_data = false;
    }

}

void DockingController::searching_state_func()
{
    // Get turtle pose
    if (!ready_turtle_pose)
    {
        // RCLCPP_INFO(get_logger(), "Turtle Pose not ready");
        return;
    }
    else if (!ready_battery_data)
    {
        return;
    }
    // if (ready_battery_data_data && (current_percent < 27.00))
    // {
    //     RCLCPP_INFO_ONCE(get_logger(), "### BATTERY FAIL: %f ###", current_percent);
    //     return;
    // }

    // if tag not detected, do nothing
    if (!ready_tag_pose) 
    {   
        if (call_send_goal)
        {
            search_goal_pose.position.x = 0.0;
            search_goal_pose.position.y = 0.0;
            send_goal(search_goal_pose, "send");
            call_send_goal = false;
        }
        return;
    }

    // send_goal(search_goal_pose, "cancel");
    turtlebot_stop();
    
    // Get a good tag estimation
    if (!valid_tag_pose)
    {
        first_tag_pose  = true;
        return;
    }
    // If no queue number recieved then can't start next state
    if (!ready_queue_state)
    {
        return;
    }

    // Assign docking state depending on queue state
    if (queue_state == "Docking")
    {
        is_docking = true;
        set_docking_state("initial_approach");  // set next state
        
    }
    else if (queue_state == "Queuing")
    {
        set_docking_state("queue_approach");    // set next state    
                 
    }

    call_send_goal = true;
    
    ready_tag_pose = false;
    ready_battery_data = false;
    ready_queue_state = false;

    start_tag_detection = false;
}

void DockingController::initial_approach_state_func()
{
    // Get turtle pose
    if (!ready_turtle_pose)
    {
        // RCLCPP_INFO(get_logger(), "Turtle Pose not ready");
        return;
    }
    // else if (!ready_battery_data_data )
    // {
    //     return;
    // }
    // if (ready_battery_data && (current_percent < 27.00))
    // {
    //     RCLCPP_INFO_ONCE(get_logger(), "### BATTERY FAIL: %f ###", current_percent);
    //     return;
    // }
    

    // Go to initial approach goal
    if (call_send_goal)
    {
        calculate_goal(queue_num);
        send_goal(initial_approach_goal_pose, "send");
        call_send_goal = false;

        // next_tag_pose = false;
    }

    // For continuous tag if needed
    // if (next_tag_pose)
    // {
    //     calculate_goal(queue_num);
    //     send_goal(initial_approach_goal_pose, "send");
    //     next_tag_pose = false;
    // }
    
    // If robot has reached initial approach goal
    float distance_to_iap = distance(initial_approach_goal_pose);

    if (distance_to_iap < initial_approach_distance_tolerance)
    // if (goal_reached)
    {
        turtlebot_stop();
        set_docking_state("final_approach");

        // goal_reached = false;
        // final_approach_time_start = steady_clock::now();    // start clock for final approach to be able to simulate time needed
    }
    
    // Get current time passed
    // steady_clock::time_point time_passed = steady_clock::now();
    // approach_time_sim_passed = duration_cast<duration<float>>(time_passed - approach_time_start).count();
    // RCLCPP_INFO_ONCE(get_logger(), "Approach Real Time Passed: %f", (approach_time_real_passed/sim_time_dilation));
    // RCLCPP_INFO_ONCE(get_logger(), "Approach Percent Used: %f", (approach_time_real_passed/31.2237*0.032022));
        
    ready_turtle_pose = false;
    ready_battery_data = false;
}

void DockingController::final_approach_state_func()
{
    // Get turtle pose
    if (!ready_turtle_pose)
    {
        // RCLCPP_INFO(get_logger(), "Turtle Pose not ready");
        return;
    }
    // else if (!ready_battery_data )
    // {
    //     return;
    // }
    // if (ready_battery_data && (current_percent < 27.00))
    // {
    //     RCLCPP_INFO_ONCE(get_logger(), "### BATTERY FAIL: %f ###", current_percent);
    //     return;
    // }
    

    if (abs(steering_angle(final_approach_goal_pose) - turtle_theta) >= angle_tolerance)
    {
        // RCLCPP_INFO(get_logger(),"robot theta position: %f", turtle_theta);
        
        vel_msg.angular.z = 1.0 * (steering_angle(final_approach_goal_pose)-turtle_theta);
        vel_publisher->publish(vel_msg);
    }
    else
    {
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
            docked_time_start = steady_clock::now();
        }
    }

    // // Get current time passed
    // steady_clock::time_point time_passed = steady_clock::now();
    // final_approach_time_sim_passed = duration_cast<duration<float>>(time_passed - final_approach_time_start).count();

    // // 31.2237 = time dilation
    // if (final_approach_time_sim_passed >= time_real_to_final_approach_goal/sim_time_dilation)
    // {
    //     set_docking_state("docked");
    //     docked_time_start = steady_clock::now();
    // }

 
    ready_turtle_pose = false;
    ready_battery_data = false;
}

void DockingController::docked_state_func()
{
    // turtlebot_stop();

    if (is_docking)
    {
        threads.push_back(std::thread(std::bind(&DockingController::queue_update_client, this, "state_change")));
        is_docking = false; 
    }

    // Get current time passed
    steady_clock::time_point time_passed = steady_clock::now();
    docked_time_passed = duration_cast<duration<float>>(time_passed - docked_time_start).count();
    
    // wait 60 second to account for charging time, 31.2237 = time dilation
    if (docked_time_passed >= time_to_charge/sim_time_dilation)
    {
        // send state change to pop robot off queue
        threads.push_back(std::thread(std::bind(&DockingController::queue_update_client, this, "state_change")));

        set_docking_state("wait");

        exit_dock(); // if charge is complete, go to original position
    }
}

void DockingController::queue_approach_state_func()
{
    // Get turtle pose
    if (!ready_turtle_pose)
    {
        // RCLCPP_INFO(get_logger(), "Turtle Pose not ready");
        return;
    }
    // else if (!ready_battery_data )
    // {
    //     return;
    // }
    // if (ready_battery_data && (current_percent < 27.00))
    // {
    //     RCLCPP_INFO_ONCE(get_logger(), "### BATTERY FAIL: %f ###", current_percent);
    //     return;
    // }
    
   
    // If no queue number recieved then can't start next state
    // if (!queue_state_received)
    // {
    //     return;
    // }

    // Get current time passed
    // steady_clock::time_point time_passed = steady_clock::now();
    // queue_approach_time_sim_passed = duration_cast<duration<float>>(time_passed - queue_approach_time_start).count();
    // RCLCPP_INFO_ONCE(get_logger(), "Queue Approach Time Passed: %f", (queue_approach_time_passed*31.2237));
    // RCLCPP_INFO_ONCE(get_logger(), "Queue Approach Percent Used: %f", (queue_approach_time_passed*31.2237*0.032022));
 
    // Go to queue approach goal
    if (call_send_goal)
    {
        calculate_goal(queue_num);
        send_goal(queue_goal_pose, "send");
        call_send_goal = false;
    }

    // If robot has reached initial approach goal
    float distance_to_qip = distance(queue_goal_pose);

    if (distance_to_qip < initial_approach_distance_tolerance)
    {
        turtlebot_stop();
        set_docking_state("in_queue");

        is_in_queue = true;
    }

    // Check if docking
    if (queue_state == "Docking")
    {
        set_docking_state("initial_approach");  // set next state
        // calculate_goal(queue_num);      // get approach goal
        call_send_goal = true;
    }
    // else if (call_send_goal)
    // {
    //     calculate_goal(queue_num);
    //     send_goal(queue_goal_pose, "send");
    //     call_send_goal = false;
    // }
    else if (new_queue_num_rcv)
    {
        new_queue_num_rcv = false;      // get approach goal
        call_send_goal = true;
    }

    ready_battery_data = false;
    ready_turtle_pose = false;
}

void DockingController::in_queue_state_func()
{
    // Get turtle pose
    if (!ready_turtle_pose)
    {
        // RCLCPP_INFO(get_logger(), "Turtle Pose not ready");
        return;
    }
    // else if (!ready_battery_data )
    // {
    //     return;
    // }
    // if (ready_battery_data && (current_percent < 27.00))
    // {
    //     RCLCPP_INFO_ONCE(get_logger(), "### BATTERY FAIL: %f ###", current_percent);
    //     return;
    // }
    
   
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

    if (call_send_goal)
    {
        calculate_goal(queue_num);
        send_goal(queue_goal_pose, "send");
        call_send_goal = false;
    }


    // Check if docking
    if (queue_state == "Docking")
    {
        set_docking_state("initial_approach");  // set next state
        // calculate_goal(queue_num);      // get approach goal
        call_send_goal = true;
    }
    // else if (call_send_goal)
    // {
    //     calculate_goal(queue_num);
    //     send_goal(queue_goal_pose, "send");
    //     call_send_goal = false;
    // }
    // If not docking check if change to new queu position
    else if (new_queue_num_rcv)
    {
        new_queue_num_rcv = false;      // get approach goal
        // calculate_goal(queue_num); 
        call_send_goal = true;
    }
    

   ready_battery_data = false;
   ready_turtle_pose = false;
}

/*** 2d Docking State Functions ***/

void DockingController::docking_state_manager_2d()
{
    if (docking_state == "")
    {
        turtlebot_turn_velocity(0.1);
        set_docking_state("wait");
    }

    if (docking_state == "wait")
    {
        // do nothing
    }

    if (docking_state == "start")
    {
        start_state_func_2d();
    }

    if (docking_state == "searching")
    {
        searching_state_func_2d();
    }

    if (docking_state == "initial_approach")
    {
        initial_approach_state_func_2d();
    }

    if (docking_state == "final_approach")
    {
        final_approach_state_func_2d();
    }

    if (docking_state == "docked")
    {
        docked_state_func_2d();
    }

    if (docking_state == "queue_approach")
    {
        queue_approach_state_func_2d();
    }

    if (docking_state == "in_queue")
    {
        in_queue_state_func_2d();
    }
}

void DockingController::start_state_func_2d()
{
    // Stop Turtlebot
    turtlebot_stop();

    if (ready_2d_pose && ready_battery_data)
    {
        // Call Queue Update Server to add new robot to queue/get dock commands
        threads.push_back(std::thread(std::bind(&DockingController::queue_update_client, this, "add_new_robot")));

        // After response is valid move to specified location

        // Set initial pose to return to when charging finished
        initial_pose.position.x = turtle_2d_x;
        initial_pose.position.y = turtle_2d_y;
        initial_pose.orientation.w = 0;
        initial_pose.orientation.x = 0;
        initial_pose.orientation.y = 0;
        initial_pose.orientation.z = 1;

        set_docking_state("searching");
 
        call_send_goal = true; // make available for search goal or origin
        

        ready_2d_pose = false;
        ready_battery_data = false;
    }

}

void DockingController::searching_state_func_2d()
{
    // Get turtle pose
    if (!ready_2d_pose)
    {
        // RCLCPP_INFO(get_logger(), "Turtle Pose not ready");
        return;
    }
    else if (!ready_battery_data)
    {
        return;
    }
    // if (ready_battery_data_data && (current_percent < 27.00))
    // {
    //     RCLCPP_INFO_ONCE(get_logger(), "### BATTERY FAIL: %f ###", current_percent);
    //     return;
    // }


    // send_goal(search_goal_pose, "cancel");
    turtlebot_stop();

    
    // If no queue number recieved then can't start next state
    if (!ready_queue_state)
    {
        return;
    }

    // Assign docking state depending on queue state
    if (queue_state == "Docking")
    {
        is_docking = true;
        set_docking_state("initial_approach");  // set next state
        
    }
    else if (queue_state == "Queuing")
    {
        set_docking_state("queue_approach");    // set next state    
                 
    }

    call_send_goal = true;
    
    ready_battery_data = false;
    ready_queue_state = false;
}

void DockingController::initial_approach_state_func_2d()
{
    // Get turtle pose
    if (!ready_2d_pose)
    {
        // RCLCPP_INFO(get_logger(), "Turtle Pose not ready");
        return;
    }
    // else if (!ready_battery_data_data )
    // {
    //     return;
    // }
    // if (ready_battery_data && (current_percent < 27.00))
    // {
    //     RCLCPP_INFO_ONCE(get_logger(), "### BATTERY FAIL: %f ###", current_percent);
    //     return;
    // }
    

    // Go to initial approach goal
    if (call_send_goal)
    {
        calculate_goal_2d(queue_num);
        send_goal(initial_approach_goal_pose, "send");
        call_send_goal = false;

        // next_tag_pose = false;
    }

    // For continuous tag if needed
    // if (next_tag_pose)
    // {
    //     calculate_goal(queue_num);
    //     send_goal(initial_approach_goal_pose, "send");
    //     next_tag_pose = false;
    // }
    
    // If robot has reached initial approach goal
    float distance_to_iap = distance_2d(initial_approach_goal_pose);
    // RCLCPP_INFO(get_logger(), "Distance to IAPG %f ", distance_to_iap);

    if (distance_to_iap < 0.25)
    // if (goal_reached)
    {
        turtlebot_stop();
        set_docking_state("final_approach");

        // goal_reached = false;
        // final_approach_time_start = steady_clock::now();    // start clock for final approach to be able to simulate time needed
    }
    
    // Get current time passed
    // steady_clock::time_point time_passed = steady_clock::now();
    // approach_time_sim_passed = duration_cast<duration<float>>(time_passed - approach_time_start).count();
    // RCLCPP_INFO_ONCE(get_logger(), "Approach Real Time Passed: %f", (approach_time_real_passed/sim_time_dilation));
    // RCLCPP_INFO_ONCE(get_logger(), "Approach Percent Used: %f", (approach_time_real_passed/31.2237*0.032022));
        
    ready_2d_pose = false;
    ready_battery_data = false;
}

void DockingController::final_approach_state_func_2d()
{
    // Get turtle pose
    if (!ready_2d_pose)
    {
        // RCLCPP_INFO(get_logger(), "Turtle Pose not ready");
        return;
    }
    // else if (!ready_battery_data )
    // {
    //     return;
    // }
    // if (ready_battery_data && (current_percent < 27.00))
    // {
    //     RCLCPP_INFO_ONCE(get_logger(), "### BATTERY FAIL: %f ###", current_percent);
    //     return;
    // }
    

    if (abs(steering_angle_2d(final_approach_goal_pose) - turtle_2d_theta) >= angle_tolerance)
    {      
        vel_msg.linear.x = 0;
        vel_msg.angular.z = 1.0 * (steering_angle_2d(final_approach_goal_pose)-turtle_2d_theta);
        vel_publisher->publish(vel_msg);
    }
    else
    {   
        float dist = distance_2d(final_approach_goal_pose);
        if (dist >= 0.05)
        {
            vel_msg.linear.x = 0.1;
            vel_msg.angular.z = 0;
            vel_publisher->publish(vel_msg);
        }
        else
        {
            turtlebot_stop();
            set_docking_state("docked");
            docked_time_start = steady_clock::now();
        }
    }

    // // Get current time passed
    // steady_clock::time_point time_passed = steady_clock::now();
    // final_approach_time_sim_passed = duration_cast<duration<float>>(time_passed - final_approach_time_start).count();

    // // 31.2237 = time dilation
    // if (final_approach_time_sim_passed >= time_real_to_final_approach_goal/sim_time_dilation)
    // {
    //     set_docking_state("docked");
    //     docked_time_start = steady_clock::now();
    // }

 
    ready_2d_pose = false;
    ready_battery_data = false;
}

void DockingController::docked_state_func_2d()
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
    
    // wait 60 second to account for charging time, 31.2237 = time dilation
    if (docked_time_passed >= time_to_charge/sim_time_dilation)
    {
        // send state change to pop robot off queue
        threads.push_back(std::thread(std::bind(&DockingController::queue_update_client, this, "state_change")));

        set_docking_state("wait");

        exit_dock(); // if charge is complete, go to original position
    }
}

void DockingController::queue_approach_state_func_2d()
{
    // Get turtle pose
    if (!ready_2d_pose)
    {
        // RCLCPP_INFO(get_logger(), "Turtle Pose not ready");
        return;
    }
    // else if (!ready_battery_data )
    // {
    //     return;
    // }
    // if (ready_battery_data && (current_percent < 27.00))
    // {
    //     RCLCPP_INFO_ONCE(get_logger(), "### BATTERY FAIL: %f ###", current_percent);
    //     return;
    // }
    
    // Go to queue approach goal
    if (call_send_goal)
    {
        calculate_goal_2d(queue_num);
        send_goal(queue_goal_pose, "send");
        call_send_goal = false;
    }

    // If robot has reached initial approach goal
    float distance_to_qip = distance_2d(queue_goal_pose);
    // RCLCPP_INFO_ONCE(get_logger(), "Distance to QiP: %f ", distance_to_qip);

    if (distance_to_qip < 0.25)
    {
        turtlebot_stop();
        set_docking_state("in_queue");

        is_in_queue = true;
    }

    // Check if docking
    if (queue_state == "Docking")
    {
        set_docking_state("initial_approach");  // set next state
        // calculate_goal(queue_num);      // get approach goal
        call_send_goal = true;
    }
    // else if (call_send_goal)
    // {
    //     calculate_goal(queue_num);
    //     send_goal(queue_goal_pose, "send");
    //     call_send_goal = false;
    // }
    else if (new_queue_num_rcv)
    {
        new_queue_num_rcv = false;      // get approach goal
        call_send_goal = true;
    }

    ready_battery_data = false;
    ready_2d_pose = false;
}

void DockingController::in_queue_state_func_2d()
{
    // Get turtle pose
    if (!ready_2d_pose)
    {
        // RCLCPP_INFO(get_logger(), "Turtle Pose not ready");
        return;
    }
    // else if (!ready_battery_data )
    // {
    //     return;
    // }
    // if (ready_battery_data && (current_percent < 27.00))
    // {
    //     RCLCPP_INFO_ONCE(get_logger(), "### BATTERY FAIL: %f ###", current_percent);
    //     return;
    // }
    
   
    // Once arrived in queue, change state
    if (is_in_queue)
    {
        threads.push_back(std::thread(std::bind(&DockingController::queue_update_client, this, "state_change")));
        is_in_queue = false;
    }


    // Check if docking
    if (queue_state == "Docking")
    {
        set_docking_state("initial_approach");  // set next state
        // calculate_goal(queue_num);      // get approach goal
        call_send_goal = true;
    }
    else if (call_send_goal)
    {
        calculate_goal_2d(queue_num);
        send_goal(queue_goal_pose, "send");
        call_send_goal = false;
    }
   
    // If not docking check if change to new queu position
    else if (new_queue_num_rcv)
    {
        new_queue_num_rcv = false;      // get approach goal
        // calculate_goal(queue_num); 
        call_send_goal = true;
    }

    
   ready_battery_data = false;
   ready_2d_pose = false;
}

void DockingController::calculate_goal_2d(int queue_num)
{    
    // If docking
    if (queue_num == 0)
    {
        // initial_approach_goal_pose.position.x =  abs(turtle_2d_x) + 2.0;
        // initial_approach_goal_pose.position.y =  abs(turtle_2d_y) + 0.0;

        initial_approach_goal_pose.position.x = 2.0;
        initial_approach_goal_pose.position.y = 0.0;
        initial_approach_goal_pose.orientation.x = 0.0;
        initial_approach_goal_pose.orientation.y = 0.0;
        initial_approach_goal_pose.orientation.z = 0.0;
        initial_approach_goal_pose.orientation.w = 1.0;

        RCLCPP_INFO(get_logger(), "initial approach_goal x: %f", initial_approach_goal_pose.position.x);
        RCLCPP_INFO(get_logger(), "initial approach_goal y: %f", initial_approach_goal_pose.position.y);

        // final_approach_goal_pose.position.x =  abs(turtle_2d_x) + 2.5;
        // final_approach_goal_pose.position.y =  abs(turtle_2d_y) + 0.0;

        final_approach_goal_pose.position.x = 2.5;
        final_approach_goal_pose.position.y = 0.0;
        final_approach_goal_pose.orientation.x = 0.0;
        final_approach_goal_pose.orientation.y = 0.0;
        final_approach_goal_pose.orientation.z = 0.0;
        final_approach_goal_pose.orientation.w = 1.0;

        RCLCPP_INFO(get_logger(), "final_approach_goal x: %f", final_approach_goal_pose.position.x);
        RCLCPP_INFO(get_logger(), "final_approach_goal y: %f", final_approach_goal_pose.position.y);

    }
    else // queueing or in queue
    {
        // queue_goal_pose.position.x =  abs(turtle_2d_y) + 2.0; 
        // queue_goal_pose.position.y =  abs(turtle_2d_y) + (-2.0 * queue_num);

        queue_goal_pose.position.x =  2.0; 
        queue_goal_pose.position.y =  (-2.0 * queue_num);

        RCLCPP_INFO(get_logger(), "queue_goal x: %f", queue_goal_pose.position.x);
        RCLCPP_INFO(get_logger(), "queue_goal y: %f", queue_goal_pose.position.y);

        // Calculate time needed to go to approach goal
        queue_approach_distance = distance_2d(queue_goal_pose);       
    }

    // new_queue_num_rcv = true;
    // last_queue_num = queue_num;
}

double DockingController::distance_2d(geometry_msgs::msg::Pose goal_pose)
{
    return sqrt(pow((goal_pose.position.x - turtle_2d_x), 2) +
                pow((goal_pose.position.y - turtle_2d_y), 2));
}

double DockingController::steering_angle_2d(geometry_msgs::msg::Pose goal_pose)
{
    return atan2(goal_pose.position.y - turtle_2d_y, goal_pose.position.x - turtle_2d_x);
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
    // If docking
    if (queue_num == 0)
    {
        // initial_approach_goal_pose.position.x = 2.0;
        // initial_approach_goal_pose.position.y = 0.0;

        // initial_approach_goal_pose.position.x = sqrt(tag_x*tag_x - turtle_y*turtle_y) + turtle_x; // for sub

        initial_approach_goal_pose.position.x = tag_x - 1.0; // for pose
        initial_approach_goal_pose.position.y = 0;

        initial_approach_goal_pose.orientation.x = 0.0;
        initial_approach_goal_pose.orientation.y = 0.0;
        initial_approach_goal_pose.orientation.z = 0.0;
        initial_approach_goal_pose.orientation.w = 1.0;

        RCLCPP_INFO(get_logger(), "initial approach_goal x: %f", initial_approach_goal_pose.position.x);
        // RCLCPP_INFO(get_logger(), "initial approach_goal y: %f", initial_approach_goal_pose.position.y);

        // final_approach_goal_pose.position.x = 2.5;
        // final_approach_goal_pose.position.y = 0.0;

        final_approach_goal_pose.position.x = tag_x - 0.5; // for pose
        final_approach_goal_pose.position.y = 0.0;

        final_approach_goal_pose.orientation.x = 0.0;
        final_approach_goal_pose.orientation.y = 0.0;
        final_approach_goal_pose.orientation.z = 0.0;
        final_approach_goal_pose.orientation.w = 1.0;

        // RCLCPP_INFO(get_logger(), "final_approach_goal x: %f", final_approach_goal_pose.position.x);
        // RCLCPP_INFO(get_logger(), "final_approach_goal y: %f", final_approach_goal_pose.position.y);

        // Calculate time needed to go to approach goal

        // TO DO: get rid of if and only keep else
        // On physical robot the turtle pose will be updated but not on sim without movement
        // float approach_distance;
        // if (last_queue_state == "")
        // {
        //     approach_distance = distance(approach_goal_pose);
        // }
        // else
        // {
        //     approach_distance = sim_distance(approach_goal_pose);
        // }
        //approach_distance = distance(approach_goal_pose);

        // 0.1 m/s assumed vel , 0.032022 is [%/s] with time , 0.32022 is [%/m] when vel=0.1m/s
        // time_real_to_approach_goal = approach_distance/velocity;
        // percent_to_approach_goal = approach_distance*0.32022;

        // RCLCPP_INFO(get_logger(), "Approach distance: %f", approach_distance);
        // RCLCPP_INFO(get_logger(), "Time to approach goal: %f", time_real_to_approach_goal);
        // RCLCPP_INFO_ONCE(get_logger(), "Percent to approach goal: %f", percent_to_approach_goal);

        // start approach clock
        // approach_time_start = std::chrono::steady_clock::now();
    }
    else // queueing or in queue
    {
        queue_goal_pose.position.x = tag_x - 1.0; 
        queue_goal_pose.position.y = -2.0 * queue_num;

        // queue_goal_pose.orientation.x = 0.0;
        // queue_goal_pose.orientation.y = 0.0;
        // queue_goal_pose.orientation.z = 0.7073;
        // queue_goal_pose.orientation.w = 0.7068;

        // Calculate time needed to go to approach goal
        queue_approach_distance = distance(queue_goal_pose);

        
        // 0.1 m/s assumed vel , 0.032022 is [%/s] with time , 0.32022 is [%/m] when vel=0.1m/s
        // time_real_to_queue_approach_goal = queue_approach_distance/velocity;
        // percent_to_queue_approach_goal = queue_approach_distance*0.32022;

        // RCLCPP_INFO_ONCE(get_logger(), "Queue Approach distance: %f", queue_approach_distance);
        // RCLCPP_INFO_ONCE(get_logger(), "Queue Time to queue approach goal: %f", time_to_queue_approach_goal);
        // RCLCPP_INFO_ONCE(get_logger(), "Queue Percent to queue approach goal: %f", percent_to_queue_approach_goal);

        // start approach clock
        // queue_approach_time_start = std::chrono::steady_clock::now();
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
        RCLCPP_INFO(get_logger(), "### %s Docking state: %s, Last state: %s ###", robot_id.c_str(), docking_state.c_str(), last_docking_state.c_str());
    }
}

void DockingController::set_queue_info(int new_queue_num, std::string new_queue_state)
{
    if (queue_num != new_queue_num)
    {
        last_queue_num = queue_num;
        queue_num = new_queue_num;
        new_queue_num_rcv = true;
        RCLCPP_INFO(get_logger(), "### %s Queue num: %d, Last num: %d ###", robot_id.c_str(), queue_num, last_queue_num);
    }

    if (queue_state != new_queue_state)
    {
        last_queue_state = queue_state;
        queue_state = new_queue_state;
        RCLCPP_INFO(get_logger(), "### %s Queue state: %s, Last state: %s ###", robot_id.c_str(), queue_state.c_str(), last_queue_state.c_str());
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
    // docking_state_manager_2d(); // Manage docking states
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