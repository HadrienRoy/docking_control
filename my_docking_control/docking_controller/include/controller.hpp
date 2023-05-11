#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"

#include "sensor_msgs/msg/battery_state.hpp"

#include "docking_interfaces/msg/current_state.hpp"
#include "docking_interfaces/msg/charging_queue.hpp"
#include "docking_interfaces/srv/docking.hpp"
#include "docking_interfaces/srv/gazebo_charge_battery.hpp"
#include "docking_interfaces/srv/queue_update.hpp"
#include "docking_interfaces/srv/state_update.hpp"
#include "docking_interfaces/srv/rank_update.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Transform.h"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include "rcl_interfaces/msg/parameter_event.hpp"

#include "cmath"
#include "unistd.h" // for sleep
#include "math.h"
#include "chrono"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

using namespace std::chrono_literals;
using namespace std::chrono;


class DockingController : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    
    DockingController() : Node("docking_controller")
    {
        
        /*** Declare Parameters ***/
        this->declare_parameter<double>("vel_linear", 0.1);
        this->get_parameter("vel_linear", vel_linear);

        this->declare_parameter<double>("vel_angular", 0.2);
        this->get_parameter("vel_angular", vel_angular);

        this->declare_parameter<double>("approach_distance_tolerance", 0.1);
        this->get_parameter("approach_distance_tolerance", approach_distance_tolerance);

        this->declare_parameter<double>("final_approach_distance_tolerance", 0.1);
        this->get_parameter("final_approach_distance_tolerance", final_approach_distance_tolerance);

        this->declare_parameter<double>("angle_tolerance", 0.02);
        this->get_parameter("angle_tolerance", angle_tolerance);

        this->declare_parameter<std::string>("robot_id", "tb3_1");
        this->get_parameter("robot_id",robot_id);


        /*** Define Publishers & Services ***/
        vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        state_publisher = this->create_publisher<docking_interfaces::msg::CurrentState>("docking_controller/current_state", 10);
        
        docking_service = this->create_service<docking_interfaces::srv::Docking>(
            "docking_controller/docking_service", std::bind(&DockingController::docking_server, this, _1, _2));
        state_update_service = this->create_service<docking_interfaces::srv::StateUpdate>(
            "docking_controller/state_update_service", std::bind(&DockingController::state_update_server, this, _1, _2));
        rank_update_service = this->create_service<docking_interfaces::srv::RankUpdate>(
            "docking_controller/rank_update_service", std::bind(&DockingController::rank_update_server, this, _1, _2));
        
        /*** Define Subscribers ***/
        // tag_pose_subscriber = this->create_subscription<geometry_msgs::msg::Pose>(
        //     "detections", 10, std::bind(&DockingController::callbackTagPose, this, _1)
        // );
        odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&DockingController::callbackOdom, this, _1));
        battery_subscriber = this->create_subscription<sensor_msgs::msg::BatteryState>(
            "battery_state", 10, std::bind(&DockingController::callbackBattery, this, _1));

        // Navigation
        nav_client = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        

        tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        // tf_timer = this->create_wall_timer(
        //     1s, std::bind(&DockingController::on_tf_timer, this));

       


        RCLCPP_INFO(this->get_logger(), "Docking Controller has been started.");
        RCLCPP_INFO(this->get_logger(), "Robot ID: %s", robot_id.c_str());

    }

    void set_docking_state(std::string new_docking_state);
    void run(); // Run for main program

private:
    rclcpp::TimerBase::SharedPtr tf_timer;

    // float sim_time_dilation = 31.2237;
    float sim_time_dilation = 46.9484;
    float velocity = 0.1;

    float percent_per_second = 0.3202;

    steady_clock::time_point approach_time_start;
    float approach_time_sim_passed;
    float time_real_to_approach_goal;
    float percent_to_approach_goal;

    steady_clock::time_point final_approach_time_start;
    float final_approach_time_sim_passed;
    float time_real_to_final_approach_goal = 5;
    

    steady_clock::time_point docked_time_start;
    float docked_time_passed;
    float time_to_charge = 60;
   
    
    steady_clock::time_point queue_approach_time_start;
    float queue_approach_time_sim_passed; // simulated time of 1% drop per minute
    float time_real_to_queue_approach_goal;
    float percent_to_queue_approach_goal;
    float queue_approach_distance;




    

    std::thread thread_queue;
    std::vector<std::thread> threads;

    /*** Variables ***/
    std::string docking_state = "";
    std::string last_docking_state = "";

    std::string queue_state = ""; // Queue state variables
    std::string last_queue_state = "";
    int queue_num = -1;
    int last_queue_num = -1;

    // Pose information
    double tag_x;
    double tag_y;

    float turtle_x;
    float turtle_y;
    float turtle_theta;
    float turtle_distance;

    // Booleans
    bool ready_tag_pose = false;
    bool ready_turtle_pose = false;
    bool start_tag_detection = false;
    bool battery_received = false;
    bool is_docking = false;
    bool queue_state_received = false;
    bool is_in_queue = false;
    bool first_approach = true;
    bool new_queue_num_rcv = false;


    // Used for calculating turning angle
    double approach_angle;
    double final_approach_angle;

    // Param variables
    double vel_linear;
    double vel_angular;
    double approach_distance_tolerance;
    double final_approach_distance_tolerance;
    double angle_tolerance;
    std::string robot_id;

    // Battery
    float current_percent;

    // PID
    double kp = 0.5;

    int tag_counter = 0;


    /*** Declare Publishers & Services ***/
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher;
    rclcpp::Publisher<docking_interfaces::msg::CurrentState>::SharedPtr state_publisher;
    rclcpp::Service<docking_interfaces::srv::Docking>::SharedPtr docking_service;
    rclcpp::Service<docking_interfaces::srv::StateUpdate>::SharedPtr state_update_service;
    rclcpp::Service<docking_interfaces::srv::RankUpdate>::SharedPtr rank_update_service;

    /*** Declare Subscribers & Service Clients ***/
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr tag_pose_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_subscriber;
    
    /*** TF2 ***/
    std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;

    /*** Declare Messages ***/
    geometry_msgs::msg::Twist vel_msg;
    geometry_msgs::msg::Pose last_pose;
    geometry_msgs::msg::Pose approach_goal_pose;
    geometry_msgs::msg::Pose final_approach_goal_pose;

    geometry_msgs::msg::Pose queue_goal_pose; // For queueing
    float current_distance; // for queueing to docking distance

    geometry_msgs::msg::Pose tag_pose;
    geometry_msgs::msg::Pose turtle_pose;

    /*** Declare Member Functions ***/
    // Client for gazebo battery charger
    void gazebo_charge_battery_client();

    // Turtlebot3 Movement Functions
    void turtlebot_stop();
    void turtlebot_move(double velocity, double radians);
    void turtlebot_turn(double radians);
    void turtlebot_turn_velocity(double angular_velocity);
    void turtlebot_forward(double velocity);

    // State Functions
    void start_state_func();
    void searching_state_func();
    void approach_state_func();
    void final_approach_state_func();
    void docked_state_func();
    void queue_approach_state_func();
    void in_queue_state_func();

    // Calculations
    double distance(geometry_msgs::msg::Pose goal_pose);
    double linear_velocity(geometry_msgs::msg::Pose goal_pose);
    double angular_velocity(geometry_msgs::msg::Pose goal_pose);
    double steering_angle(geometry_msgs::msg::Pose goal_pose);

    void calculate_goal(int queue_num);
    float sim_distance(geometry_msgs::msg::Pose goal_pose);
    void set_queue_info(int new_queue_num, std::string new_queue_state);

    // Controller Functions
    void get_last_tag_pose();
    void docking_state_manager();
    void publish_state();

    void docking_server(
        const std::shared_ptr<docking_interfaces::srv::Docking::Request> request,
        const std::shared_ptr<docking_interfaces::srv::Docking::Response> response);

    void state_update_server(
        const std::shared_ptr<docking_interfaces::srv::StateUpdate::Request> request,
        const std::shared_ptr<docking_interfaces::srv::StateUpdate::Response> response);

    void rank_update_server(
        const std::shared_ptr<docking_interfaces::srv::RankUpdate::Request> request,
        const std::shared_ptr<docking_interfaces::srv::RankUpdate::Response> response);

    void queue_update_client(std::string type);

    /*** Define Callback Functions ***/
    void callbackTagPose(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);

        tf2::Matrix3x3 m(q);

        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // tag_x = msg->position.x;
        // tag_y = msg->position.y;

        ready_tag_pose = true;
    }

    void callbackOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);

        tf2::Matrix3x3 m(q);

        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        turtle_x = msg->pose.pose.position.x;
        turtle_y = msg->pose.pose.position.y;
        turtle_theta = yaw;


        // turtle_y = 0;  // FOR TESTING, ODOM IS NOT WORKING
        turtle_distance = sqrt((turtle_x*turtle_x) + (turtle_y*turtle_y));

        // RCLCPP_INFO_ONCE(get_logger(), "Distance to origin: %0.2f", turtle_distance);

        ready_turtle_pose = true;
    }

    void callbackBattery(const sensor_msgs::msg::BatteryState::SharedPtr msg)
    {            
        current_percent = msg->percentage;

        battery_received = true;
    }

    void on_tf_timer()
    {
        // if (start_tag_detection)
        // {
        //     geometry_msgs::msg::TransformStamped transformStamped;

        //     std::string fromFrameRel = "tag_36h11_00408";
        //     std::string toFrameRel = robot_id+"/odom"; // was odom
        //     try
        //     {
        //         transformStamped = tf_buffer->lookupTransform(
        //             toFrameRel, fromFrameRel,
        //             tf2::TimePointZero);

        //             tag_counter += 1;
        //     }
        //     catch (tf2::TransformException &ex)
        //     {
        //         RCLCPP_INFO(
        //             this->get_logger(), "Could not transform %s to %s: %s",
        //             toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
        //         return;
        //     }

        //     tag_x = transformStamped.transform.translation.x;
        //     tag_y = transformStamped.transform.translation.y;

        //     // RCLCPP_INFO_ONCE(get_logger(),"tag_x timer position: %f", tag_x);
        //     // RCLCPP_INFO_ONCE(get_logger(),"tag_y timer position: %f", tag_y);
            
        //     ready_tag_pose = true;
        // }
        
    }

    /* Navigation2 Goal Functions */
    
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client;

    void sendGoal(geometry_msgs::msg::Pose goal_pose)
    {
        while (!this->nav_client->wait_for_action_server())
        {
            RCLCPP_INFO(get_logger(), "Waiting for action server...");
        }

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

    void navFeedbackCallback(GoalHandleNavigateToPose::SharedPtr,const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        RCLCPP_INFO(get_logger(), "Distance remaining = %f", feedback->distance_remaining);
    }
    
    void navResultCallback(const GoalHandleNavigateToPose::WrappedResult & result)
    {
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Success!!!");
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



};