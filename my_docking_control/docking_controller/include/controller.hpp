#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"

#include "sensor_msgs/msg/battery_state.hpp"

#include "docking_interfaces/msg/current_state.hpp"
#include "docking_interfaces/srv/docking.hpp"
#include "docking_interfaces/srv/gazebo_charge_battery.hpp"
#include "docking_interfaces/srv/queue_update.hpp"
#include "docking_interfaces/srv/state_update.hpp"

#include "nav_msgs/msg/odometry.hpp"

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

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

using namespace std::chrono_literals;

class DockingController : public rclcpp::Node
{
public:
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

        this->declare_parameter<std::string>("robot_id", "0");
        this->get_parameter("robot_id",robot_id);

        /*** Define Publishers & Services ***/
        vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        state_publisher = this->create_publisher<docking_interfaces::msg::CurrentState>("docking_controller/current_state", 10);
        
        docking_service = this->create_service<docking_interfaces::srv::Docking>(
            "docking_controller/docking_service", std::bind(&DockingController::docking_server, this, _1, _2));
        state_update_service = this->create_service<docking_interfaces::srv::StateUpdate>(
            "docking_controller/state_update_service", std::bind(&DockingController::state_update_server, this, _1, _2));

        /*** Define Subscribers ***/
        // tag_pose_subscriber = this->create_subscription<geometry_msgs::msg::Pose>(
        //     "detections", 10, std::bind(&DockingController::callbackTagPose, this, _1)
        // );
        odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&DockingController::callbackOdom, this, _1));
        battery_subscriber = this->create_subscription<sensor_msgs::msg::BatteryState>(
            "battery_state", 10, std::bind(&DockingController::callbackBattery, this, _1));
        

        tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        tf_timer = this->create_wall_timer(
            1s, std::bind(&DockingController::on_tf_timer, this));

        RCLCPP_INFO(this->get_logger(), "Docking Controller has been started.");
        RCLCPP_INFO(this->get_logger(), "Robot ID: %s", robot_id.c_str());

    }

    void set_docking_state(std::string new_docking_state);
    void run(); // Run for main program

private:
    rclcpp::TimerBase::SharedPtr tf_timer;

    std::thread thread_queue;
    std::vector<std::thread> threads;

    /*** Variables ***/
    std::string docking_state = "";
    std::string last_docking_state = "";

    std::string queue_state;
    int queue_num;

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

    // Calculations
    double distance(geometry_msgs::msg::Pose goal_pose);
    double linear_velocity(geometry_msgs::msg::Pose goal_pose);
    double angular_velocity(geometry_msgs::msg::Pose goal_pose);
    double steering_angle(geometry_msgs::msg::Pose goal_pose);

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


        turtle_y = 0;  // FOR TESTING, ODOM IS NOT WORKING
        turtle_distance = sqrt((turtle_x*turtle_x) + (turtle_y*turtle_y));

        ready_turtle_pose = true;
    }

    void callbackBattery(const sensor_msgs::msg::BatteryState::SharedPtr msg)
    {            
        current_percent = msg->percentage;

        battery_received = true;
    }

    void on_tf_timer()
    {
        if (start_tag_detection)
        {
            geometry_msgs::msg::TransformStamped transformStamped;

            std::string fromFrameRel = "tag_36h11_00408";
            std::string toFrameRel = "odom"; // was odom
            try
            {
                transformStamped = tf_buffer->lookupTransform(
                    toFrameRel, fromFrameRel,
                    tf2::TimePointZero);

                    tag_counter += 1;
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_INFO(
                    this->get_logger(), "Could not transform %s to %s: %s",
                    toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
                return;
            }

            tag_x = transformStamped.transform.translation.x;
            tag_y = transformStamped.transform.translation.y;

            // RCLCPP_INFO_ONCE(get_logger(),"tag_x timer position: %f", tag_x);
            // RCLCPP_INFO_ONCE(get_logger(),"tag_y timer position: %f", tag_y);
            
            ready_tag_pose = true;
        }
        
    }
};