#include <rclcpp/rclcpp.hpp>
#include <cmath>

#include <docking_interfaces/msg/current_state.hpp>
#include <docking_interfaces/srv/docking.hpp>
#include <docking_interfaces/srv/start_april_tag_detection.hpp>

#include <sensor_msgs/msg/battery_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>


using std::placeholders::_1;


class DockingClient : public rclcpp::Node
{
    public:
        DockingClient() : Node("docking_client")
        {
            /*** SUBSCRIBERS DEFINITIONS***/
            battery_subscriber = this->create_subscription<sensor_msgs::msg::BatteryState>(
                "/battery_state", 10, std::bind(&DockingClient::callbackBattery, this, _1)
            );
            odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
                "odom", 10, std::bind(&DockingClient::callbackOdom, this, std::placeholders::_1)
            );
            vel_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
                "cmd_vel", 10, std::bind(&DockingClient::callbackVel, this, std::placeholders::_1)
            );

            RCLCPP_INFO(this->get_logger(), "Docking Client has been started.");
        }

        void callAprilTagDetectionService()
        {
            auto client = this->create_client<docking_interfaces::srv::StartAprilTagDetection>("detect_apriltag_pupil/start_apriltag_detection");
            while (!client->wait_for_service(std::chrono::seconds(1)))
            {
                RCLCPP_WARN(this->get_logger(), "Waiting for the apriltag detection server to be up...");
            }

            std::string tag_request =  "start";

            auto request = std::make_shared<docking_interfaces::srv::StartAprilTagDetection::Request>();
            request->service = tag_request;

            auto future = client->async_send_request(request);

            try
            {
                auto response = future.get();
                RCLCPP_INFO(this->get_logger(), "AprilTag detection service request (docking:=%s) successful.", tag_request.c_str());
            }
            catch (const std::exception &e)
            {
                RCLCPP_INFO(this->get_logger(), "AprilTag detection service request (docking:=%s) failed.", tag_request.c_str());
            }
        }

        void callDockingService()
        {
            auto client = this->create_client<docking_interfaces::srv::Docking>("docking_controller/docking_service");
            while (!client->wait_for_service(std::chrono::seconds(1)))
            {
                RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
            }

            //std::string docking_request = this->declare_parameter("docking_request", "start");
            std::string docking_request =  "start";

            auto request = std::make_shared<docking_interfaces::srv::Docking::Request>();
            request->service = docking_request;

            auto future = client->async_send_request(request);

            try
            {
                auto response = future.get();
                RCLCPP_INFO(this->get_logger(), "Docking service request (docking:=%s) successful.", docking_request.c_str());
            }
            catch (const std::exception &e)
            {
                RCLCPP_INFO(this->get_logger(), "Docking service request (docking:=%s) failed.", docking_request.c_str());
            }
        }

       bool stop_client = false;
       bool docking_required = false;

       float max_distance_percentage = 216.2;   // 216.2m when comparing with %
       float max_distance_voltage = 240.3;      // 240.3m when comparing with V

       float max_percentage = 1.00; // 100%
       float max_voltage = 12.5;    // 12.5V

       float min_percentage = 0.25; // 25%
       float min_voltage = 11.0;    // 11.0V

       float percent_threshold = 10;      // 10%
       float voltage_threshold = 0.1*1.5;   // 10% of 1.5V

       float percent_per_meter = 0.3467
       float volatge_per_meter = 0.0062
       

    private:
        /*** VARIABLES ***/
        std::thread thread_dock;
        std::thread thread_tag;

        float x_dist;
        float y_dist;
        float current_distance; // from dock

        float current_percent;  // of battery
        float current_voltage;  // of battery

        float x_vel;
        float y_vel;

        /*** INTERFACES ***/
        sensor_msgs::msg::BatteryState  last_battery_msg;
        nav_msgs::msg::Odometry         last_odom_msg;
        geometry_msgs::msg::Twist       last_vel_msg;

        /*** SUBSCRIPTION DECLARATIONS***/
        rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_subscriber;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr        odom_subscriber;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr      vel_subscriber;

        void callbackBattery(const sensor_msgs::msg::BatteryState::SharedPtr msg)
        {            
            current_percent = msg->percent;
            current_voltage = msg->voltage;
        }

        void callbackOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            x_dist = msg->pose.pose.position.x;
            y_dist = msg->pose.pose.position.y;

            current_distance = sqrt((x_dist*x_dist) + (y_dist*y_dist));
        }

        void callbackVel(const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            x_vel = msg->linear.x;
            y_vel = msg->linear.y;
        }


        /*** FUNCTIONS ***/
        void isDockingRequired(float distance, float percentage, float voltage)
        {
            // Condition 1: Current Battery % - threshold > % left needed to dock
            if ((current_percent- percent_threshold) > (current_distance * percent_per_meter))
            {
                docking_required = true;
            }
            // Condition 2: Current Battery V - threshold > V left needed to dock
            else if ((current_voltage - voltage_threshold) > (current_distance * voltage_per_meter))
            {
                docking_required = true;
            }


            // If docking requirement is met
            // 1. Start AprilTag Detection (service call)
            // 2. Start Docking process (service call)
            if (docking_required)
            {
                if (!stop_client)
                {
                    RCLCPP_INFO(this->get_logger(), "Charging Required.");

                    thread_tag = std::thread(std::bind(&DockingClient::callAprilTagDetectionService, this));
                    thread_dock = std::thread(std::bind(&DockingClient::callDockingService, this));
                    
                    thread_tag.detach();
                    thread_dock.detach();

                    stop_client = true;
                }
            }

        }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DockingClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}