#include <rclcpp/rclcpp.hpp>
#include <cmath>

#include <docking_interfaces/msg/current_state.hpp>
#include "docking_interfaces/msg/charging_queue.hpp"
#include <docking_interfaces/srv/docking.hpp>
#include <docking_interfaces/srv/start_april_tag_detection.hpp>

#include <sensor_msgs/msg/battery_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>


using std::placeholders::_1;
using namespace std::chrono_literals;

class DockingClient : public rclcpp::Node
{
    public:
        DockingClient() : Node("docking_client")
        {
            this->declare_parameter<std::string>("robot_id", "tb3_1");
            this->get_parameter("robot_id",robot_id);


            // For testing
            this->declare_parameter<float>("init_x_pose", 0.0);
            this->get_parameter("init_x_pose", init_x_pose);

            this->declare_parameter<float>("init_y_pose", 0.0);
            this->get_parameter("init_y_pose", init_y_pose);

            this->declare_parameter<bool>("sim_2d", false);
            this->get_parameter("sim_2d",sim_2d);


            /*** SUBSCRIBERS DEFINITIONS***/
            battery_subscriber = this->create_subscription<sensor_msgs::msg::BatteryState>(
                "battery_state", 10, std::bind(&DockingClient::callbackBattery, this, _1)
            );
            odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
                "odom", 10, std::bind(&DockingClient::callbackOdom, this, std::placeholders::_1)
            );
            vel_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
                "cmd_vel", 10, std::bind(&DockingClient::callbackVel, this, std::placeholders::_1)
            );
            queue_subscriber = this->create_subscription<docking_interfaces::msg::ChargingQueue>(
                "/charging_queue_size", 10, std::bind(&DockingClient::callbackQueue, this, _1)
            );

            // Every 50ms check to see if docking is required
            timer_ = this->create_wall_timer(
                50ms, std::bind(&DockingClient::isDockingRequired, this)
            );

            RCLCPP_INFO(this->get_logger(), "Docking Client has been started.");
        }

        void callAprilTagDetectionService()
        {
            auto client = this->create_client<docking_interfaces::srv::StartAprilTagDetection>("detect_tag_pupil/start_apriltag_detection");
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

        
    private:
        // PARAMS
        std::string robot_id;

        /*** VARIABLES ***/
        std::thread thread_dock;
        std::thread thread_tag;

        float current_percent;  // battery data
        float current_voltage;  

        float x_dist;           // odom data
        float y_dist;
        float current_distance; 

        float x_vel = 0; //TEST Value         
        float y_vel;

        bool stop_client = false;       // docking requirement function bools
        bool docking_required = false;

        float max_distance = 216;   // max distant the robot can physically travel until shutdown
        float max_percentage = 100; // 100%
        float min_percentage = 27;  // 25%
        
        float percent_buff = 7.5;    // 10%
        float distance_buff = max_distance/10;

        bool odom_received = false;
        bool vel_received = true; // CHANGE: to false after testing
        bool battery_received = false;

        int queue_size = 0;

        int t_charge = 60;

        int charging_condition;

        int count = 0; // TESTING
        bool sim_2d = false;
        float init_x_pose;
        float init_y_pose;

        // Constants for % and V battery equations, y = mx+b
        const float percent_slope = -0.2648;
        const float percent_intercept = 0.4792;     
        const float percent_per_second = 0.0213;

        /*** INTERFACES ***/
        sensor_msgs::msg::BatteryState  last_battery_msg;
        nav_msgs::msg::Odometry         last_odom_msg;
        geometry_msgs::msg::Twist       last_vel_msg;

        rclcpp::TimerBase::SharedPtr timer_;

        /*** SUBSCRIPTION DECLARATIONS***/
        rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_subscriber;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr        odom_subscriber;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr      vel_subscriber;
        rclcpp::Subscription<docking_interfaces::msg::ChargingQueue>::SharedPtr queue_subscriber;
        

        void callbackBattery(const sensor_msgs::msg::BatteryState::SharedPtr msg)
        {            
            current_percent = msg->percentage;
            current_voltage = msg->voltage;

            battery_received = true;
        }

        void callbackOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            x_dist = msg->pose.pose.position.x;
            y_dist = msg->pose.pose.position.y;

            // FOR TESTING, ODOM IS NOT WORKING
            // y_dist = 0;
            // current_distance = sqrt((x_dist*x_dist) + (y_dist*y_dist));

            if (sim_2d)
            {
                current_distance = sqrt((init_x_pose*init_x_pose) + (init_y_pose*init_y_pose));
            }
            else
            {
                current_distance = sqrt((x_dist*x_dist) + (y_dist*y_dist));
            }
            

            odom_received = true;
        }

        void callbackVel(const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            x_vel = msg->linear.x;
            y_vel = msg->linear.y;

            vel_received = true;
        }

        void callbackQueue(const docking_interfaces::msg::ChargingQueue::SharedPtr msg)
        {
            queue_size = msg->size;
        }


        /*** FUNCTIONS ***/
        void isDockingRequired()
        {
            // Wait for all message to be received
            if (!(battery_received && odom_received /*&& vel_received*/))
            {
                return;
            }

            float percent_per_meter = percent_slope*0.1+ percent_intercept;

            float queue_buff = queue_size*t_charge*percent_per_second; // Assuming 60s charge time and constant %/s dissipation
            // std::cout << queue_buff;

            float percent_req = current_percent - percent_buff - queue_buff - min_percentage;
            float percent_to_dock = current_distance * percent_per_meter;

            // Condition 1: Current Battery % - threshold > % left needed to dock
            if (percent_req < percent_to_dock)
            {
                docking_required = true;
                charging_condition = 1;
            }
            else if((current_distance - distance_buff) > (max_distance/2))
            {
                docking_required = true;
                charging_condition = 2;
            }

            // If docking requirement is met
            //      1. Start AprilTag Detection (service call)
            //      2. Start Docking process (service call)
            if (docking_required)
            {
                if (!stop_client)
                {
                    // RCLCPP_INFO(this->get_logger(), "Charging Required: Condition %s", charging_condition);
                    // RCLCPP_INFO(this->get_logger(), "Percent w/ Buff: %f, Percent to dock: %f.", percent_needed_w_buff, percent_to_dock);

                    thread_tag = std::thread(std::bind(&DockingClient::callAprilTagDetectionService, this));
                    thread_dock = std::thread(std::bind(&DockingClient::callDockingService, this));
                    
                    thread_tag.detach();
                    thread_dock.detach();

                    stop_client = true;
                }
            }
            else
            {
                battery_received = false;
                odom_received = false;
                // vel_received = false;
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