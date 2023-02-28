#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

#include <iostream>
#include <fstream>

using std::placeholders::_1;
using namespace std::chrono_literals;

class RecordDataNode : public rclcpp::Node
{
    public:
        RecordDataNode() : Node("record_data")
        {
            subscriber_odom = this->create_subscription<nav_msgs::msg::Odometry>(
                "odom", 10, std::bind(&RecordDataNode::callback_odom, this, std::placeholders::_1)
            );
            subscriber_vel = this->create_subscription<geometry_msgs::msg::Twist>(
                "cmd_vel", 10, std::bind(&RecordDataNode::callback_vel, this, std::placeholders::_1)
            );
            subscriber_bat = this->create_subscription<sensor_msgs::msg::BatteryState>(
                "battery_state", 10, std::bind(&RecordDataNode::callback_bat, this, std::placeholders::_1)
            );

            timer_ = this->create_wall_timer(
                30s, std::bind(&RecordDataNode::callback_timer, this)
            );

            RCLCPP_INFO(this->get_logger(), "Data Recording has been started.");
        }
    
    private:
        /** VARIABLES **/
        float data[100][7];

        float x_dist;
        float y_dist;

        float x_vel = 1;
        float y_vel;

        float battery_percentage;
        float battery_voltage;

        int i = 0;
        int j = 0;

        int time = 0;

        /** INTERFACES **/
        nav_msgs::msg::Odometry odom_msg;
        geometry_msgs::msg::Twist vel_msg;
        sensor_msgs::msg::BatteryState battery_msg;

        rclcpp::TimerBase::SharedPtr timer_;

        /*** SUBSCRIPTION DECLARATIONS***/
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_odom;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_vel;
        rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr subscriber_bat;

        void callback_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            x_dist = msg->pose.pose.position.x;
            y_dist = msg->pose.pose.position.y;

            // odom_msg.pose.pose.position.x = msg->pose->pose->position->x;
            // odom_msg.pose.pose.position.y = msg->pose->pose->position->y;
        }

        void callback_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            x_vel = msg->linear.x;
            y_vel = msg->linear.y;
        }

        void callback_bat(const sensor_msgs::msg::BatteryState::SharedPtr msg)
        {
            battery_percentage = msg->percentage;
            battery_voltage = msg->voltage;
        }

        void callback_timer()
        {
            time += 5;
            data[i][j++] = time;
            data[i][j++] = x_dist;
            data[i][j++] = y_dist;
            data[i][j++] = x_vel;
            data[i][j++] = y_vel;
            data[i][j++] = battery_percentage;
            data[i][j++] = battery_voltage;
            i++;
            j = 0;
            
            // std::cout << @data[i] << std::endl;
            // printf("%f", x_vel);

            RCLCPP_INFO(this->get_logger(), "CSV Line: %d", i);

            if (i == 60) 
            {
                std::ofstream myfile;
                myfile.open("my_data.csv");
                
                for (auto& row : data)
                {
                    for (auto col : row)
                        myfile << col << ',';
                    myfile << '\n';
                }
                myfile.close();

                RCLCPP_INFO(this->get_logger(), "CSV Saved.");
                
            }

        }


};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RecordDataNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
