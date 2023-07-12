#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class GazeboBatteryState : public rclcpp::Node
{
    public:
        GazeboBatteryState() : Node("gazebo_battery_state")
        {
            /*** Declare Parameters ***/
            this->declare_parameter<std::string>("robot_id", "tb3_1");
            this->get_parameter("robot_id",robot_id);

            /*** Define Publishers & Services ***/
            battery_pub = this->create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 10);

            /*** Define Variable ***/
            timer = this->create_wall_timer(
                1s,
                std::bind(&GazeboBatteryState::update_gazebo_battery_state, this));

            // Use Turtlebot3 battery data
            battery_state_msg.voltage = 11.1;
            battery_state_msg.temperature = 20.0;
            battery_state_msg.current = -36.0;
            battery_state_msg.charge = 1.8;
            battery_state_msg.capacity = 1.8;
            battery_state_msg.design_capacity = 1.8;
            battery_state_msg.percentage = 100.0;
            battery_state_msg.power_supply_status = battery_state_msg.POWER_SUPPLY_STATUS_DISCHARGING;
            battery_state_msg.power_supply_health = battery_state_msg.POWER_SUPPLY_HEALTH_GOOD;
            battery_state_msg.power_supply_technology = battery_state_msg.POWER_SUPPLY_TECHNOLOGY_LIPO;
            battery_state_msg.present = true;

            if (robot_id == "tb3_1"){battery_state_msg.percentage = 100.0; battery_level = 100.00;}
            else if (robot_id == "tb3_2"){battery_state_msg.percentage = 100.0; battery_level = 100.00;}
            else if (robot_id == "tb3_3"){battery_state_msg.percentage = 100.0; battery_level = 100.00;}
            else if (robot_id == "tb3_4"){battery_state_msg.percentage = 100.0; battery_level = 100.00;}
            else if (robot_id == "tb3_5"){battery_state_msg.percentage = 100.0; battery_level = 100.00;}
            else if (robot_id == "tb3_0"){battery_state_msg.percentage = 100.0; battery_level = 100.00;}

            RCLCPP_INFO(this->get_logger(), "Gazebo Battery State Node has been started.");
        }

    private:
        /*** Declare Variables ***/
        rclcpp::TimerBase::SharedPtr timer;
        int counter;
        float battery_level;

        std::string robot_id;

        std::string cmd_battery;
        std::string name;

        /*** Declare Publishers & Services ***/
        rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub;
    
        /*** Declare Messages ***/
        sensor_msgs::msg::BatteryState battery_state_msg;

        /*** Declare Functions ***/
        void update_gazebo_battery_state()
        {
            counter += 1;

            

            // if (robot_id == "tb3_2" && counter % 1 == 0)
            // {
            //     counter = 0;

            //     battery_level -= 1.00;
            //     battery_state_msg.percentage = battery_level;
            // }
            if (counter % 1 == 0)
            {
                counter = 0;

                battery_level -= 1.00;
                battery_state_msg.percentage = battery_level;
            }

            battery_pub->publish(battery_state_msg);
        }


};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GazeboBatteryState>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
