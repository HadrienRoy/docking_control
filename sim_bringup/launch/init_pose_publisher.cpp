#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class InitPosePublisher : public rclcpp::Node
{
    public:
        InitPosePublisher() : Node("init_pose_publisher")
        {
            /*** Declare Parameters ***/
            this->declare_parameter<std::string>("robot_id", "tb3_1");
            this->get_parameter("robot_id",robot_id);

            this->declare_parameter<float>("init_x_pose", 0.0);
            this->get_parameter("init_x_pose", init_x_pose);

            this->declare_parameter<float>("init_y_pose", 0.0);
            this->get_parameter("init_y_pose", init_y_pose);

            init_pose_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);

        }

        void run()
        {
            geometry_msgs::msg::PoseWithCovarianceStamped init_pose;
            init_pose.header.stamp = this->get_clock()->now();
            init_pose.header.frame_id = "map";
            
            init_pose.pose.pose.position.x = init_x_pose;
            init_pose.pose.pose.position.y = init_y_pose;
            init_pose.pose.pose.orientation.x = 0.0;
            init_pose.pose.pose.orientation.y = 0.0;
            init_pose.pose.pose.orientation.z = 0.0;
            init_pose.pose.pose.orientation.w = 1.0;
            
            std::array<double, 36> covariance= {0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                    0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0,  0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0,  0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0,  0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0,  0.0, 0.0, 0.0, 0.0, 0.06853891945200942};
            init_pose.pose.covariance = covariance;

            init_pose_pub->publish(init_pose);
        }

    private:
        /*** Declare Variables ***/
        std::string robot_id;
        float init_x_pose;
        float init_y_pose;

        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_pub;       
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InitPosePublisher>();
    rclcpp::Rate rate(0.1);
    rate.sleep();
    // int count = 0;
    // while (count<=5)
    // {
    //     rate.sleep();
    //     count++;
    //     node->run();
    //     rclcpp::spin_some(node);
    // }
    node->run();
    rclcpp::spin(node);
    rate.sleep();
    

    rclcpp::shutdown();
    return 0;
}
