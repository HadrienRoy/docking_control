#include "controller.hpp"

void DockingController::queue_update_client(std::string type)
{
    // Create client
    auto client = this->create_client<docking_interfaces::srv::QueueUpdate>("scheduler/queue_update_service");
    while (!client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
    }

    // Create request
    auto request = std::make_shared<docking_interfaces::srv::QueueUpdate::Request>();
    request->type = type;
    request->id = robot_id;
    request->distance= 20.0; // dummy
    request->battery= 15.0; //dummy

    auto future = client->async_send_request(request);

    try
    {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Queue Update service request (type:=%s)successful.", type.c_str());
    }
    catch (const std::exception &e)
    {
        RCLCPP_INFO(this->get_logger(), "Queue Update service request (type:=%s) failed.", type.c_str());
    }
}