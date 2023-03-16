#include "controller.hpp"

void DockingController::docking_server(
            const std::shared_ptr<docking_interfaces::srv::Docking::Request> request,
            const std::shared_ptr<docking_interfaces::srv::Docking::Response> response)
{
    if (docking_state == "docked") set_docking_state("");

    if ((request->service == "start") && (docking_state == "")) 
    {
        RCLCPP_INFO(this->get_logger(), "Docking started");
        set_docking_state("start");
        response->success = true;
        
        start_tag_detection = true;
    } 
    else if (request->service == "cancel") 
    {
        turtlebot_stop();
        set_docking_state("");
        response->success = true;
    } 
    else 
    {
        response->success = false;
        RCLCPP_INFO(this->get_logger(), "Unknown action to autodocking service.");
    }
}