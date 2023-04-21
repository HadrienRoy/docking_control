#include "controller.hpp"

void DockingController::state_update_server(
            const std::shared_ptr<docking_interfaces::srv::StateUpdate::Request> request,
            const std::shared_ptr<docking_interfaces::srv::StateUpdate::Response> response)
{
    queue_state = request->queue_state;
    queue_num = request->queue_num;

    RCLCPP_INFO(this->get_logger(), "Queue State: %s", queue_state.c_str());
    RCLCPP_INFO(this->get_logger(), "Queue Number: %d", queue_num);

    response->success = true;
}