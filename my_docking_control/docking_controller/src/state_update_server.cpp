#include "controller.hpp"

void DockingController::state_update_server(
            const std::shared_ptr<docking_interfaces::srv::StateUpdate::Request> request,
            const std::shared_ptr<docking_interfaces::srv::StateUpdate::Response> response)
{

    set_queue_info(request->queue_num, request->queue_state);
    ready_queue_state = true;

    // RCLCPP_INFO(this->get_logger(), "Queue State: %s", queue_state.c_str());
    // RCLCPP_INFO(this->get_logger(), "Queue Number: %d", queue_num);

    response->success = true;
}