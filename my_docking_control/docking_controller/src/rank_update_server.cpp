#include "controller.hpp"

void DockingController::rank_update_server(
            const std::shared_ptr<docking_interfaces::srv::RankUpdate::Request> request,
            const std::shared_ptr<docking_interfaces::srv::RankUpdate::Response> response)
{
    response->id = robot_id;
    response->distance = turtle_distance;
    response->battery = current_percent; 
}