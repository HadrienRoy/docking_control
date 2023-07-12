#include "controller.hpp"

void DockingController::rank_update_server(
            const std::shared_ptr<docking_interfaces::srv::RankUpdate::Request> request,
            const std::shared_ptr<docking_interfaces::srv::RankUpdate::Response> response)
{
    response->id = robot_id;
    // response->distance = turtle_distance;
    response->battery = current_percent; 

    if (sim_2d)
    {
        // request->distance = sqrt((init_x_pose*init_x_pose) + (init_y_pose*init_y_pose));
        response->distance = turtle_2d_distance;
    }
    else
    {
        response->distance = turtle_distance;
    }

}