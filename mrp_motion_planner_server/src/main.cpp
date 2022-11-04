#include "mrp_motion_planner_server/motion_planner_server.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor1 =
      std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  std::shared_ptr<mrp_motion_planner::MotionPlannerServer> motion_planner_server =
      std::make_shared<mrp_motion_planner::MotionPlannerServer>("spotturn");
  executor1->add_node(motion_planner_server->get_node_base_interface());
  executor1->spin();
  return 0;
}