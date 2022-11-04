#include "mrp_rvo/rvo.hpp"

namespace mrp_motion_planner
{
  RVO::RVO()
  {
  }

  RVO::~RVO()
  {
  }

  void RVO::initialise()
  {
  }

  void RVO::start()
  {
  }

  void RVO::stop()
  {
  }

  void RVO::setPath(const std::vector<geometry_msgs::msg::PoseStamped> &path)
  {
  }
  void RVO::calculateVelocityCommand(
      const geometry_msgs::msg::Pose &current_pose,
      geometry_msgs::msg::Twist &vel_cmd)
  {
  }

  void RVO::setMembersOdom(const std::vector<nav_msgs::msg::Odometry> &others_odom)
  {
  }

  void RVO::setLaserScan(const sensor_msgs::msg::LaserScan &scan)
  {
  }

  double RVO::getDistanceToGoal(const geometry_msgs::msg::Pose &current_pose)
  {
    
  }

  bool RVO::reachGoal()
  {
    
  }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mrp_motion_planner::RVO, mrp_local_server_core::MotionPlannerInterface)