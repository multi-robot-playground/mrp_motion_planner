#include "mrp_spotturn_controller/spotturn_controller.hpp"

namespace mrp_motion_planner
{

  SpotTurn::SpotTurn()
  {
    current_waypoint_indx_ = 0;
    setLinearMax(0.22);
    setAngularMax(2.5);
    setLinearError(0.01);
    setAngularError(0.01);
    at_position_ = false;
    reach_goal_ = false;
  }

  SpotTurn::~SpotTurn()
  {
  }

  void SpotTurn::setLinearMax(const double &linear_max)
  {
    max_linear_vel_ = linear_max;
  }
  void SpotTurn::setAngularMax(const double &angular_max)
  {
    max_angular_vel_ = angular_max;
  }
  void SpotTurn::setLinearError(const double &linear_err)
  {
    linear_error_ = linear_err;
  }
  void SpotTurn::setAngularError(const double &angular_err)
  {
    angular_error_ = angular_err;
  }

  void SpotTurn::initialise()
  {
    std::cout << "Initialising spotturn controller" << std::endl;
  }

  void SpotTurn::setPath(const std::vector<geometry_msgs::msg::PoseStamped> &path)
  {
    path_ = path;
    current_waypoint_indx_ = 0;
    at_position_ = false;
    reach_goal_ = false;
  }

  void SpotTurn::calculateVelocityCommand(
      const geometry_msgs::msg::Pose &current_pose,
      geometry_msgs::msg::Twist &vel_cmd)
  {
    if(current_waypoint_indx_ == path_.size())
    {
      return;
    }
    geometry_msgs::msg::Pose current_waypoint = path_.at(current_waypoint_indx_).pose;
    vel_cmd.angular.z = calculateAngularVelocity(current_pose, current_waypoint);
    vel_cmd.linear.x = calculateLinearVelocity(current_pose, current_waypoint);

    if (vel_cmd.angular.z == 0 && vel_cmd.linear.x == 0 && at_position_)
    {
      if (current_waypoint_indx_ < path_.size())
      {
        current_waypoint_indx_++;
      }
      else
      {
        reach_goal_ = true;
      }
      at_position_ = false;
    }
  }

  double SpotTurn::getDistanceToGoal(const geometry_msgs::msg::Pose &current_pose)
  {
    geometry_msgs::msg::Pose current_waypoint = path_.at(current_waypoint_indx_).pose;
    return mrp_common::GeometryUtils::euclideanDistance(current_pose, current_waypoint);
  }

  bool SpotTurn::reachGoal()
  {
    return reach_goal_;
  }

  void SpotTurn::setMembersOdom(const std::vector<nav_msgs::msg::Odometry> &others_odom)
  {
  }

  void SpotTurn::setLaserScan(const sensor_msgs::msg::LaserScan &scan)
  {
  }

  double SpotTurn::calculateLinearVelocity(const geometry_msgs::msg::Pose &current_pose,
                                                     const geometry_msgs::msg::Pose &current_waypoint)
  {
    double distance = mrp_common::GeometryUtils::euclideanDistance(current_pose, current_waypoint);

    double x1 = current_pose.position.x;
    double x2 = current_waypoint.position.x;
    double current_yaw = mrp_common::GeometryUtils::yawFromPose(current_pose);

    double y1 = current_pose.position.y;
    double y2 = current_waypoint.position.y;
    double target_yaw = mrp_common::GeometryUtils::yawFromPose(current_waypoint);

    double theta = atan2((y2 - y1), (x2 - x1)) - current_yaw;
    if (theta > M_PI)
    {
      theta = theta - 2 * M_PI;
    }
    else if (theta < -M_PI)
    {
      theta = theta + 2 * M_PI;
    }

    if (distance <= linear_error_)
    {
      distance = 0;
      at_position_ = true;
    }

    // Linear
    double linear_vel = distance;
    if (distance > max_linear_vel_)
    {
      linear_vel = max_linear_vel_;
    }

    if (abs(theta) < angular_error_)
    {
      linear_vel = distance;

      if (linear_vel > max_linear_vel_)
      {
        linear_vel = max_linear_vel_;
      }
    }
    else
    {
      linear_vel = 0;
    }
    return linear_vel;
  }
  double SpotTurn::calculateAngularVelocity(const geometry_msgs::msg::Pose &current_pose,
                                                      const geometry_msgs::msg::Pose &current_waypoint)
  {
    double x1 = current_pose.position.x;
    double x2 = current_waypoint.position.x;
    double current_yaw = mrp_common::GeometryUtils::yawFromPose(current_pose);

    double y1 = current_pose.position.y;
    double y2 = current_waypoint.position.y;
    double target_yaw = mrp_common::GeometryUtils::yawFromPose(current_waypoint);

    double theta = atan2((y2 - y1), (x2 - x1)) - current_yaw;

    if (theta > M_PI)
    {
      theta = theta - 2 * M_PI;
    }
    else if (theta < -M_PI)
    {
      theta = theta + 2 * M_PI;
    }

    if (abs(theta) <= angular_error_)
    {
      theta = 0;
    }

    if (at_position_)
    {
      theta = target_yaw - current_yaw;
      if (abs(theta) <= angular_error_)
      {
        theta = 0;
      }
    }

    double angular_vel = theta;
    if (abs(theta) > max_angular_vel_)
    {
      angular_vel = (theta / abs(theta)) * max_angular_vel_;
    }

    return angular_vel;
  }
} // namespace spotturn_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mrp_motion_planner::SpotTurn, mrp_local_server_core::MotionPlannerInterface)