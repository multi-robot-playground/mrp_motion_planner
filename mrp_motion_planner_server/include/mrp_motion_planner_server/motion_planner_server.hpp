#ifndef MRP_MOTION_PLANNER_SERVER__MOTION_PLANNER_SERVER_HPP_
#define MRP_MOTION_PLANNER_SERVER__MOTION_PLANNER_SERVER_HPP_

#include <chrono>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <pluginlib/class_loader.hpp>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/follow_path.hpp"

#include "mrp_common/lifecycle_node.hpp"
#include "mrp_common/service_client.hpp"
#include "mrp_common/action_server.hpp"
#include "mrp_common/logging.hpp"

#include "mrp_local_server_core/local_motion_planner.hpp"

#include "mrp_comms_msgs/srv/get_all_teams.hpp"
#include "mrp_comms_msgs/srv/get_members_in_team.hpp"

namespace mrp_motion_planner
{
  class MotionPlannerServer : public mrp_common::LifecycleNode
  {
  public:
    MotionPlannerServer(const std::string &planner_name);
    virtual ~MotionPlannerServer();

    bool initialise();
    bool start();
    bool stop();

    bool loadPlanner(const std::string &planner_name);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State &state) override;

    bool setWaypoints(const std::vector<geometry_msgs::msg::Pose> waypoints);
    bool getRobotCurrentPose(geometry_msgs::msg::Pose &pose) const;

    void setMemberRobotNames(const std::vector<std::string> &robot_names);

  protected:
    static const std::string FALLBACK_PLANNER;
    struct RobotOdom
    {
      nav_msgs::msg::Odometry current_odom;
      std::recursive_mutex mtx;
      std::atomic<bool> ready{false};
    };

    // Motion planner
    std::shared_ptr<pluginlib::ClassLoader<mrp_local_server_core::MotionPlannerInterface>> loader_ptr_;
    std::shared_ptr<mrp_local_server_core::MotionPlannerInterface> planner_ptr_;
    std::chrono::milliseconds planner_rate_;

    // Robot related
    std::string robot_name_;
    std::string planner_name_;
    std::map<std::string, std::string> planner_name_map_;

    // Robot Odom
    std::shared_ptr<RobotOdom> robot_odom_;

    // Publisher for cmd_vel
    std::string robot_cmd_vel_topic_name_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr robot_cmd_vel_pub_;
    void createCmdVelPublisher();

    // Subscriber for getting current pose of the robot
    std::string robot_odom_topic_name_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot_odom_sub_;
    void createOdomSubscriber();

    // Action server for path execution
    std::shared_ptr<mrp_common::ActionServer<nav2_msgs::action::FollowPath>> follow_path_action_server_;
    void createFollowPathActionServer();

    // Other robots related
    std::vector<std::string> member_robots_names_;
    std::map<std::string, rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> member_robots_odom_sub_map_;
    std::map<std::string, std::shared_ptr<RobotOdom>> member_robots_odom_data_map_;
    std::atomic<bool> all_members_odom_ready_{false};
    void registerMemberRobots();

    // Service client to ask for team configuration
    std::shared_ptr<mrp_common::ServiceClient<mrp_comms_msgs::srv::GetAllTeams>> get_team_client_;
    void createGetTeamClient();

    std::shared_ptr<mrp_common::ServiceClient<mrp_comms_msgs::srv::GetMembersInTeam>> get_all_robots_in_team_client_;
    void createGetMemberInTeamClient();

    void getAllTeams(std::vector<int> &team_id_list);
    void getAllMembersInTeam(const int &team_id, std::vector<std::string> &member_names);

    void followPath();
    
    void updatePath();
    void computeAndPublishVelocity();
    bool reachEndOfPath();

    // Other utils functions
    void publishVelocity(const geometry_msgs::msg::Twist &control_velocity);
    void publishZeroVelocity();
  };
}

#endif