/*********************************************************************
 * Author: Vineet Tambe
 *********************************************************************/
#ifndef DDG_WAITSTEP_AT_WAYPOINT_HPP_
#define DDG_WAITSTEP_AT_WAYPOINT_HPP_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <string>

#include "nav2_core/waypoint_task_executor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ddg_waitstep_at_waypoint_plugin {

class WaitstepAtWaypoint : public nav2_core::WaypointTaskExecutor {
 public:
  /**
   * @brief Construct a new Wait At Waypoint Arrival object
   *
   */
  WaitstepAtWaypoint();

  /**
   * @brief Destroy the Wait At Waypoint Arrival object
   *
   */
  ~WaitstepAtWaypoint();

  /**
   * @brief declares and loads parameters used (waypoint_pause_duration_)
   *
   * @param parent parent node that plugin will be created
   * withing(waypoint_follower in this case)
   * @param plugin_name
   */
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
                  const std::string& plugin_name);

  /**
   * @brief Override this to define the body of your task that you would like to
   * execute once the robot arrived to waypoint
   *
   * @param curr_pose current pose of the robot
   * @param curr_waypoint_index current waypoint, that robot just arrived
   * @return true if task execution was successful
   * @return false if task execution failed
   */
  bool processAtWaypoint(const geometry_msgs::msg::PoseStamped& curr_pose,
                         const int& curr_waypoint_index);

 protected:
  // the robot will sleep waypoint_pause_duration_ milliseconds
  int waypoint_pause_duration_;
  bool is_enabled_;
  rclcpp::Logger logger_{rclcpp::get_logger("nav2_waypoint_follower")};
  geometry_msgs::msg::PoseStamped prev_pose;
};

}  // namespace ddg_waitstep_at_waypoint_plugin

#endif  // DDG_WAITSTEP_AT_WAYPOINT_HPP_
