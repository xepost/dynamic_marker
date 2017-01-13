#ifndef DECISIONPROCESSNODE_HPP
#define DECISIONPROCESSNODE_HPP

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"

#include <dynamic_reconfigure/server.h>
#include <dynamic_marker/dynamic_param_configConfig.h>

class DecisionProcessNode {
 public:
  DecisionProcessNode();

  ros::NodeHandle nh;

  // ROS message callbacks
  void marker_status_cb(const std_msgs::String marker_status_msg);
  void marker_pose_cb(const geometry_msgs::PoseStamped marker_pose_msg);
  void quad_odom_callback(const nav_msgs::Odometry& odo_msg);
  void dynamic_reconfigure_callback(
     dynamic_marker::dynamic_param_configConfig& config, uint32_t level);


  void measure_latency(void);

 private:
  ros::Subscriber m_marker_status_sub;
  ros::Subscriber m_quad_vel_sub;
  ros::Subscriber m_marker_pose_sub;
  ros::Publisher m_marker_size_pub;
  ros::Publisher m_debug_pub;  //! For debugging variables in rqt_plot
  // dynamic reconfigure server
  dynamic_reconfigure::Server<dynamic_marker::dynamic_param_configConfig>
      m_server;

  nav_msgs::Odometry m_odo_msg;
  ros::Time t;
  ros::Time old_t;
  double m_marker_size;
  bool m_marker_size_pub_lock;
};

#endif  // DECISIONPROCESSNODE_HPP

