#ifndef DECISIONPROCESSNODE_HPP
#define DECISIONPROCESSNODE_HPP

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "ardrone_autonomy/navdata_altitude.h"
#include "dynamic_marker/set_marker.h"
#include "dynamic_marker/set_marker_response.h"

#include <dynamic_reconfigure/server.h>
#include <dynamic_marker/dynamic_param_configConfig.h>

class DecisionProcessNode {
 public:
  DecisionProcessNode();

  ros::NodeHandle nh_;

  // ROS message callbacks
  void observer_distance_cb(const ardrone_autonomy::navdata_altitude altitude_msg); // Distance in milimiters between the observer (camera) and the marker
  void marker_status_cb(const std_msgs::String marker_status_msg);
  void marker_pose_cb(const geometry_msgs::PoseStamped marker_pose_msg);
  void dynamic_reconfigure_callback(
     dynamic_marker::dynamic_param_configConfig& config, uint32_t level);


  void measure_latency(void);

 private:
  ros::Subscriber observer_distance_sub_;
  ros::Subscriber marker_status_sub_;    //TODO: remove by using a service
  ros::Subscriber marker_pose_sub_;

  ros::Publisher marker_size_pub_;

  // dynamic reconfigure server
  dynamic_reconfigure::Server<dynamic_marker::dynamic_param_configConfig>
      server_;

  double marker_size_;
  bool marker_size_pub_lock_;
};

#endif  // DECISIONPROCESSNODE_HPP


