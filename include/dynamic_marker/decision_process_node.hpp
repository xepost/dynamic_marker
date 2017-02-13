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

#include "ar_sys/Board_service.h"
#include "ar_sys/Board_serviceResponse.h"


#include <dynamic_reconfigure/server.h>
#include <dynamic_marker/dynamic_param_configConfig.h>

class DecisionProcessNode {
 public:
  DecisionProcessNode();

  ros::NodeHandle nh_;

  // ROS message callbacks
  void observer_distance_cb(const ardrone_autonomy::navdata_altitude altitude_msg); // Distance in milimiters between the observer (camera) and the marker
  void set_marker_response_cb(const dynamic_marker::set_marker_response marker_response_msg);
  void ar_sys_marker_pose_cb(const geometry_msgs::PoseStamped marker_pose_msg);
  void whycon_marker_pose_cb(const geometry_msgs::PoseStamped marker_pose_msg);
  void set_new_fiducial_marker(int marker_family, int marker_id, double marker_size, std::string marker_frame_name);

  void dynamic_reconfigure_callback(
     dynamic_marker::dynamic_param_configConfig& config, uint32_t level);


  void measure_latency(void);



 private:
  ros::Subscriber observer_distance_sub_;
  ros::Subscriber set_marker_response_sub_;    //TODO: remove by using a service
  ros::Subscriber ar_sys_marker_pose_sub_;
  ros::Subscriber whycon_marker_pose_sub_;

  ros::ServiceClient ar_sys_board_service_client_;


  ros::Publisher output_pose_pub_;
  ros::Publisher set_marker_pub_;

  // dynamic reconfigure server
  dynamic_reconfigure::Server<dynamic_marker::dynamic_param_configConfig>
      server_;

  enum marker_family_enum_ { whycon, aruco_single, aruco_multi, pitag};
  dynamic_marker::set_marker set_marker_msg_;

  // Variables to check that the dynamic display and marker recognition
  // are using the must current marker

  bool display_marker_updated_;
  bool ar_sys_marker_updated_;
};

#endif  // DECISIONPROCESSNODE_HPP


