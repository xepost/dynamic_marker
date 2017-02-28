#ifndef DECISIONPROCESSNODE_HPP
#define DECISIONPROCESSNODE_HPP

#include "ros/ros.h"
#include "tf2/convert.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/TransformStamped.h"
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
  void quadcopter_altitude_cb(const ardrone_autonomy::navdata_altitude altitude_msg); // Distance in milimiters between the quadcopter and the marker
  void marker_control(const double marker_cam_distance);
  void set_marker_response_cb(const dynamic_marker::set_marker_response marker_response_msg);
  void ar_sys_marker_transform_cb(const geometry_msgs::TransformStamped ar_sys_marker_transform_msg);
  void whycon_marker_pose_cb(const geometry_msgs::PoseArray marker_pose_array_msg);
  void set_new_fiducial_marker(int marker_family, int marker_id, double marker_size, std::string marker_frame_name);
  void timer_set_marker_cb(const ros::TimerEvent&);
  void skip_one_frame_timer_cb(const ros::TimerEvent&);
  void calc_cam_fov_(double fx, double fy, int img_width, int img_height, double& fovx, double& fovy);
  void calc_max_tilt_(double fovx, double marker_size, double marker_cam_distance);
  void calc_ideal_marker_size_(double fovx, double max_quad_tilt, double marker_cam_distance, double tracking_error, double &marker_ideal_size);


  void dynamic_reconfigure_callback(
     dynamic_marker::dynamic_param_configConfig& config, uint32_t level);


  void measure_latency(void);



 private:
  ros::Subscriber observer_distance_sub_;
  ros::Subscriber set_marker_response_sub_;    //TODO: remove by using a service
  ros::Subscriber ar_sys_marker_transform_sub_;
  ros::Subscriber whycon_marker_pose_sub_;

  ros::ServiceClient ar_sys_board_service_client_;


  ros::Publisher output_pose_pub_;
  ros::Publisher set_marker_pub_;
  ros::Publisher output_transform_pub_;
  ros::Publisher experimental_transform_pub_;

  tf2_ros::TransformBroadcaster tf2_broadcaster_;

  // dynamic reconfigure server
  dynamic_reconfigure::Server<dynamic_marker::dynamic_param_configConfig>
      server_;

  enum marker_family_enum_ { whycon, aruco_single, aruco_multi, pitag};
  dynamic_marker::set_marker marker_msg_to_display_;

  // Variables to check that the dynamic display and marker recognition
  // are using the must current marker

  bool display_marker_updated_;
  bool current_marker_detection_is_enabled_;
  bool set_marker_cycle_ended_;
  ros::Duration delay_image_capture_;
  ros::Time set_new_marker_instant_;

  ros::Timer timer_set_marker_;
  ros::Timer timer_skip_one_frame_;
  double camera_fps_;
  double threshold_aruco_single_; // This should be a function of the distance and marker size....
  double threshold_aruco_multi_; // This is hard coded, should be the default size of aruco_board
  double marker_fov_;
  double max_marker_size_;

  int last_marker_family_;
  int last_marker_id_;
  int last_marker_size_;
  int current_marker_family_;
  int current_marker_id_;
  double current_marker_size_;
  std::string current_marker_frame_name_;
};


#endif  // DECISIONPROCESSNODE_HPP



