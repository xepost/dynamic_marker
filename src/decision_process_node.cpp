#include "dynamic_marker/decision_process_node.hpp"

DecisionProcessNode::DecisionProcessNode() {
  ros::NodeHandle params("~");
  // Topic Parameters
  std::string s;

  // Subscribed topics configuration
  params.param<std::string>("oberver_distance_topic", s, "/ardrone/navdata_altitude");
  observer_distance_sub_ = nh_.subscribe(s, 1, &DecisionProcessNode::observer_distance_cb, this,
                                ros::TransportHints().tcpNoDelay());


  params.param<std::string>("marker_pose_topic", s, "/ar_multi_boards/pose");
  marker_pose_sub_ = nh_.subscribe(s, 1, &DecisionProcessNode::marker_pose_cb, this,
                                ros::TransportHints().tcpNoDelay());


  params.param<std::string>("marker_status_topic", s, "/dynamic_marker/set_marker_response");
  marker_status_sub_ = nh_.subscribe(s, 1, &DecisionProcessNode::marker_status_cb, this,
                                   ros::TransportHints().tcpNoDelay());

  // Published topics
  params.param<std::string>("set_marker_topic", s, "/dynamic_marker/set_marker");
  marker_size_pub_ = nh_.advertise<std_msgs::Int32>(s, 1);

  // Services


  // Dynamic parameter reconfigure
  dynamic_reconfigure::Server<
      dynamic_marker::dynamic_param_configConfig>::CallbackType f;
  f = boost::bind(&DecisionProcessNode::dynamic_reconfigure_callback, this, _1, _2);
  server_.setCallback(f);


  marker_size_pub_lock_ = false;
}

void DecisionProcessNode::marker_status_cb(const std_msgs::String marker_status_msg){
  if (marker_status_msg.data == "OK"){
    // Marker is already updated on the display
    // Measure time until this message
    // Decide to liberate the lock for sending new marker_size
    marker_size_pub_lock_ = false;
  }
}

void DecisionProcessNode::marker_pose_cb(const geometry_msgs::PoseStamped marker_pose_msg){
  double z = marker_pose_msg.pose.position.z;
  int marker_size;
  /*
  if (z > 0.6){
      marker_size = 150;
  } else {
    marker_size = z*150/0.6;
  }
  std_msgs::Int32 marker_size_msg;
  marker_size_msg.data = marker_size;
  if(!marker_size_pub_lock_){
    marker_size_pub_.publish(marker_size_msg);
    dynamic_marker::SetMarker::Request req;
    dynamic_marker::SetMarker::Response res;
    req.marker_family = 0; //aruco
    req.marker_id = 0; // id 0
    req.marker_size = marker_size;
    set_marker_srv_client_.call(req,res);
  }
  */
}


void DecisionProcessNode::observer_distance_cb(const ardrone_autonomy::navdata_altitude altitude_msg){
  int altitude =  altitude_msg.altitude_raw; // in milimeters


  // llamar al servicio (en la pantalla externa)
  // llamar al servicio en ar_sys
}



void DecisionProcessNode::dynamic_reconfigure_callback(
  dynamic_marker::dynamic_param_configConfig& config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %f", config.marker_size);
  marker_size_ = config.marker_size;
  std_msgs::Int32 marker_size_msg;
  marker_size_msg.data = marker_size_;
  marker_size_pub_.publish(marker_size_msg);
}
