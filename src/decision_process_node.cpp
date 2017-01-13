#include "dynamic_marker/decision_process_node.hpp"

DecisionProcessNode::DecisionProcessNode() {
  ros::NodeHandle params("~");
  // Topic Parameters
  std::string s;

  params.param<std::string>("marker_pose_topic", s, "/ar_multi_boards/pose");
  m_marker_pose_sub = nh.subscribe(s, 1, &DecisionProcessNode::marker_pose_cb, this,
                                ros::TransportHints().tcpNoDelay());


  params.param<std::string>("odometry_topic", s, "/ardrone/odometry");
  m_quad_vel_sub = nh.subscribe(s, 1, &DecisionProcessNode::quad_odom_callback, this,
                                ros::TransportHints().tcpNoDelay());

  params.param<std::string>("dynamic_marker_status_topic", s, "/dynamic_marker_status");
  m_marker_status_sub = nh.subscribe(s, 1, &DecisionProcessNode::marker_status_cb, this,
                                   ros::TransportHints().tcpNoDelay());

  params.param<std::string>("marker_size_topic", s, "/marker_size");
  m_marker_size_pub = nh.advertise<std_msgs::Int32>(s, 1);

  // Dynamic parameter reconfigure
  dynamic_reconfigure::Server<
      dynamic_marker::dynamic_param_configConfig>::CallbackType f;
  f = boost::bind(&DecisionProcessNode::dynamic_reconfigure_callback, this, _1, _2);
  m_server.setCallback(f);

  m_debug_pub = nh.advertise<std_msgs::Float64>("/ardrone_velocity/debug", 1);

  m_marker_size_pub_lock = false;
}

void DecisionProcessNode::marker_status_cb(const std_msgs::String marker_status_msg){
  if (marker_status_msg.data == "OK"){
    // Marker is already updated on the display
    // Measure time until this message
    // Decide to liberate the lock for sending new marker_size
    m_marker_size_pub_lock = false;
  }


}

void DecisionProcessNode::marker_pose_cb(const geometry_msgs::PoseStamped marker_pose_msg){
  double z = marker_pose_msg.pose.position.z;
  int marker_size;

  if (z > 0.6){
      marker_size = 150;
  } else {
    marker_size = z*150/0.6;
  }
  std_msgs::Int32 marker_size_msg;
  marker_size_msg.data = marker_size;
  if(!m_marker_size_pub_lock){
    m_marker_size_pub.publish(marker_size_msg);
  }
}

void DecisionProcessNode::quad_odom_callback(const nav_msgs::Odometry& odo_msg) {
  std_msgs::Float64 debug_msg;
  m_odo_msg = odo_msg;
  debug_msg.data = 0;
  m_debug_pub.publish(debug_msg);
}


void DecisionProcessNode::dynamic_reconfigure_callback(
  dynamic_marker::dynamic_param_configConfig& config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %f", config.marker_size);

  m_marker_size = config.marker_size;

  std_msgs::Int32 marker_size_msg;
  marker_size_msg.data = m_marker_size;
  m_marker_size_pub.publish(marker_size_msg);
}



// Debo esperar por confirmación desde el display si es que quiero mandar un nuevo tamaño de marcador
// La forma mas sencilla es implementar un delay para no estar cambiando tanto de marcador
// En ar_sys debo buscar la forma de modificar el tamaño del marcador dinámicamente
