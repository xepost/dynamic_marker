#include "dynamic_marker/decision_process_node.hpp"

DecisionProcessNode::DecisionProcessNode() {
  ros::NodeHandle params("~");
  // Topic Parameters
  std::string s;

  // Subscribed topics
  params.param<std::string>("oberver_distance_topic", s, "/ardrone/navdata_altitude");
  observer_distance_sub_ = nh_.subscribe(s, 1, &DecisionProcessNode::observer_distance_cb, this,
                                ros::TransportHints().tcpNoDelay());


  params.param<std::string>("ar_sys_marker_pose_topic", s, "/ar_multi_boards/pose");
  ar_sys_marker_pose_sub_ = nh_.subscribe(s, 1, &DecisionProcessNode::ar_sys_marker_pose_cb, this,
                                ros::TransportHints().tcpNoDelay());

  params.param<std::string>("whycon_marker_pose_topic", s, "/whycon/pose");
  whycon_marker_pose_sub_ = nh_.subscribe(s, 1, &DecisionProcessNode::whycon_marker_pose_cb, this,
                                ros::TransportHints().tcpNoDelay());


  params.param<std::string>("marker_status_topic", s, "/dynamic_marker/set_marker_response");
  set_marker_response_sub_ = nh_.subscribe(s, 1, &DecisionProcessNode::marker_response_cb, this,
                                   ros::TransportHints().tcpNoDelay());

  // Published topics
  params.param<std::string>("set_marker_topic", s, "/dynamic_marker/set_marker");
  set_marker_pub_ = nh_.advertise<dynamic_marker::set_marker>(s, 1);

  // Published topics
  params.param<std::string>("output_pose_topic", s, "/dynamic_marker/pose");
  output_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(s, 1);

  // Dynamic parameter reconfigure
  dynamic_reconfigure::Server<
      dynamic_marker::dynamic_param_configConfig>::CallbackType f;
  f = boost::bind(&DecisionProcessNode::dynamic_reconfigure_callback, this, _1, _2);
  server_.setCallback(f);
}

void DecisionProcessNode::marker_response_cb(const std_msgs::String marker_status_msg){
  if (marker_status_msg.data == "OK"){
    // Marker is already updated on the display
    // Measure time until this message
    // Decide to liberate the lock for sending new marker_size
    //marker_size_pub_lock_ = false;
  }
}

void DecisionProcessNode::ar_sys_marker_pose_cb(const geometry_msgs::PoseStamped marker_pose_msg){
  // we check first that we are indeed tracking an ar_sys marker
  if (set_marker_msg_.marker_family == aruco_single ||
      set_marker_msg_.marker_family == aruco_multi){

    //double z = marker_pose_msg.pose.position.z;


    //publish the pose to the output pose topic
    output_pose_pub_.publish(marker_pose_msg);


  }


}

void DecisionProcessNode::whycon_marker_pose_cb(const geometry_msgs::PoseStamped marker_pose_msg){
  // we check first that we are indeed tracking an whycon marker
  if (set_marker_msg_.marker_family == whycon){

    //double z = marker_pose_msg.pose.position.z;

    //publish the pose to the output pose topic
    output_pose_pub_.publish(marker_pose_msg);

  }

}


void DecisionProcessNode::observer_distance_cb(const ardrone_autonomy::navdata_altitude altitude_msg){
  int altitude =  altitude_msg.altitude_raw; // in millimeters
  int th_whycon = 3000; // whycon threshold in millimiters
  int th_aruco_single = 500; // aruco_single board threshold in millimiters
  int marker_size; //side lenght (aruco) or diameter (whycon) in millimiters



  // Decision process (select marker size and type)

  if (altitude >= th_whycon){
    // We use whycon
    // only one size
    set_marker_msg_.marker_family = whycon;
    set_marker_msg_.marker_id = 0; //default?
    set_marker_msg_.marker_size = 150; // max screen lenght?

  } else if (altitude > th_aruco_single && altitude < th_whycon){
    // We use aruco single ID 88
    // Size changes dynamically
    set_marker_msg_.marker_family = aruco_single;
    set_marker_msg_.marker_id = 88;
    set_marker_msg_.marker_size = 150; //! TODO: changes linearly with distance.. we have to develop this equation


  } else if (altitude <= th_aruco_single){
    // We use aruco multi
    // Size changes dynamically
    set_marker_msg_.marker_family = aruco_multi;
    set_marker_msg_.marker_id = 0; //default?
    set_marker_msg_.marker_size = 150; //! TODO: changes linearly with distance.. we have to develop this equation

  }

  //!TODO: remove this when the above part is ready
  // Only aruco for testing
  set_marker_msg_.marker_family = aruco_single;
  set_marker_msg_.marker_id = 88;
  set_marker_msg_.marker_size = altitude*150/600; //with our camera 600 mm is a good distance to recognize a 150 mm marker


  // We define that the pose form ar_sys as invalid until we get confirmation
  display_marker_updated_ = false;
  ar_sys_pose_valid_ = false;    //!TODO: (rosa) change this variable to true when ar_sys changes the configuration

  // Set the marker in the display screen server
  set_marker_pub_.publish(set_marker_msg_);




  // Set the marker in the ar_sys node




  //!TODO: ROSA

}



void DecisionProcessNode::dynamic_reconfigure_callback(
  dynamic_marker::dynamic_param_configConfig& config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %f", config.marker_size);
  marker_size_ = config.marker_size;
  std_msgs::Int32 marker_size_msg;
  marker_size_msg.data = marker_size_;
  set_marker_pub_.publish(marker_size_msg);
}
