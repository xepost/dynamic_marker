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
  set_marker_response_sub_ = nh_.subscribe(s, 1, &DecisionProcessNode::set_marker_response_cb, this,
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


  // Service in ar_sys to change the marker settings
  ar_sys_board_service_client_ = nh_.serviceClient<ar_sys::Board_service>("/ar_multi_boards/changeboard");

  // The marker display should start with an Aruco single marker
  // using the same default values as ar_sys
  display_marker_updated_ = true;
  ar_sys_marker_updated_ = true;
  set_marker_msg_.message_id = 0;
}

void DecisionProcessNode::set_marker_response_cb(const dynamic_marker::set_marker_response marker_response_msg){
  ROS_INFO("Marker is updated on the display");
  if (marker_response_msg.result && marker_response_msg.message_id == set_marker_msg_.message_id){
    //The marker was succesfully updated on the screen
    display_marker_updated_ = true;
    ar_sys_marker_updated_ = true; //!TODO: (ROSA) this doesnt belong here, it should be made valid by the ar_sys node using another topic or service response (your implementation)

  } else{
    //!TODO do something really smart in case that fails
    ROS_DEBUG("display screen couldnt update the marker with the current configuration:");
    ROS_DEBUG("dynamic_marker_screen response: %s",marker_response_msg.message.c_str());
  }
}

void DecisionProcessNode::ar_sys_marker_pose_cb(const geometry_msgs::PoseStamped marker_pose_msg){
  // we check first that we are indeed tracking an ar_sys marker
  if (set_marker_msg_.marker_family == aruco_single ||
      set_marker_msg_.marker_family == aruco_multi){

    //We check here that the current marker is being displayed on the screen
    // and that ar_sys is already configured for the new marker

    if(display_marker_updated_ && ar_sys_marker_updated_){
      //publish the pose to the output pose topic
      output_pose_pub_.publish(marker_pose_msg);
      //!TODO: RAUL for tracking also it is necessary to update the trasnformation tree
    }
  }
}

void DecisionProcessNode::whycon_marker_pose_cb(const geometry_msgs::PoseStamped marker_pose_msg){
  // we check first that we are indeed tracking an whycon marker
  if (set_marker_msg_.marker_family == whycon){
    if(display_marker_updated_){
      //publish the pose to the output pose topic
      output_pose_pub_.publish(marker_pose_msg);
      //!TODO: RAUL for tracking also it is necessary to update the trasnformation tree
      //! Since whycon doesnt provide POSE we can use the measurements of the quadcopter IMU for angles
    }
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
  ar_sys_marker_updated_ = false;    //!TODO: (rosa) change this variable to true when ar_sys changes the configuration

  // Set the marker in the display screen server
  ROS_INFO("Sending new marker to display");
  set_marker_msg_.message_id ++; //! TODO remember to do this everytime for checking the ID maybe a function to force it
  set_marker_pub_.publish(set_marker_msg_);

  // Set the marker in the ar_sys node

  //!TODO: ROSA use a publisher or a services to tell ar_sys about the new marker.
}



void DecisionProcessNode::dynamic_reconfigure_callback(
  dynamic_marker::dynamic_param_configConfig& config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %d", config.marker_size);
  //!TODO: (raul/rosa) add the other parameters of the marker to the dynamic reconfigure cfg
  set_new_fiducial_marker(config.marker_family, config.marker_id, config.marker_size,
                          config.marker_frame_name);
}


void DecisionProcessNode::set_new_fiducial_marker(int marker_family, int marker_id, double marker_size, std::string marker_frame_name){
    set_marker_msg_.marker_family = marker_family;
    set_marker_msg_.marker_id = marker_id;
    set_marker_msg_.marker_size = marker_size;
    ROS_INFO("Sending new marker to display");
    set_marker_msg_.message_id ++; //! TODO remember to do this everytime for checking the ID
    set_marker_pub_.publish(set_marker_msg_);

    if(marker_family == 0){
      ROS_INFO("Sending new marker to Whycon detection system");
      ROS_ERROR("Alert! Not yet implemented");

    } else if (marker_family == 1 || marker_family == 2){
      ROS_INFO("Sending new marker to Aruco detection system (ar_sys)");
      ar_sys::Board_serviceRequest req;
      ar_sys::Board_serviceResponse response;
      req.marker_family = marker_family;
      req.marker_id = marker_id;
      req.marker_name = marker_frame_name;
      req.marker_size = marker_size;
      ar_sys_board_service_client_.call(req, response);
    }
  }
