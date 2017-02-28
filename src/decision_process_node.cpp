#include "dynamic_marker/decision_process_node.hpp"

DecisionProcessNode::DecisionProcessNode() {
  ros::NodeHandle params("~");
  // Topic Parameters
  std::string s;

  // Subscribed topics
  params.param<std::string>("oberver_distance_topic", s, "/ardrone/navdata_altitude");
  observer_distance_sub_ = nh_.subscribe(s, 1, &DecisionProcessNode::quadcopter_altitude_cb, this,
                                ros::TransportHints().tcpNoDelay());


  params.param<std::string>("ar_sys_marker_transform_topic", s, "/ar_multi_boards/transform");
  ar_sys_marker_transform_sub_ = nh_.subscribe(s, 1, &DecisionProcessNode::ar_sys_marker_transform_cb, this,
                                ros::TransportHints().tcpNoDelay());

  params.param<std::string>("whycon_marker_pose_topic", s, "/whycon/poses");
  whycon_marker_pose_sub_ = nh_.subscribe(s, 1, &DecisionProcessNode::whycon_marker_pose_cb, this,
                                ros::TransportHints().tcpNoDelay());


  params.param<std::string>("marker_status_topic", s, "/dynamic_marker/set_marker_response");
  set_marker_response_sub_ = nh_.subscribe(s, 1, &DecisionProcessNode::set_marker_response_cb, this,
                                   ros::TransportHints().tcpNoDelay());

  // Published topics
  params.param<std::string>("set_marker_topic", s, "/dynamic_marker/set_marker");
  set_marker_pub_ = nh_.advertise<dynamic_marker::set_marker>(s, 1);

  params.param<std::string>("output_pose_topic", s, "/dynamic_marker/pose");
  output_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(s, 1);

  params.param<std::string>("output_tf_topic", s, "/dynamic_marker/transform");
  output_transform_pub_ = nh_.advertise<geometry_msgs::TransformStamped>(s, 1);


  experimental_transform_pub_ = nh_.advertise<geometry_msgs::TransformStamped>("/dynamic_marker/experimental_transform", 1);

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
  current_marker_detection_is_enabled_ = true;
  marker_msg_to_display_.message_id = 0;

  delay_image_capture_ = ros::Duration(0.270);
  //! TODO make camera fps as a parameter
  camera_fps_ = 20.0;
  timer_skip_one_frame_  = nh_.createTimer(ros::Duration(1/camera_fps_), &DecisionProcessNode::skip_one_frame_timer_cb, this, true);
  timer_set_marker_ = nh_.createTimer(delay_image_capture_, &DecisionProcessNode::timer_set_marker_cb, this, true);
  set_marker_cycle_ended_ = true;


  // Testing camera fov related functions, TODO erase
  double fovx, fovy, marker_ideal_size;

  calc_cam_fov_(368.167328, 443.671722, 720, 576, fovx, fovy);

  calc_max_tilt_(fovx, 0.16, 0.5);

  calc_ideal_marker_size_(fovx, 0.1, 0.25, 0.10, marker_ideal_size);




}


void DecisionProcessNode::set_marker_response_cb(const dynamic_marker::set_marker_response marker_response_msg){
  ros::Duration dt;
  dt = ros::Time::now() -set_new_marker_instant_;
  ROS_INFO("Received response from dynamic display");
  if (marker_response_msg.result && marker_response_msg.message_id == marker_msg_to_display_.message_id){
    //The marker was succesfully updated on the screen
    display_marker_updated_ = true;
    ROS_INFO("Marker is updated on the display, delay: %f", dt.toSec());

  } else{
    //!TODO do something really smart in case that fails
    ROS_ERROR("display screen couldnt update the marker with the current configuration:");
    ROS_ERROR("dynamic_marker_screen response: %s",marker_response_msg.message.c_str());
  }
}

void DecisionProcessNode::ar_sys_marker_transform_cb(const geometry_msgs::TransformStamped ar_sys_marker_transform_msg){
  geometry_msgs::TransformStamped tf_msg_out;
  tf_msg_out = ar_sys_marker_transform_msg;
  // we check first that we are indeed tracking an ar_sys marker
  if (current_marker_family_ == aruco_single ||
      current_marker_family_ == aruco_multi){

    // We check here that the current marker is being displayed on the screen
    // considering the capture delay

    if(display_marker_updated_ && current_marker_detection_is_enabled_){
      tf_msg_out.header.frame_id = "cam";
      tf_msg_out.child_frame_id = current_marker_frame_name_;


      //!TODO remove the following, only valid for experiment 2

      float scale_change = current_marker_size_/0.12; //default marker size




      tf_msg_out.transform.translation.x = tf_msg_out.transform.translation.x*scale_change;
      tf_msg_out.transform.translation.y = tf_msg_out.transform.translation.y*scale_change;
      tf2_broadcaster_.sendTransform(tf_msg_out);
      //publish the transform to the output topic
      output_transform_pub_.publish(tf_msg_out);

      tf_msg_out.transform.translation.z = tf_msg_out.transform.translation.z *scale_change;
      experimental_transform_pub_.publish(tf_msg_out);

       //!END OF REMOVE




      marker_control(tf_msg_out.transform.translation.z);
    } else{
      ROS_INFO("Marker is not yet stable!");
    }
  }




}

void DecisionProcessNode::whycon_marker_pose_cb(const geometry_msgs::PoseArray marker_pose_array_msg){
  // we check first that we are indeed tracking a Whycon marker
  if (current_marker_family_ != whycon){
    return;
  }

  // We check here that the current marker is being displayed on the screen
  // considering the capture delay

  if(display_marker_updated_ && current_marker_detection_is_enabled_){
    geometry_msgs::Pose marker_pose_msg = marker_pose_array_msg.poses[0];
    geometry_msgs::TransformStamped tf_msg_out;
    tf2::Transform transform_in;
    tf2::Transform transform_helper;

    // Lets convert our pose message into a transform
    tf2::fromMsg(marker_pose_msg, transform_in);

    // We need to adjust Whycon to the same coordinate system of ar_sys
    tf2::Quaternion q;
    transform_helper.setIdentity();

    // First we get rid of the orientation obtained from Whycon (we only use 3D positions)
    // since it is too hard to control due to the symetry of the marker
    q.setRPY(0, 0, 0);
    transform_in.setRotation(q);

    // We now flip the frame so Z axis is looking up from the marker plane.
    q.setRPY((180)*M_PI/180, 0, 0);
    transform_helper.setRotation(q);
    transform_in = transform_in*transform_helper;

    // Finally we rotate around Z for a convenient alignment with the quadcopter
    // forward direction of movement, this particular of each config.
    q.setRPY(0, 0, (90)*M_PI/180);
    transform_helper.setRotation(q);
    transform_in = transform_in*transform_helper;

    // we create a transform from the pose
    tf_msg_out.header = marker_pose_array_msg.header;
    //!TODO fix this in define_target
    //! // WE need to publish the transform UGLY HACK
    tf_msg_out.header.frame_id = "cam";
    tf_msg_out.child_frame_id = current_marker_frame_name_;
    tf_msg_out.transform = tf2::toMsg(transform_in);
    tf2_broadcaster_.sendTransform(tf_msg_out);

    //publish the transform to the output topic
    output_transform_pub_.publish(tf_msg_out);
    marker_control(tf_msg_out.transform.translation.z);
  } else{
    ROS_INFO("Marker is not yet stable!");
  }
}



void DecisionProcessNode::quadcopter_altitude_cb(const ardrone_autonomy::navdata_altitude altitude_msg){
  return;
  int altitude =  altitude_msg.altitude_raw; // in millimeters
  int th_whycon = 3000; // whycon threshold in millimiters
  int th_aruco_single = 500; // aruco_single board threshold in millimiters
  int marker_size; //side lenght (aruco) or diameter (whycon) in millimiters


}


void DecisionProcessNode::marker_control(const double marker_cam_distance){
  return;
  int marker_family, marker_id;
  double marker_size;


  // Decision process (select marker size and type)

  if (marker_cam_distance >= threshold_aruco_single_){
    // We use whycon
    // only one size
    marker_family = whycon;
    marker_id = 0; //default?
    marker_size = max_marker_size_;

  } else if (marker_cam_distance < threshold_aruco_single_){
    // We use aruco multiboard single ID 88
    // Size changes dynamically
    double fovx, fovy, marker_ideal_size = 0.0;
    //!TODO This values are dependant on the camera calibration file. Change!
    calc_cam_fov_(368.167328, 443.671722, 720, 576, fovx, fovy);
    calc_ideal_marker_size_(fovy, 0.2, marker_cam_distance, 0.00, marker_ideal_size);

    marker_family = aruco_multi;
    marker_id = 88;
    marker_size = std::min(marker_ideal_size, max_marker_size_);
    marker_size = std::max(marker_size,0.02);



  }
//  else if (marker_cam_distance > 0.0001 && marker_cam_distance <= threshold_aruco_multi_ ){
//    // We use aruco multi
//    // Size changes dynamically
//    marker_family = aruco_multi;
//    marker_id = 0; //default?
//    marker_size = 0.05; //! TODO: changes linearly with distance.. we have to develop this equation

//  }
  else{
    // We use whycon
    // only one size
    marker_family = whycon;
    marker_id = 0; //default?
    marker_size = max_marker_size_;
  }

  set_new_fiducial_marker(marker_family, marker_id, marker_size,"land_target");
}

void DecisionProcessNode::dynamic_reconfigure_callback(
  dynamic_marker::dynamic_param_configConfig& config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %d", config.marker_size);

  set_new_fiducial_marker(config.marker_family, config.marker_id, config.marker_size,
                          config.marker_frame_name);

  delay_image_capture_ = ros::Duration(config.capture_delay);
  marker_fov_ = config.marker_fov;
  threshold_aruco_single_ = config.threshold_aruco_single;
  threshold_aruco_multi_ = config.threshold_aruco_multi;
  max_marker_size_ = config.max_marker_size;
}


void DecisionProcessNode::set_new_fiducial_marker(int marker_family, int marker_id, double marker_size, std::string marker_frame_name){



  if(set_marker_cycle_ended_ == false){
    ROS_DEBUG("Cannot change the  marker, there is still an Update being performed");
    return;
  }
  if (marker_family == last_marker_family_ && marker_id == last_marker_id_ && marker_size == last_marker_size_){
    ROS_DEBUG("Marker is already displayed, nothing to do.");
    return;
  }


  set_new_marker_instant_ = ros::Time::now();

  set_marker_cycle_ended_ = false;
  display_marker_updated_ = false;

  last_marker_family_= marker_family;
  last_marker_id_ = marker_id;
  last_marker_size_ = marker_size;


  marker_msg_to_display_.marker_family = marker_family;
  marker_msg_to_display_.marker_id = marker_id;
  marker_msg_to_display_.marker_size = marker_size;
  if (marker_family == whycon){
    //Whycon doesnt allow dynamic changes
    marker_msg_to_display_.marker_size = max_marker_size_;
  }
  marker_msg_to_display_.marker_name = marker_frame_name;
  ROS_DEBUG("Marker size sent: %f", marker_size);

  ROS_INFO("Sending new marker to display, size: %f", marker_msg_to_display_.marker_size);
  marker_msg_to_display_.message_id ++; //! TODO remember to do this everytime for checking the ID
  set_marker_pub_.publish(marker_msg_to_display_);

  timer_set_marker_.stop();
  timer_set_marker_.setPeriod(delay_image_capture_);
  timer_set_marker_.start();
  }


void DecisionProcessNode::timer_set_marker_cb(const ros::TimerEvent&){
  current_marker_detection_is_enabled_ = false;

  current_marker_family_ = marker_msg_to_display_.marker_family;
  current_marker_id_ = marker_msg_to_display_.marker_id;
  current_marker_size_ = marker_msg_to_display_.marker_size;
  current_marker_frame_name_ = marker_msg_to_display_.marker_name;

  if(marker_msg_to_display_.marker_family == 0){
    ROS_INFO("Sending new marker to Whycon detection system");

  } else if (marker_msg_to_display_.marker_family  == 1 || marker_msg_to_display_.marker_family  == 2){
    ar_sys::Board_serviceRequest req;
    ar_sys::Board_serviceResponse response;

    req.marker_family = current_marker_family_;
    req.marker_id = current_marker_id_;
    req.marker_name = current_marker_frame_name_;

    //! ALERT REMOVE THIS ONLY FOR EXPERIMENT 2
    //! req.marker_size = marker_msg_to_display_.marker_size;
    req.marker_size = 0.12;
    //! END OF REMOVE
    ROS_INFO("Sending new marker to Aruco detection system (ar_sys), size: %f", req.marker_size);
    ROS_INFO("Setting the timer to wait for camera delay");
    ar_sys_board_service_client_.call(req, response);

    if (response.result){

    } else {
      ROS_ERROR("Ar_Sys service response: FALSE. Cant change the marker.");
    }
  }
  //we force to skip one frame just to be sure.
  timer_skip_one_frame_.stop();
  timer_skip_one_frame_.start();
}


void DecisionProcessNode::skip_one_frame_timer_cb(const ros::TimerEvent&){
  if(!display_marker_updated_){
    //we force to skip one frame just to be sure.
    timer_skip_one_frame_.stop();
    timer_skip_one_frame_.start();
  }
  ROS_INFO("The new marker is updated in display and ready to be detected");
  current_marker_detection_is_enabled_ = true;
  set_marker_cycle_ended_ = true;
}


void DecisionProcessNode::calc_cam_fov_(double fx, double fy, int img_width, int img_height, double& fovx, double& fovy){
  //Taken from calib3D in Opencv
  /* Calculate fovx and fovy. */
  fovx = 2 * atan(img_width / (2 * fx)) ;
  fovy = 2 * atan(img_height / (2 * fy));
  ROS_DEBUG("The camera field of view is: (fovx = %f, fovy = %f)", fovx* 180.0 / M_PI, fovy* 180.0 / M_PI);
}


void DecisionProcessNode::calc_max_tilt_(double fovx, double marker_size, double marker_cam_distance){
  double max_tilt = (fovx)/2.0f - atan(marker_size/(2*marker_cam_distance));
  ROS_DEBUG("The max tilting angle for a marker of %f m at %f m of distance  is: %f", marker_size, marker_cam_distance, max_tilt* 180.0 / M_PI);
}


void DecisionProcessNode::calc_ideal_marker_size_(double fovx, double max_quad_tilt, double marker_cam_distance, double tracking_error, double &marker_ideal_size){
  marker_ideal_size = 2*marker_cam_distance*tan(fovx/2-2*max_quad_tilt) - 2*tracking_error;
  marker_ideal_size = 2*marker_cam_distance*tan(fovx*marker_fov_);

  ROS_DEBUG("The ideal marker size is for a distance of %f meters: %f", marker_cam_distance, marker_ideal_size);
}
