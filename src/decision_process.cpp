#include "dynamic_marker/decision_process_node.hpp"
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>


int main(int argc, char* argv[]) {
  ros::init(argc, argv, "decision_process");  // Name of the node
  DecisionProcessNode Node;

   int32_t looprate = 60; //hz
   ros::Rate loop_rate(looprate);

  // ros::spin();
  while (Node.nh_.ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}
