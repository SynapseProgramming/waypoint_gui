
#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <iostream>
#include <math.h>
#include <fstream>
#include <vector>

using namespace visualization_msgs;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  ROS_INFO_STREAM( feedback->marker_name << " is now at x: "
      << feedback->pose.position.x << ", y: " << feedback->pose.position.y
      << ", z: " << feedback->pose.position.z << "orientation: z: "<<feedback->pose.orientation.z<<", w: "<<feedback->pose.orientation.w );
}

void place_phantom_marker(double px,double py,double qz,double qw,int index,int title){
  // create an interactive marker for our server
  std::string description;
  if(title==1){description="Goal:"+std::to_string(index);}
  else if(title==2){description="Door_Goal:"+std::to_string(index);}

  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "map";
  int_marker.header.stamp=ros::Time::now();
  int_marker.name = description;
  int_marker.description = description;

 //set the position of the interactive marker
 int_marker.pose.position.x=px;
 int_marker.pose.position.y=py;
 int_marker.pose.position.z=0.0;
 int_marker.pose.orientation.x=0.0;
 int_marker.pose.orientation.y=0.0;
 int_marker.pose.orientation.z=qz;
 int_marker.pose.orientation.w=qw;

//create a arrow marker
  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::ARROW;
  box_marker.scale.x = 0.45;
  box_marker.scale.y = 0.2;
  box_marker.scale.z = 0.2;
  box_marker.color.r = 0.0f;
  box_marker.color.g = 1.0f;
  box_marker.color.b = 0.0f;
  box_marker.color.a = 1.0;

  // create a non-interactive control which contains the arrow
  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  //place the grey box marker visualization into the non interactive control.
  box_control.markers.push_back( box_marker );

  // add the control to the interactive marker
  int_marker.controls.push_back( box_control );

  visualization_msgs::InteractiveMarkerControl main_control;

  main_control.orientation_mode = InteractiveMarkerControl::FIXED;

 //set x controls
  tf::Quaternion orien(1.0, 0.0, 0.0, 1.0);
  orien.normalize();
  tf::quaternionTFToMsg(orien, main_control.orientation);
  main_control.name = "move_x";
  main_control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(main_control);

  //set y controls
   orien = tf::Quaternion(0.0, 0.0, 1.0, 1.0);
   orien.normalize();
   tf::quaternionTFToMsg(orien, main_control.orientation);
   main_control.name = "move_y";
   main_control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
   int_marker.controls.push_back(main_control);

   //set rotation about z axis
   orien = tf::Quaternion(0.0, 1.0, 0.0, 1.0);
   orien.normalize();
   tf::quaternionTFToMsg(orien, main_control.orientation);
   main_control.name = "rotate_z";
   main_control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
   int_marker.controls.push_back(main_control);

  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  server->insert(int_marker, &processFeedback);
  server->applyChanges();

}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_marker");

  // create an interactive marker server on the topic namespace simple_marker
  server.reset( new interactive_markers::InteractiveMarkerServer("simple_marker","",false) );


  place_phantom_marker(1,0,1,0,1,1);
  place_phantom_marker(2,0,1,0,2,2);




  // start the ROS main loop
  ros::spin();


}
// %Tag(fullSource)%
