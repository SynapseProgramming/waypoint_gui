
#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <iostream>
#include <math.h>
#include <fstream>
#include <vector>

#include <boost/algorithm/string.hpp>

using namespace visualization_msgs;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

std::vector<double> x_vect,y_vect,z_vect,w_vect;
std::vector<double> x_door_vect,y_door_vect,z_door_vect,w_door_vect;

void print_elements(){
  std::cout<<"x: ";
  for(unsigned int i=0; i<x_vect.size();i++){std::cout<<(x_vect[i])<<" ";}
  std::cout<<std::endl;

  std::cout<<"y: ";
  for(unsigned int i=0; i<y_vect.size();i++){std::cout<<(y_vect[i])<<" ";}
  std::cout<<std::endl;

  std::cout<<"z: ";
  for(unsigned int i=0; i<z_vect.size();i++){std::cout<<(z_vect[i])<<" ";}
  std::cout<<std::endl;

  std::cout<<"w: ";
  for(unsigned int i=0; i<w_vect.size();i++){std::cout<<(w_vect[i])<<" ";}
  std::cout<<std::endl;


  std::cout<<"dx: ";
  for(unsigned int i=0; i<x_door_vect.size();i++){std::cout<<(x_door_vect[i])<<" ";}
  std::cout<<std::endl;

  std::cout<<"dy: ";
  for(unsigned int i=0; i<y_door_vect.size();i++){std::cout<<(y_door_vect[i])<<" ";}
  std::cout<<std::endl;

  std::cout<<"dz: ";
  for(unsigned int i=0; i<z_door_vect.size();i++){std::cout<<(z_door_vect[i])<<" ";}
  std::cout<<std::endl;

  std::cout<<"dw: ";
  for(unsigned int i=0; i<w_door_vect.size();i++){std::cout<<(w_door_vect[i])<<" ";}
  std::cout<<std::endl;


}

void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  ROS_INFO_STREAM( feedback->marker_name << " is now at x: "
      << feedback->pose.position.x << ", y: " << feedback->pose.position.y
      << ", z: " << feedback->pose.position.z << "orientation: z: "<<feedback->pose.orientation.z<<", w: "<<feedback->pose.orientation.w );
//std::cout<<feedback->marker_name<<std::endl;
//over here, we would place the updated values into the arrays

std::string input(feedback->marker_name);
std::vector<std::string>result;
boost::split(result,input, boost::is_any_of(":"));
std::string goal_type=result[0];
int goal_index=(std::stoi(result[1]))-1;
if(goal_type=="Goal"){
//update the positional values at that index
x_vect[goal_index]=feedback->pose.position.x;
y_vect[goal_index]=feedback->pose.position.y;
z_vect[goal_index]=feedback->pose.orientation.z;
w_vect[goal_index]=feedback->pose.orientation.w;
}
else if(goal_type=="Door_Goal"){
  //update the positional values at that index
  x_door_vect[goal_index]=feedback->pose.position.x;
  y_door_vect[goal_index]=feedback->pose.position.y;
  z_door_vect[goal_index]=feedback->pose.orientation.z;
  w_door_vect[goal_index]=feedback->pose.orientation.w;
}

print_elements();

}//bracket of process feedback

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


  place_phantom_marker(1,0,1,0,1,1);//goal (1)
  //add in the marker into the vectors
  x_vect.push_back(1);
  y_vect.push_back(0);
  z_vect.push_back(1);
  w_vect.push_back(0);

  place_phantom_marker(3,2,1,0,2,1);//goal (2)
  //add in the marker into the vectors
  x_vect.push_back(3);
  y_vect.push_back(2);
  z_vect.push_back(1);
  w_vect.push_back(0);

  place_phantom_marker(2,0,1,0,1,2);//door goal (1)
  x_door_vect.push_back(2);
  y_door_vect.push_back(0);
  z_door_vect.push_back(1);
  w_door_vect.push_back(0);

  place_phantom_marker(2,2,1,0,2,2);//door goal (2)
  x_door_vect.push_back(2);
  y_door_vect.push_back(2);
  z_door_vect.push_back(1);
  w_door_vect.push_back(0);



  // start the ROS main loop
  ros::spin();


}
// %Tag(fullSource)%
