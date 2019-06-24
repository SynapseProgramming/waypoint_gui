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
//function prototype
void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

//global variables here!
std::ofstream outfile;
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
std::string file_path;
//create vectors to store waypoint info from text file
std::vector<double> x_vect,y_vect,z_vect,w_vect;
std::vector<double> x_door_vect,y_door_vect,z_door_vect,w_door_vect;

//print elements function mainly used for debugging
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


void print_array(std::string array_head, std::vector<double> &array){
  outfile<<array_head<< ": [";
  for(unsigned int i=0; i<array.size();i++){
  //if i is at the last element of the vector
  if(i==(array.size()-1)){outfile<<array[i]<<"]\n";}
  else{outfile<<array[i]<<",";}
  }
}

//place_phantom_marker(double px,double py,double qz,double qw,int index,int title)
void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
//declare static variables
static double x,y,z,w;

switch(feedback->event_type){
case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
{
ROS_INFO_STREAM( feedback->marker_name << " is now at x: "
    << feedback->pose.position.x << ", y: " << feedback->pose.position.y
    << ", z: " << feedback->pose.position.z << "orientation: z: "<<feedback->pose.orientation.z<<", w: "<<feedback->pose.orientation.w );
//update variables
x=feedback->pose.position.x;
y=feedback->pose.position.y;
z=feedback->pose.orientation.z;
w=feedback->pose.orientation.w;

if((feedback->marker_name)!="waypoint"){
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
}
}
break;

case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
  ROS_INFO_STREAM(feedback->marker_name<< ": menu item " << feedback->menu_entry_id);
  //std::cout<<feedback->menu_entry_id<<std::endl;
  if(feedback->menu_entry_id==1){// if Set as Goal is selected, save goals
  x_vect.push_back(x);
  y_vect.push_back(y);
  z_vect.push_back(z);
  w_vect.push_back(w);
  int goal_vect_size=x_vect.size();
  place_phantom_marker(x,y,z,w,goal_vect_size,1);
  ROS_WARN("GOAL AT THIS LOCATION HAS BEEN SAVED. NET GOALS: %d",goal_vect_size);
  }
  else if(feedback->menu_entry_id==2){// if Set as door Goal is selected, save door goals
    x_door_vect.push_back(x);
    y_door_vect.push_back(y);
    z_door_vect.push_back(z);
    w_door_vect.push_back(w);
    int doorgoal_vect_size=x_door_vect.size();
  place_phantom_marker(x,y,z,w,doorgoal_vect_size,2);
  ROS_WARN("DOOR GOAL AT THIS LOCATION HAS BEEN SAVED. NET DOOR GOALS: %d",doorgoal_vect_size);
  }
  else if(feedback->menu_entry_id==3){//if Save all Goals is selected, save all goals into text file
  outfile.open(file_path);
  if(outfile.is_open()){
  print_array("x",x_vect);
  print_array("y",y_vect);
  print_array("z",z_vect);
  print_array("w",w_vect);

  print_array("dx",x_door_vect);
  print_array("dy",y_door_vect);
  print_array("dz",z_door_vect);
  print_array("dw",w_door_vect);
  ROS_WARN("All locations have been saved!");
  outfile.close();
}
else{
  ROS_FATAL("Could not find waypoints file. Please check file path!\n");
  //kill the node, since we could not load the file(critical)
  ros::shutdown();
}
  }
  break;


}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_marker");
  ros::NodeHandle n;

  // create an interactive marker server on the topic namespace simple_marker
  server.reset( new interactive_markers::InteractiveMarkerServer("simple_marker","",false) );
 //create an instance of the menu handler object
  interactive_markers::MenuHandler menu_handler;
  //create menu windows
  menu_handler.insert( "Set as Goal", &processFeedback );
  menu_handler.insert( "Set as Door Goal", &processFeedback );
  menu_handler.insert( "Save all Goals", &processFeedback );

  //obtain the filepath of waypoints file
  n.getParam("file_path",file_path);

  //read in the params
  n.getParam("x",x_vect);
  n.getParam("y",y_vect);
  n.getParam("z",z_vect);
  n.getParam("w",w_vect);
  n.getParam("dx",x_door_vect);
  n.getParam("dy",y_door_vect);
  n.getParam("dz",z_door_vect);
  n.getParam("dw",w_door_vect);
 //plot out the existing waypoints
  if(x_vect.size()!=0){
    for(unsigned int i=0; i<x_vect.size();i++){place_phantom_marker(x_vect[i],y_vect[i],z_vect[i],w_vect[i],(i+1),1);}
    //we would want to plot out the door goal waypoints
    for(unsigned int j=0; j<x_door_vect.size();j++){place_phantom_marker(x_door_vect[j],y_door_vect[j],z_door_vect[j],w_door_vect[j],(j+1),2);}
  }

  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "map";
  int_marker.header.stamp=ros::Time::now();
  int_marker.name = "waypoint";
  int_marker.description = "Waypoint Generator";

// just for visualization: box_marker->box_control->int_marker
//for actual 1 dof control: main_control->int_marker
  // create a grey box marker just for visualization
  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::ARROW;
  box_marker.scale.x = 0.45;
  box_marker.scale.y = 0.2;
  box_marker.scale.z = 0.2;
  box_marker.color.r = 0.5;
  box_marker.color.g = 0.5;
  box_marker.color.b = 0.5;
  box_marker.color.a = 1.0;

  //create a control for the menu system
  visualization_msgs::InteractiveMarkerControl box_control;
  //declare that the control is a menu kind
  box_control.interaction_mode=InteractiveMarkerControl::MENU;
  box_control.name="menu control";
  box_control.always_visible = true;
  //place the grey box marker visualization into the box_control.
  box_control.markers.push_back( box_marker );

  // add the control to the interactive marker
  int_marker.controls.push_back( box_control );

  // create a control which will move the box
  // this control does not contain any markers,
  // which will cause RViz to insert two arrows
  //ensure that rotation about z axis is independent the xy translation
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
  //copy current menu state into the marker
  menu_handler.apply(*server,int_marker.name);

  // 'commit' changes and send to all clients
  server->applyChanges();

  // start the ROS main loop
  ros::spin();
}
// %Tag(fullSource)%
