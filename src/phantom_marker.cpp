
#include <ros/ros.h>
#include <iostream>
#include <interactive_markers/interactive_marker_server.h>

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;



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
  //update the server
  server->insert(int_marker);

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_marker");
  double x=0,y=0,z=0,w=0;
  // create an interactive marker server on the topic namespace simple_marker
  server.reset( new interactive_markers::InteractiveMarkerServer("simple_marker","",false) );
  int count=0;
  int title=0;
  while(ros::ok()){



  std::cout<<"Please enter the x,y,z,w position values!"<<std::endl;
  std::cin>>x>>y>>z>>w;
  std::cout<<"please enter 1. for the (goal), and enter 2 for (door_goal)"<<std::endl;
  std::cin>>title;
  place_phantom_marker(x,y,z,w,count,title);

  count++;

  // 'commit' changes and send to all clients
  server->applyChanges();

  // start the ROS main loop
  ros::spinOnce();

  }
}
// %Tag(fullSource)%
