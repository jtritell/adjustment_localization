#include <stdio.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32.h>

using namespace visualization_msgs;
using namespace std;

enum { LEFT,RIGHT };

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

geometry_msgs::Point origin,infront;

static std::string leftframe="/base_link";
static std::string rightframe="/base_link";

ros::Publisher left_pub;
ros::Publisher right_pub;

void setAdjustment(geometry_msgs::Point destination,int side=LEFT){
  InteractiveMarker int_marker;
  int_marker.header.frame_id = leftframe;
  if(side==LEFT)
    int_marker.name = "leftAdjustment";
  else
    int_marker.name = "rightAdjustment";
  int_marker.pose.position = origin;
  InteractiveMarkerControl control;
  Marker marker;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.points.push_back(origin);
  marker.points.push_back(destination);
  marker.scale.x=.1;
  marker.scale.y = .2;
  marker.scale.z = .2;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a=1.0;
  control.always_visible = true;
  control.markers.push_back(marker);
  int_marker.controls.push_back(control);
  server->insert(int_marker);
  server->applyChanges();
}

void updateArrow( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback,int side){
  setAdjustment(feedback->pose.position,side);
  if(side==LEFT)
    left_pub.publish(feedback->pose.position);
  else
    right_pub.publish(feedback->pose.position);
}

void updateLeftCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  updateArrow(feedback,LEFT);
}

void updateRightCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  updateArrow(feedback,RIGHT);
}

void adjustmentOn(int side=0){
  InteractiveMarker int_marker;
  if(side==RIGHT){
    int_marker.header.frame_id = rightframe;
    int_marker.name = "rightDestination";
  }
  else{
    int_marker.header.frame_id = leftframe;
    int_marker.name = "leftDestination";
  }
  int_marker.pose.position = infront;
  InteractiveMarkerControl control;
  Marker marker;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.scale.x=marker.scale.y=marker.scale.z=0.2;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a=0.25;
  control.markers.push_back(marker);
  control.interaction_mode =
    visualization_msgs::InteractiveMarkerControl::MOVE_3D;
  int_marker.controls.push_back(control);
  server->insert(int_marker);
  if(side==LEFT)
    server->setCallback(int_marker.name, &updateLeftCallback);
  else
    server->setCallback(int_marker.name, &updateRightCallback);
  server->applyChanges();
}

void adjustmentsCallback(const std_msgs::Int32::ConstPtr& msg)
{
  if(msg->data==0){
    server->erase("leftDestination");
    server->erase("leftAdjustment");
    server->applyChanges();
  }
  else if(msg->data==1){
    setAdjustment(infront,LEFT);
    adjustmentOn(LEFT);
  }
  
  if(msg->data==2){
    server->erase("rightDestination");
    server->erase("rightAdjustment");
    server->applyChanges();
  }
  else if(msg->data==3){
    setAdjustment(infront,RIGHT);
    adjustmentOn(RIGHT);
  }
  
}

int main(int argc, char** argv){
    origin.x=origin.y=origin.z=0;
    infront.x=infront.y=infront.z=1;
    ros::init(argc, argv, "hand_readjustment");
    server.reset( new interactive_markers::InteractiveMarkerServer("hand_readjustment") );
  ros::Duration(0.1).sleep();
    setAdjustment(infront);
    adjustmentOn();
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/adjustments",1000,adjustmentsCallback);
    left_pub =n.advertise<geometry_msgs::Point>("left_adjustment",1000);
    right_pub =n.advertise<geometry_msgs::Point>("right_adjustment",1000);
    ros::spin();
}
