#include <stdio.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Int32.h>

using namespace visualization_msgs;
using namespace std;

enum { LEFT,RIGHT };

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

geometry_msgs::Point origin,infront;

ros::Publisher tilt_pub;

static std::string leftpalm="/Body_LWP";
static std::string rightpalm="/Body_RWP";
static std::string leftfoot="/Body_LAP";
static std::string rightfoot="/Body_RAP";

static tf::Vector3 offset_lefthand=tf::Vector3(0.00,0.067,0);
static tf::Quaternion rotation_lefthand = 
  tf::Quaternion(0.0,0.707106,0.0,0.707106);
static tf::Vector3 offset_righthand=tf::Vector3(0.00,-0.067,0);
static tf::Quaternion rotation_righthand = 
  tf::Quaternion(0.0,0.707106,0.0,0.707106);
static tf::Vector3 offset_leftfoot=tf::Vector3(0.07,0,0);
static tf::Quaternion rotation_leftfoot = 
  tf::Quaternion(0.0,0.0,0.0,1.0);
static tf::Vector3 offset_rightfoot=tf::Vector3(0.07,0,0);
static tf::Quaternion rotation_rightfoot = 
  tf::Quaternion(0.0,0.0,0.0,1.0);

const std::string directory = "package://drchubo-v3/meshes/";

std::string frame;
tf::Vector3 offset;
tf::Quaternion rotation;

ros::Publisher adjustment_pub;
int frameid;


void updateRPY(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
  tilt_pub.publish(feedback->pose.orientation);
}

void makeTranspose(){
  InteractiveMarker int_marker;
  InteractiveMarkerControl control;
  int_marker.header.frame_id = "adjustment";
  int_marker.name = "TransposeMarker";
  int_marker.pose.position = origin;
  Marker marker;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  std::string stl;
  if(frameid==1)
    stl="convhull_LWP_merged.stl";
  if(frameid==2)
    stl="convhull_RWP_merged.stl";
  if(frameid==3)
    stl="convhull_LAR_merged.stl";
  if(frameid==4)
    stl="convhull_RAR_merged.stl";
  std::string filename=directory+stl;
  marker.mesh_resource = filename;
  marker.color.r = 0.8;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a=0.5;
  control.always_visible = true;
  control.markers.push_back(marker);
  int_marker.controls.push_back(control);

  InteractiveMarkerControl rotate_control;
  
  rotate_control.orientation.w = 1;
  rotate_control.orientation.x = 0;
  rotate_control.orientation.y = 1;
  rotate_control.orientation.z = 0;
  rotate_control.name = "rotate_z";
  int_marker.scale = 0.5;
  control.orientation_mode = InteractiveMarkerControl::FIXED;
  rotate_control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(rotate_control);

  rotate_control.orientation.w = 1;
  rotate_control.orientation.x = 0;
  rotate_control.orientation.y = 0;
  rotate_control.orientation.z = 1;
  rotate_control.name = "rotate_y";
  rotate_control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  control.orientation_mode = InteractiveMarkerControl::FIXED;
  int_marker.controls.push_back(rotate_control);

    rotate_control.orientation.w = 1;
    rotate_control.orientation.x = 1;
    rotate_control.orientation.y = 0;
    rotate_control.orientation.z = 0;
    rotate_control.name = "rotate_x";
    rotate_control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    control.orientation_mode = InteractiveMarkerControl::FIXED;
    int_marker.controls.push_back(rotate_control);
  server->insert(int_marker);
  server->setCallback(int_marker.name, &updateRPY, visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE );
  server->applyChanges();
}

void setAdjustment(geometry_msgs::Point::ConstPtr msg){
  static tf::TransformBroadcaster br;
  static tf::TransformListener listener;
  tf::StampedTransform transform;
  try{
    listener.lookupTransform("/Body_LAP",frame,ros::Time(0),transform);
  }
  catch(tf::TransformException ex){
  }
  tf::Transform t;
  ros::Time time = ros::Time::now();
  tf::Vector3 x(msg->x,msg->y,msg->z);
  t.setOrigin(transform.getOrigin()+x+offset);
  t.setRotation(transform.getRotation());
  //  t.setRotation(transform.getRotation().inverse());
//  t.setRotation(rotation);
  br.sendTransform(tf::StampedTransform(t,time, "Body_LAP", "adjustment"));
}

void setLink(std_msgs::Int32::ConstPtr msg){
  frameid=msg->data;
  if(msg->data==0){
    server->erase("TransposeMarker");
    server->erase("AdjustmentMarker");
  }
  else if(msg->data==1){
    frame=leftpalm;
    offset = offset_lefthand;
    rotation = rotation_lefthand;
  }
  else if(msg->data==2){
    frame=rightpalm;
    offset = offset_righthand;
    rotation = rotation_righthand;
  }
  else if(msg->data==3){
    frame=leftfoot;
    offset = offset_leftfoot;
    rotation = rotation_leftfoot;
  }
  else if(msg->data==4){
    frame=rightfoot;
    offset = offset_rightfoot;
    rotation = rotation_rightfoot;
  }
  makeTranspose();
}

void setTilt(geometry_msgs::Quaternion::ConstPtr msg){
  geometry_msgs::Pose pose;
  pose.position.x=pose.position.y=pose.position.z=0;
  pose.orientation = *msg;
  server->setPose("TransposeMarker",pose);
  server->applyChanges();
}



/*
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
  marker.scale.x=marker.scale.y=marker.scale.z=0.05;
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
*/
int main(int argc, char** argv){
    origin.x=origin.y=origin.z=0;
    infront.x=infront.y=infront.z=1;
    ros::init(argc, argv, "hand_readjustment");
    server.reset( new interactive_markers::InteractiveMarkerServer("hand_readjustment") );
    ros::Duration(0.1).sleep();
    //adjustmentOn();
    ros::NodeHandle n;
    ros::Subscriber sub1 = n.subscribe("/adjustment/link",1000,setLink);
    ros::Subscriber sub2 = n.subscribe("/adjustment/point",1000,setAdjustment);
    ros::Subscriber sub3 = n.subscribe("/adjustment/tilt",1000,setTilt);
    tilt_pub =n.advertise<geometry_msgs::Quaternion>("adjustment/tilt2",1000);
//    adjustment_pub =n.advertise<geometry_msgs::Point>("adjustment/point",1000);
    //    right_pub =n.advertise<geometry_msgs::Point>("right_adjustment",1000);
    ros::spin();
}
