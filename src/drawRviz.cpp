#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <landing_vision/markers.h>
#include <landing_vision/marker4c.h>

using namespace std;

static const string msgTopic = "pose_estimation/camera_coordinate";

landing_vision::markers markerPoints;
void marker_points_callback(const landing_vision::markers::ConstPtr& msg){
  markerPoints = *msg;
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "draw_in_rviz");
  ros::NodeHandle n;
  ros::Rate rate(30);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber marker_sub = n.subscribe<landing_vision::markers>(msgTopic, 1, marker_points_callback);

  while (ros::ok())
  {
    visualization_msgs::Marker centers,cnrs, redpoint;
    centers.header.frame_id	= cnrs.header.frame_id		=redpoint.header.frame_id	= "/map";
    centers.header.stamp	= cnrs.header.stamp		=redpoint.header.stamp		= ros::Time::now();
    centers.ns			= cnrs.ns			=redpoint.ns			= "marker_points";
    centers.action		= cnrs.action			=redpoint.action		= visualization_msgs::Marker::ADD;
    centers.pose.orientation.w	= cnrs.pose.orientation.w	=redpoint.pose.orientation.w	= 1.0;
    
    centers.id = 0;
    centers.type = visualization_msgs::Marker::POINTS;
    centers.scale.x = 0.2;
    centers.scale.y = 0.2;
    centers.color.r = 1.0f;
    centers.color.g = 1.0f;
    centers.color.b = 1.0f;
    centers.color.a = 1.0;

    cnrs.id = 1;
    cnrs.type = visualization_msgs::Marker::LINE_STRIP;
    cnrs.scale.x = 0.1;
    cnrs.scale.y = 0.1;
    cnrs.color.g = 1.0f;
    cnrs.color.a = 1.0;
    
    redpoint.id = 2;
    redpoint.type = visualization_msgs::Marker::POINTS;
    redpoint.scale.x = 0.2;
    redpoint.scale.y = 0.2;
    redpoint.color.r = 1.0f;
    redpoint.color.a = 1.0;
    
    for(int i=0;i<markerPoints.markers.size();i++){
      landing_vision::marker4c mk4c = markerPoints.markers[i];
      if(mk4c.ID != 0){
	geometry_msgs::Point p;
	p.x = mk4c.center.x*10;
	p.y = mk4c.center.y*10;
	p.z = mk4c.center.z*10;
// 	ROS_INFO("(%.2f, %.2f, %.2f)", p.x, p.y, p.z);
	centers.points.push_back(p);
	for(int c=0;c<mk4c.points.size();c++){
	  p.x = mk4c.points[c].x*10;
	  p.y = mk4c.points[c].y*10;
	  p.z = mk4c.points[c].z*10;
	  cnrs.points.push_back(p);
	}
	p.x = mk4c.points[0].x*10;
	p.y = mk4c.points[0].y*10;
	p.z = mk4c.points[0].z*10;
// 	redpoint.points.push_back(p);
	centers.points.push_back(p);
	cnrs.points.push_back(p);
      }
    }

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      ros::Duration(1.0).sleep();
    }
    marker_pub.publish(centers);
    marker_pub.publish(cnrs);
//     marker_pub.publish(redpoint);

    rate.sleep();
    ros::spinOnce();
  }
}
