#include <ros/ros.h>

#include <landing_vision/markers.h>
#include <landing_vision/marker4c.h>

using namespace std;

static const std::string msgTopic = "pose_estimation/camera_coordinate";

landing_vision::markers mkData;
void cam_pose_callback(const landing_vision::markers::ConstPtr& msg){
  mkData = *msg;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "topicRcv");
  ros::NodeHandle nh_;
  ros::Subscriber sub_cam_pose = nh_.subscribe<landing_vision::markers>(msgTopic, 1, cam_pose_callback);
  cout << "topicRcv started\n";
  ros::Rate rate(30.0);
  uint32_t seq = 0;
  while (ros::ok()){
    ros::spinOnce();
    if (mkData.header.seq != seq){
      for (int i=0;i<mkData.markers.size();i++){
	landing_vision::marker4c mk4c = mkData.markers[i];
	if (mk4c.ID != 0){
	  cout << "ID:" << unsigned(mk4c.ID) << endl;
	  double x,y,z;
	  x = mk4c.center.x;
	  y = mk4c.center.y;
	  z = mk4c.center.z;
	  
	  for (int j=0;j<mk4c.points.size();j++){
	    cout  << "[" << mk4c.points[j].x-x << ", " << mk4c.points[j].y-y << ", " << mk4c.points[j].z-z << "]\n";
	  }
	}
      }
      cout << "------\n";
      seq = mkData.header.seq;
    }
  }
}