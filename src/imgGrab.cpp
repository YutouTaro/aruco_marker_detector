#include <string>
#include <map>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <landing_vision/markers.h>
#include <std_msgs/Header.h>
#include <landing_vision/marker4c.h>
#include <geometry_msgs/Point.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/sfm.hpp>

// #include <Eigen/Core>
// #include <Eigen/Geometry>
// #include "sophus/so3.h"

using namespace std;

static const std::string OPENCV_WINDOW = "Image window";
std::string msgTopic;
// const std::string srvTopic = "/image_converter/output_video";
const std::string srvTopic_marker = "pose_estimation/camera_coordinate";

string filePath;
cv::Mat mtx, dist, mtxInv;
double markerLength, imgScale;
int rateLost0, rateTracking1;
bool isLoaded, reintialize = false;

class MarkerPose3D{
public:
  int _id;
  vector<cv::Point2f> _cnrs2D;
//   vector<cv::Point3f> _corners3D;
  landing_vision::marker4c _marker;
  cv::Vec3d _t0, _dt;
  cv::Mat _R0, _R0inv, _dR, _tf;
  vector<cv::Point3f> markerPts3D;
//   markerPts3D.push_back(cv::Point3f(-markerLength/2.0,  markerLength/2.0, 0.0));
//   markerPts3D.push_back(cv::Point3f(-markerLength/2.0, -markerLength/2.0, 0.0));
//   markerPts3D.push_back(cv::Point3f( markerLength/2.0, -markerLength/2.0, 0.0));
//   markerPts3D.push_back(cv::Point3f( markerLength/2.0,  markerLength/2.0, 0.0));
public:
  MarkerPose3D(){}
  
  MarkerPose3D(int ID, vector<cv::Point2f> cnrs):_id(ID), _cnrs2D(cnrs){
//     vector<cv::Point3f> markerPts3D;
    markerPts3D.push_back(cv::Point3f(-markerLength/2.0,  markerLength/2.0, 0.0));
    markerPts3D.push_back(cv::Point3f(-markerLength/2.0, -markerLength/2.0, 0.0));
    markerPts3D.push_back(cv::Point3f( markerLength/2.0, -markerLength/2.0, 0.0));
    markerPts3D.push_back(cv::Point3f( markerLength/2.0,  markerLength/2.0, 0.0));
    cv::Vec3d r0;
    cv::solvePnP(markerPts3D, _cnrs2D, mtx, dist, r0, _t0, false);
    
    float x = (_cnrs2D[0].x+_cnrs2D[1].x+_cnrs2D[2].x+_cnrs2D[3].x)/4;
    float y = (_cnrs2D[0].y+_cnrs2D[1].y+_cnrs2D[2].y+_cnrs2D[3].y)/4;
    cv::Mat pNorm = (cv::Mat_<double>(3,1) << x,y,1);
//     cv::Point3d p((cv::Mat)(mtxInv*pNorm*_t0[2]-cv::Mat(_t0)));
    cv::Point3d p_cam((cv::Mat)(mtxInv*pNorm*_t0[2]));
//     _tf = (cv::Mat_<double>(3,3) << 0,1,0, 1,0,0, 0,0,-1);
    _tf = (cv::Mat_<double>(3,3) << 0,0,-1, 1,0,0, 0,-1,0);
//     cv::Point3d p_w = p_cam - cv::Point3d(cv::Mat(_t0));
    cv::Point3d p_w( cv::Mat(_tf*(cv::Mat(p_cam)-cv::Mat(_t0))) );
//     cv::Mat R0, R0inv;
    cv::Rodrigues(r0, _R0);
    // rotate the matrix wrt y axis by 90 degree
//     _R0 = (cv::Mat_<double>(3,3) << -1,0,0,0,1,0,0,0,-1) * _R0; // y90
//     _R0 = (cv::Mat_<double>(3,3) << -1,0,0, 0,-1,0, 0,0,1) * _R0;
    _R0inv = _R0.inv();
//     cout << "p = " << p << endl;
    _marker.ID = _id;
    _marker.center.x = p_w.x;
    _marker.center.y = p_w.y;
    _marker.center.z = p_w.z;
    
    _marker.points.clear();
//     _corners3D.clear();
    for (int iCnr=0;iCnr<4;iCnr++){
      pNorm = (cv::Mat_<double>(3,1) << _cnrs2D[iCnr].x,_cnrs2D[iCnr].y,1);
      cv::Mat p_w(markerPts3D[iCnr]);
      double z = _R0.at<double>(2,0)*p_w.at<float>(0,0) + _R0.at<double>(2,1)*p_w.at<float>(0,1) + _R0.at<double>(2,2)*p_w.at<float>(0,2) + _t0[2]; 
//       cv::Mat pp(_R0inv*(mtxInv*pNorm*z-cv::Mat(_t0)));
      cv::Mat pp(_tf*(mtxInv*pNorm*z-cv::Mat(_t0)));
//       cout << "p[" << iCnr << "]=" << pp << endl;
      geometry_msgs::Point pt3D;
      pt3D.x = pp.at<double>(0);
      pt3D.y = pp.at<double>(1);
      pt3D.z = pp.at<double>(2);
      _marker.points.push_back(pt3D);
    }
    
    _dt[0] = 0;_dt[1] = 0;_dt[2] = 0;
    _dR = cv::Mat::eye(3,3,CV_64FC1);
  }

  void updatePose(vector<cv::Point2f>cnrs){
//     cout << "update\n" << endl;
    cv::Vec3d dr, dt;
    cv::solvePnP(markerPts3D, cnrs, mtx, dist, dr, dt, false);
    cv::Mat dR, dRInv;
    cv::Rodrigues(dr, dR);
    dRInv = dR.inv();
    float x = (cnrs[0].x+cnrs[1].x+cnrs[2].x+cnrs[3].x)/4;
    float y = (cnrs[0].y+cnrs[1].y+cnrs[2].y+cnrs[3].y)/4;
    cv::Mat pNorm = (cv::Mat_<double>(3,1) << x,y,1);
    cv::Point3d p_cam(cv::Mat(mtxInv*pNorm*dt[2]));
//     cout << "_R0\n" <<_R0 << endl << "dRInv\n" << dRInv << endl << "p_cam\n" << cv::Mat(p_cam) << endl << "dt\n" << cv::Mat(dt) << endl << "_t0\n" << cv::Mat(_t0) << endl;
//     cv::Point3d p_w = cv::Point3d(cv::Mat(_R0*dRInv*(cv::Mat(p_cam)-cv::Mat(dt))+cv::Mat(_t0)));
    cv::Point3d p_w = cv::Point3d(cv::Mat(_tf*(cv::Mat(p_cam)-cv::Mat(_t0))));
    
    _marker.center.x = p_w.x;
    _marker.center.y = p_w.y;
    _marker.center.z = p_w.z;
    
    _marker.points.clear();
    for (int iCnr=0;iCnr<4;iCnr++){
      pNorm = (cv::Mat_<double>(3,1) << cnrs[iCnr].x,cnrs[iCnr].y,1);
      cv::Mat p_w(markerPts3D[iCnr]);
      double z = dR.at<double>(2,0)*p_w.at<float>(0,0) + dR.at<double>(2,1)*p_w.at<float>(0,1) + dR.at<double>(2,2)*p_w.at<float>(0,2) + dt[2]; 
      cv::Mat pp(_tf*((mtxInv*pNorm*z)-cv::Mat(_t0)));
      geometry_msgs::Point pt3D;
      pt3D.x = pp.at<double>(0);
      pt3D.y = pp.at<double>(1);
      pt3D.z = pp.at<double>(2);
      _marker.points.push_back(pt3D);
    }
  }
  
  void updatePoseIter(vector<cv::Point2f>cnrs){
//     cout << "updating" << endl;
    
//     vector<cv::Point2f> cnrsLast = _cnrs2D;
    // predicted points in 3D 
    vector<cv::Point3f> cornersPredict;
    for(int i=0;i<4;i++){
      cornersPredict.push_back(cv::Point3f(_marker.points[i].x, _marker.points[i].y, _marker.points[i].z));
    }
    // predicted point in 2D is _cnrs2D
    
    // compute residual. Format: [du1, dv1, du2, dv2, ..., du4, dv4]^T
    cv::Mat duv(cv::Size(1,8), CV_64FC1);
    for(int i=0;i<4;i++){
      duv.at<double>(i*2  , 0) = (double)(_cnrs2D[i].x-cnrs[i].x);
      duv.at<double>(i*2+1, 0) = (double)(_cnrs2D[i].y-cnrs[i].y);
    }
    
    int ct=0;
    cv::Mat duvsqr = duv.t()*duv;
    double sumsqr = duvsqr.at<double>(0,0);
    if (sumsqr > 30){
      cout << "duv0 = " << duv.t();
      cout << " " << sumsqr << endl;
    }
    
    while(ct <5 && sumsqr> 30){ //
      // construct jacobian matrix
      cv::Mat jacobian(cv::Size(6,8), CV_64FC1);
      double fx = mtx.at<double>(0,0), fy = mtx.at<double>(1,1);
//       cout << "fx = " << fx << ", fy=" <<fy << endl;
      for(int i=0;i<4;i++){
	double x = cornersPredict[i].x, y = cornersPredict[i].y, z = cornersPredict[i].z;
// 	cout << "cornersPredict[i] = " << cornersPredict[i] << endl << x << ", " << y << ", " << z << endl;
	jacobian.at<double>(i*2  , 0) = fx/z;
	jacobian.at<double>(i*2  , 1) = 0;
	jacobian.at<double>(i*2  , 2) = -fx*x/z/z;
	jacobian.at<double>(i*2  , 3) = -fx*x*y/z/z;
	jacobian.at<double>(i*2  , 4) = fx+fx*x*x/z/z;
	jacobian.at<double>(i*2  , 5) = -fx*y/z;
	jacobian.at<double>(i*2+1, 0) = 0;
	jacobian.at<double>(i*2+1, 1) = fy/z;
	jacobian.at<double>(i*2+1, 2) = -fy*y/z/z;
	jacobian.at<double>(i*2+1, 3) = -fy-fy*y*y/z/z;
	jacobian.at<double>(i*2+1, 4) = fy*x*y/z/z;
	jacobian.at<double>(i*2+1, 5) = fy*x/z;
      }
//       cout << jacobian << endl << endl;
      // [J] X [dT dphi] = [duv]
      // find dT and dphi
      cv::Mat dXI(cv::Size(6,1), CV_64FC1);
      cv::SVD svd(jacobian, cv::SVD::FULL_UV);
      svd.backSubst(duv, dXI);
//       _t0 += cv::Vec3d(dXI.at<double>(0,0), dXI.at<double>(1,0), dXI.at<double>(2,0));
      
      
      // update _dt, _dR
  //     Sophus::SO3 phiNew = Sophus::SO3::exp(Eigen::Vector3d(_r[0], _r[1], _r[2])) * Sophus::SO3::exp(Eigen::Vector3d(dTphi.at<double>(3,0), dTphi.at<double>(4,0), dTphi.at<double>(5,0)));
  //     Eigen::Vector3d phiNewEigen = phiNew.log();
  //     _r = cv::Vec3d(phiNewEigen[0], phiNewEigen[1], phiNewEigen[2]);
      cv::Mat drho = (cv::Mat_<double>(3,1) << dXI.at<double>(0,0), dXI.at<double>(1,0), dXI.at<double>(2,0) );
      cv::Mat dphi = (cv::Mat_<double>(3,1) << dXI.at<double>(3,0), dXI.at<double>(4,0), dXI.at<double>(5,0));
      cout << "rho=" << drho.t() << ",phi=" << dphi.t() << endl;
      double theta = sqrt(((cv::Mat)(dphi.t()*dphi)).at<double>(0,0));
      cv::Mat n = dphi/theta;
      cout << ",theta=" << theta << ", n=" << n.t() << endl;
      cv::Mat J = sin(theta)/theta*cv::Mat::eye(3,3,CV_64FC1) + (1-sin(theta)/theta)*n*n.t() + (1-cos(theta))/theta * cv::sfm::skew(n);
      cv::Vec3d dt(cv::Mat(J*drho));
      cout << "dt=" <<dt.t()<< endl;
      _t0 += dt;
//       _t0 += cv::Vec3d(cv::Mat(J*drho));

      cv::Mat dPHI;
      cv::Rodrigues(dphi, dPHI);
//       cout << "dPHI = \n" << dPHI << endl;
      _R0 = _R0*dPHI;

      // update cornersLast and cnrsLast
      _marker.points.clear();
      for(int i=0;i<4;i++){
// 	cv::Mat P_old = (cv::Mat)cornersLast[i];
	cv::Mat P_old = (cv::Mat)markerPts3D[i];
	P_old.convertTo(P_old, CV_64FC1);
// // 	cv::Point3d p3d(cv::Mat(_dR* P_old + (cv::Mat)_dt));
// // 	cornersLast[i] = p3d;
// // 	cv::Mat p_c = _R0 * (cv::Mat)p3d + (cv::Mat)_t0; // in camera frame
	cv::Mat P_c = _R0 * P_old + (cv::Mat)_t0;
	geometry_msgs::Point pt3D;
	pt3D.x = P_c.at<double>(0);
	pt3D.y = P_c.at<double>(1);
	pt3D.z = P_c.at<double>(2);
	_marker.points.push_back(pt3D);
	P_c = P_c / P_c.at<double>(2); // in img frame
	P_c = mtx * P_c; // homogerous coor in pic frame
	cv::Point2d p2d(P_c.at<double>(0), P_c.at<double>(1));
  //       cnrsLast[i] = p2d;
	
	
	duv.at<double>(i*2  , 0) = (double)(p2d.x-cnrs[i].x);
	duv.at<double>(i*2+1, 0) = (double)(p2d.y-cnrs[i].y);
      }
      cout << "duv" << ct << "= " << duv.t() << endl;
      ct++;
    }
    // in the end, update crnr coordinates for next frame
    _cnrs2D = cnrs;
    
    /*_corners = cnrs;
    vector<cv::Point3f> markerPts3D;
    markerPts3D.push_back(cv::Point3f(-markerLength/2.0,  markerLength/2.0, 0.0));
    markerPts3D.push_back(cv::Point3f(-markerLength/2.0, -markerLength/2.0, 0.0));
    markerPts3D.push_back(cv::Point3f( markerLength/2.0, -markerLength/2.0, 0.0));
    markerPts3D.push_back(cv::Point3f( markerLength/2.0,  markerLength/2.0, 0.0));
    cv::solvePnP(markerPts3D, _corners, mtx, dist, _r, _t, false);
    
    float x = (_corners[0].x+_corners[1].x+_corners[2].x+_corners[3].x)/4;
    float y = (_corners[0].y+_corners[1].y+_corners[2].y+_corners[3].y)/4;
    cv::Mat pNorm = (cv::Mat_<double>(3,1) << x,y,1);
    cv::Point3d p((cv::Mat)(mtxInv*pNorm*_t[2]));
//     _marker.ID = _id;
    _marker.center.x = p.x;
    _marker.center.y = p.y;
    _marker.center.z = p.z;
    
    _marker.points.clear();
    for (int iCnr=0;iCnr<4;iCnr++){
      pNorm = (cv::Mat_<double>(3,1) << _corners[iCnr].x,_corners[iCnr].y,1);
      p = ((cv::Point3d)(cv::Mat)(mtxInv*pNorm*_t[2]));
      geometry_msgs::Point pt3D;
      pt3D.x = p.x;
      pt3D.y = p.y;
      pt3D.z = p.z;
      _marker.points.push_back(pt3D);*/
    
    
  }
  
};


bool loadConfig(string filePath, cv::Mat* mtx, cv::Mat* dist, int* rateLost0, int* rateTracking1, double* markerLength, double* imgScale, string* msgTopic);

class ImageConverter{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher markerData_pub;
//   ros::Publisher isLost_pub;
//   ros::Publisher hover_pub;
//   std_msgs::Bool isLost;
//   bool isHover = false;
/********** Setup marker matters **********/
//   string paraPath = argv[2];//"./src/caliParameter.yaml"; //"~/catkin_ws/src/beginner_tutorials/src/caliParameter.yaml"
  int _keyIn;
  vector<int> markerIds;
  vector< vector< cv::Point2f > > markerCorners, rejectedCandidates;
  cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);

  cv::Mat imgMarker;
//   int imgCounter = 0;
  int state = 0;
  int freq = rateLost0;
//   vector<Marker3D> markerMap;
  map<int, MarkerPose3D>markersMap;
  
//   landing_vision::markerData msgMarker;
  landing_vision::markers msgMarkers;
  uint32_t seq = 0;
  
public:
  ImageConverter(): it_(nh_){
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe(msgTopic, 1, &ImageConverter::imageCb, this);
//     image_pub_ = it_.advertise(srvTopic, 1);
    markerData_pub = nh_.advertise<landing_vision::markers>(srvTopic_marker,1);
//     isLost_pub = nh_.advertise<std_msgs::Bool>(srvTopic_isLost,1);
//     isLost.data = true;
//     hover_pub = nh_.advertise<std_msgs::Empty>(srvTopic_hover,1);
    
    if (isLoaded){
      ROS_INFO("Parameters loaded.\n");
    }else{
      ROS_ERROR("Parameters not loaded. Please check the root. The file should be locate at the same folder as the imgGrab.cpp file\n roscd landing_vision\n");
    }
    
    
    cv::namedWindow(OPENCV_WINDOW);
}

  ~ImageConverter(){
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    try{
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    (cv_ptr->image).copyTo(imgMarker);
    if ( imgMarker.empty() ){
      return void();
    }
    
//     cv::threshold(imgMarker, imgMarker, 1, 128, cv::THRESH_BINARY);
//     if ( imgCounter % freq == 0){
    vector<int> markerIds;
    cv::aruco::detectMarkers(imgMarker, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
    if (markerCorners.size() > 0){
      freq = rateTracking1; // frequency set to MODE_tracking
//       if (isLost.data){
// 	ROS_INFO("Tracking started.");
//       }
//       isLost.data = false;
//       isLost_pub.publish(isLost);
      
      vector<cv::Vec3d> rvecs, tvecs;
      cv::aruco::estimatePoseSingleMarkers(markerCorners, markerLength, mtx, dist, rvecs, tvecs);
      msgMarkers.markers.clear();
      for(int i=0;i<markerCorners.size();i++){
// 	cout << "ID" << markerIds[i] << ":\n";
// 	ROS_INFO("ID %d detected", markerIds[i]);
	// pose estimation
	if (reintialize || markersMap.find(markerIds[i]) == markersMap.end()){
	  // new marker
// 	  MarkerPose3D p3d(markerIds[i], markerCorners[i]);
	  markersMap[markerIds[i]] = MarkerPose3D(markerIds[i], markerCorners[i]);
	  reintialize = false;
	}else{
	  // existing marker
// 	  markersMap[markerIds[i]] = MarkerPose3D(markerIds[i], markerCorners[i]);
	  markersMap[markerIds[i]].updatePose(markerCorners[i]);
	}

	msgMarkers.markers.push_back(markersMap[markerIds[i]]._marker);
	
	cv::aruco::drawAxis(imgMarker, mtx, dist, rvecs[i], tvecs[i],markerLength*0.4);
	bool isOldMarker = false;
// 	for (int j=0;j<markerMap.size();j++){
// 	  if (markerMap[j]._ID == markerIds[i]){
// 	    isOldMarker = true;
// 	    markerMap[j].addNewInfo(p);
// 	    break;
// 	  }
// 	}
// 	if (!isOldMarker){
// 	  markerMap.push_back(Marker3D(markerIds[i],p));
// 	}
      } // endfor process each marker
      msgMarkers.header.seq = seq;
      msgMarkers.header.stamp = ros::Time::now();
      markerData_pub.publish(msgMarkers);
//       cout << "----" << endl;
    }else{ // endif marker detected 
      freq = rateLost0; // frequency set to MODE_lost
//       if (!(isLost.data )){
// 	ROS_INFO("Tracking lost.");
//       }
//       isLost.data = true;
//       isLost_pub.publish(isLost);
    } // endelse of marker detected
//     for(int j=0;j<markerMap.size();j++){
//       if (markerMap[j]._frameLost == 0){
// 	markerMap[j]._frameLost++;
// 	continue;
//       }else if (markerMap[j]._frameLost < 10){
// 	cv::Point3d p;
// 	markerMap[j].guessPos(&p);
//       }else{
// 	// remove the marker info
// 	markerMap.erase(markerMap.begin()+j);
// 	j--;
//       }
//     }
    cv::aruco::drawDetectedMarkers(imgMarker, markerCorners, markerIds);
    
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, imgMarker);
    _keyIn = cv::waitKey(1);
    if (_keyIn == 'R' || _keyIn == 'r'){
      reintialize = true;
      cout << "Reset\n";
    }
    seq++;
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "imgGrab");
  if (argc > 1){
    filePath = argv[1];
    cout << "roslaunch mode detected.\n";
  }else{
    filePath = "./src/landing_vision/src/caliParameter.yaml";
    cout << "rosrun mode detected.\n";
  }
  isLoaded = loadConfig(filePath, &mtx, &dist, &rateLost0, &rateTracking1, &markerLength, &imgScale, &msgTopic);
  mtxInv = mtx.inv();

  ImageConverter ic;
  ros::spin();
  return 0;
}

bool loadConfig(string filePath, cv::Mat* mtx, cv::Mat* dist, int* rateLost0, int* rateTracking1, double* markerLength, double* imgScale, string* msgTopic){
  cv::FileStorage cvFile;
  bool isOpen = cvFile.open(filePath,cv::FileStorage::Mode::READ);
  bool isLoaded = true;
  if (isOpen){
    cvFile["mtx"] >> *mtx;
    if((*mtx).rows!=3 || (*mtx).cols != 3){
      cout  << "mtx invalid!\n";
      isLoaded = false;
    }
    cvFile["dist"] >> *dist;
    if((*dist).rows!=1 || (*dist).cols != 5){
      cout  << "dist invalid!\n";
      isLoaded = false;
    }
    cvFile["rateLost0"] >> *rateLost0;
    if (*rateLost0 <= 0){
      *rateLost0 = 1;
      cout << "rateLost0 invalid, default value(" << *rateLost0 << "1) is used.\n";
    }
    cvFile["rateTracking1"] >> *rateTracking1;
    if (*rateTracking1 <= 0){
      *rateTracking1 = 5;
      cout << "rateTracking1 invalid, default value(" << *rateTracking1  << ") is used.\n";
    }
    cvFile["markerLength"] >> *markerLength;
    if (*markerLength <= 0){
      *markerLength = 0.1;
      cout << "markerLength invalid, default value(" << *markerLength << ") is used.\n";
    }
    cvFile["imgScale"] >> *imgScale;
    if (*imgScale <= 0){
      *imgScale = 1;
      cout << "imgScale invalid, default value(" << *imgScale << ") is used.\n";
    }
    cvFile["imgTopic"] >> *msgTopic;
    cout << "image topic: " << *msgTopic << endl;
  }else{
    cout << filePath << " does not exist.\n";
  }
  cvFile.release();
  return (isOpen && isLoaded);
}