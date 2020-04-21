#include <iostream>
#include <string>
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;


class ScanMatching {

  private:
    ros::NodeHandle nh_;
    vector<geometry_msgs::Point> points_;
    vector<geometry_msgs::Point> transformed_points_;
    vector<geometry_msgs::Point> prev_points_;
    tf::TransformBroadcaster br_;
    tf::Transform tr_;
    geometry_msgs::PoseStamped estimated_pose_;
    const int MAX_ITERATIONS = 100;
    //Topic names for publishers and subscribers
    const string& SCAN_TOPIC  = "/scan";
    const string& POSE_TOPIC = "/scan_match_pose";
    const string& ODOM_TOPIC = "/odom";
    const string& FAKE_SCAN_TOPIC = "/fake_scan_match";
    const string& DRIVE_TOPIC = "/nav";

    //Publishers
    ros::Publisher pose_pub_;
    ros::Publisher fake_scan_pub_;
    ros::Publisher drive_pub_;

    //Subscribers
    ros::Subscriber odom_sub_;
    ros::Subscriber scan_sub_;

  public:

    ScanMatching(ros::NodeHandle& nh) :nh_(nh) {
      cout<<"Node started..\n";
      //Publishers : Add others as per your need
      drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(DRIVE_TOPIC, 1);
      fake_scan_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(FAKE_SCAN_TOPIC, 1);
      pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(POSE_TOPIC, 1);
      //Subscriber for odometry and laserscan
      odom_sub_ = nh_.subscribe(ODOM_TOPIC, 10, &ScanMatching::odom_callback, this);
      scan_sub_ = nh_.subscribe(SCAN_TOPIC, 1, &ScanMatching::scan_callback, this);
    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg){
        //get odometry information from odometry topic
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
        //get laserscan data from the scan topic
    }

    void do_scan_matching() {
      //The following steps are inspired from the Censi's PL-ICP paper. Try to modularize your code for each step to make it more readable and debuggable.
      //Use Eigen math library for linear algebra, matrix and vector operations, geometrical transformations.
      /*1.Compute the coordinates of the second scan’s points in the first scan’s frame of reference, 
       according to the roto-translation obtained from odometry update.*/



      /*2.Find correspondence between points of the current and previous frame. You can use naive way of looking 
      through all points in sequence or use radial ordering of laser points to speed up the search.*/


      //3. Based on the correspondences, find the necessary tranform.
           //3.a. Construct the necessary matrices as shown in the paper for solution with Lagrange's multipliers.

           //3.b. You should get a fourth order polynomial in lamda which you can solve to get value(hint:greatest real root of polynomial equn) of lamda.

           //3.b. Use the calcualted value of lamda to estimate the transform using equation 24 in the Censi's paper.


      //4.Publish the estimated pose from scan matching based on the transform obstained. You can visualize the pose in rviz.


      /*5.Also transform the previous frame laserscan points using the roto-translation transform obtained and visualize it. Ideally, this should coincide
      with your actual current laserscan message.*/
    }

    ~ScanMatching() {}
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "scan_matcher");
  ros::NodeHandle nh;
  ScanMatching scan_matching(nh);
  scan_matching.do_scan_matching();
  ros::spin();
  return 0;
}
