/** ****************************************************************************************
*  This node presents a fast and precise method to estimate the planar motion of a lidar
*  from consecutive range scans. It is very useful for the estimation of the robot odometry from
*  2D laser range measurements.
*  This module is developed for mobile robots with innacurate or inexistent built-in odometry.
*  It allows the estimation of a precise odometry with low computational cost.
*  For more information, please refer to:
*
*  Planar Odometry from a Radial Laser Scanner. A Range Flow-based Approach. ICRA'16.
*  Available at: http://mapir.uma.es/papersrepo/2016/2016_Jaimez_ICRA_RF2O.pdf
*
* Maintainer: Javier G. Monroy
* MAPIR group: https://mapir.isa.uma.es
*
* Modifications: Jeremie Deray & (see contributons on github)
******************************************************************************************** */

#include "rf2o_laser_odometry/CLaserOdometry2DNode.hpp"

using namespace rf2o;

CLaserOdometry2DNode::CLaserOdometry2DNode(): Node("CLaserOdometry2DNode")
{
  RCLCPP_INFO(get_logger(), "Initializing RF2O node...");

  // Read Parameters
  //----------------
  this->declare_parameter<std::string>("laser_scan_topic", "/scan");
  this->get_parameter("laser_scan_topic", laser_scan_topic);
  this->declare_parameter<std::string>("odom_topic", "/odom_rf2o");
  this->get_parameter("odom_topic", odom_topic);
  this->declare_parameter<std::string>("base_frame_id", "base_link");
  this->get_parameter("base_frame_id", base_frame_id);
  this->declare_parameter<std::string>("odom_frame_id", "odom");
  this->get_parameter("odom_frame_id", odom_frame_id);
  this->declare_parameter<bool>("publish_tf", true);
  this->get_parameter("publish_tf", publish_tf);
  this->declare_parameter<std::string>("init_pose_from_topic", "/base_pose_ground_truth");
  this->get_parameter("init_pose_from_topic", init_pose_from_topic);
  this->declare_parameter<double>("freq", 10.0);
  this->get_parameter("freq", freq);

  // Declare and get laser pose parameters (as an example: x, y, z, roll, pitch, yaw)
  double laser_x = 0.0, laser_y = 0.0, laser_z = 0.0, laser_roll = 0.0, laser_pitch = 0.0, laser_yaw = 0.0;
  this->declare_parameter<double>("base_link_to_laser_tf.x", 0.0);
  this->declare_parameter<double>("base_link_to_laser_tf.y", 0.0);
  this->declare_parameter<double>("base_link_to_laser_tf.z", 0.0);
  this->declare_parameter<double>("base_link_to_laser_tf.roll", 0.0);
  this->declare_parameter<double>("base_link_to_laser_tf.pitch", 0.0);
  this->declare_parameter<double>("base_link_to_laser_tf.yaw", 0.0);
  this->get_parameter("base_link_to_laser_tf.x", laser_x);
  this->get_parameter("base_link_to_laser_tf.y", laser_y);
  this->get_parameter("base_link_to_laser_tf.z", laser_z);
  this->get_parameter("base_link_to_laser_tf.roll", laser_roll);
  this->get_parameter("base_link_to_laser_tf.pitch", laser_pitch);
  this->get_parameter("base_link_to_laser_tf.yaw", laser_yaw);

  // Declare and get initial robot pose parameters (x, y, z, roll, pitch, yaw)
  double init_x = 0.0, init_y = 0.0, init_z = 0.0, init_roll = 0.0, init_pitch = 0.0, init_yaw = 0.0;
  this->declare_parameter<double>("initial_pose.x", 0.0);
  this->declare_parameter<double>("initial_pose.y", 0.0);
  this->declare_parameter<double>("initial_pose.z", 0.0);
  this->declare_parameter<double>("initial_pose.roll", 0.0);
  this->declare_parameter<double>("initial_pose.pitch", 0.0);
  this->declare_parameter<double>("initial_pose.yaw", 0.0);
  this->get_parameter("initial_pose.x", init_x);
  this->get_parameter("initial_pose.y", init_y);
  this->get_parameter("initial_pose.z", init_z);
  this->get_parameter("initial_pose.roll", init_roll);
  this->get_parameter("initial_pose.pitch", init_pitch);
  this->get_parameter("initial_pose.yaw", init_yaw);

  // Compose the laser_tf pose
  laser_tf = Pose3d::Identity();
  laser_tf.linear() = matrixRollPitchYaw(laser_roll, laser_pitch, laser_yaw).cast<double>();
  laser_tf.translation()(0) = laser_x;
  laser_tf.translation()(1) = laser_y;
  laser_tf.translation()(2) = laser_z;

  // Init Publishers and Subscribers
  //---------------------------------
  if (publish_tf)
  {
    RCLCPP_INFO(get_logger(), "Publishing TF: [base_link] to [odom]");
    odom_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  }
  odom_pub  = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 5);
  laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(laser_scan_topic,rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
      std::bind(&CLaserOdometry2DNode::LaserCallBack, this, std::placeholders::_1));
  
  // Initialize pose
  if (init_pose_from_topic != "")
  {
    initPose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        init_pose_from_topic,
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
        std::bind(&CLaserOdometry2DNode::initPoseCallBack, this, std::placeholders::_1));
    GT_pose_initialized  = false;
  }
  else
  {
    // init to parameters
    GT_pose_initialized = true;
    initial_robot_pose.pose.pose.position.x = init_x;
    initial_robot_pose.pose.pose.position.y = init_y;
    initial_robot_pose.pose.pose.position.z = init_z;
    tf2::Quaternion q;
    q.setRPY(init_roll, init_pitch, init_yaw);
    initial_robot_pose.pose.pose.orientation = tf2::toMsg(q);
  }

  // Init variables
  rf2o_ref.module_initialized = false;
  rf2o_ref.first_laser_scan   = true;
}


/**
 * Keeps the last scan from the 2D lidar to be latter processed
 * On the first laser scan, the node is initialized.
*/
void CLaserOdometry2DNode::LaserCallBack(const sensor_msgs::msg::LaserScan::SharedPtr new_scan)
{
  if (GT_pose_initialized)
  {
    // Keep in memory the last received laser_scan
    last_scan = *new_scan;
    rf2o_ref.current_scan_time = last_scan.header.stamp;
    
    if (rf2o_ref.first_laser_scan == false)
    {
      // copy laser range data to rf2o internal variable
      std::copy(new_scan->ranges.begin(), new_scan->ranges.begin() + rf2o_ref.width, rf2o_ref.range_wf.data());
      // inform of new scan available
      new_scan_available = true;
    }
    else
    {
      // Initialize module on first scan (from laser params)
      rf2o_ref.setLaserPose(laser_tf);
      rf2o_ref.init(last_scan, initial_robot_pose.pose.pose);
      rf2o_ref.first_laser_scan = false;
    }
  }
}


bool CLaserOdometry2DNode::scan_available()
{
  return new_scan_available;
}


/**
 * Process the last scans to estimate the current odometry
*/
void CLaserOdometry2DNode::process()
{
  // Do only run when a new scan is ready 
  if( rf2o_ref.is_initialized() && scan_available() )
  {
    // Process odometry estimation
    rf2o_ref.odometryCalculation(last_scan);

    // Publish odometry over ROS2 (tf/topic)
    publish();

    // Do not run on the same data!
    new_scan_available = false;
  }
  else
  {
    // This is a warning. We depend on laser scans, so no meaning running faster than scan freq.
    RCLCPP_DEBUG(get_logger(), "Waiting for laser_scans....");
  }
}


/**
 * This function is used to initialize the robot pose before estimating its odometry.
 * By default the odometry will start from pose_0, but when comparing different methods
 * it may be necessary to start from a different pose.
*/
void CLaserOdometry2DNode::initPoseCallBack(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr new_initPose)
{
  // Initialize module on first GT pose. Else do Nothing!
  if (!GT_pose_initialized)
  {
    initial_robot_pose.pose.pose = new_initPose->pose.pose;
    initial_robot_pose.pose.covariance = new_initPose->pose.covariance;
    initial_robot_pose.header = new_initPose->header;
    GT_pose_initialized = true;
  }
}


/**
 * Publish current odocmetry estimation over ROS
 * According to the node parameters it will publish over tf and/or especified topic
*/
void CLaserOdometry2DNode::publish()
{
  // 1. publish odom as a topic (no harm!)
  RCLCPP_DEBUG(get_logger(), "Publishing odom over topic:[%s]", odom_topic.c_str());
  tf2::Quaternion tf_quaternion;
  tf_quaternion.setRPY(0.0, 0.0, rf2o::getYaw(rf2o_ref.robot_pose_.rotation()));
  geometry_msgs::msg::Quaternion quaternion = tf2::toMsg(tf_quaternion);
  
  // compose odom msg
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = rf2o_ref.last_odom_time;    // the time of the last scan used!
  odom.header.frame_id = odom_frame_id;
  //set the position
  odom.pose.pose.position.x = rf2o_ref.robot_pose_.translation()(0);
  odom.pose.pose.position.y = rf2o_ref.robot_pose_.translation()(1);
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = quaternion;
  //set the velocity
  odom.child_frame_id = base_frame_id;
  odom.twist.twist.linear.x = rf2o_ref.lin_speed;    //linear speed
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = rf2o_ref.ang_speed;   //angular speed
  //publish the message
  odom_pub->publish(odom);

  // 2. publish over tf? (one one node should publish this transform!)
  if (publish_tf)
  {
    RCLCPP_DEBUG(get_logger(), "Publishing TF: [base_link] to [odom]");
    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp = rf2o_ref.last_odom_time;    // the time of the last scan used!
    odom_trans.header.frame_id = odom_frame_id;
    odom_trans.child_frame_id = base_frame_id;
    odom_trans.transform.translation.x = rf2o_ref.robot_pose_.translation()(0);
    odom_trans.transform.translation.y = rf2o_ref.robot_pose_.translation()(1);
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = quaternion;
    //send the transform
    odom_broadcaster->sendTransform(odom_trans);
  }
}


//-----------------------------------------------------------------------------------
//                                   MAIN
//-----------------------------------------------------------------------------------
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto myLaserOdomNode = std::make_shared<rf2o::CLaserOdometry2DNode>();

  // set desired loop rate
  rclcpp::Rate rate(myLaserOdomNode->freq);

  // Loop
  while (rclcpp::ok()){ 
      rclcpp::spin_some(myLaserOdomNode);
      myLaserOdomNode->process();
      rate.sleep();
  }

  return 0;
}