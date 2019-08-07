/*
 * pointcloud process ROS.cpp
 * Date: 2019-08-06
*/
#include <stdio.h>
#include <stdlib.h>
#include <fstream>

#include "ros/ros.h"
#include <std_srvs/SetBool.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


class camera_control_node
{

private:
  ros::NodeHandle node_;

  // Subscriber
  ros::Subscriber trigger_command_sub_;

  // Publisher
  ros::Publisher filter_pub_, odom_pub_, pub_undistorted_pc_;

  // Service
  //ros::ServiceServer clear_num_service_;

  // Params
  double pic_distance_threshold_;
  bool first_pose_;
  bool trigger_camera_;

  int pic_count_;


  std::string fix_frame_;
  std::string base_link_;
  std::string data_file_;
  std::string init_sh_;
  std::string capture_sh_;

  tf::TransformListener listener;
  tf::StampedTransform frame_transform_;

  geometry_msgs::PoseStamped current_pose_;
  geometry_msgs::PoseStamped last_pose_;

  std::ofstream out_data_path_;

public:

  camera_control_node();
  ~camera_control_node();
  void init();
  void triggerCallback(const std_msgs::Bool::ConstPtr& msg);
  bool lookUpTransform();
  void capture_camera();
  void update();
};

camera_control_node::camera_control_node()
: first_pose_(true)
, pic_count_(0)
, pic_distance_threshold_(2.0)
, trigger_camera_(false)
{
  // parameters
  ros::param::get("/camera_control_node/data_file", data_file_);
  ros::param::get("/camera_control_node/pic_distance_threshold", pic_distance_threshold_);
  ros::param::get("/camera_control_node/init_sh", init_sh_);
  ros::param::get("/camera_control_node/capture_sh", capture_sh_);
  if(!ros::param::get("/camera_control_node/fix_frame", fix_frame_))
  {
      fix_frame_ = "/map";
  }
  if(!ros::param::get("/camera_control_node/base_link", base_link_))
  {
      base_link_ = "/base_link";
  }
  // subscribers

  trigger_command_sub_ = node_.subscribe<std_msgs::Bool> ("/trigger", 1, &camera_control_node::triggerCallback, this);
  // imu_sub_ = node_.subscribe<sensor_msgs::Imu> ("/imu/data", 1, &camera_control_node::imuCallback, this);

  // publishers
  //image_pub_ = node_.advertise<sensor_msgs::Image>("/image", 1);

  // Services
  //clear_num_service_ = node_.advertiseService("/image_process/clear_num_service", &camera_control_node::clear_num_service, this);
  init();
  current_pose_.pose.position.x = 0;
  current_pose_.pose.position.y = 0;
  current_pose_.pose.position.z = 0;
  current_pose_.pose.orientation.x = 0;
  current_pose_.pose.orientation.y = 0;
  current_pose_.pose.orientation.z = 0;
  current_pose_.pose.orientation.w = 0;

  last_pose_ = current_pose_;

  out_data_path_.open(data_file_, std::ios_base::app);
    

}

camera_control_node::~camera_control_node()
{
  out_data_path_.close();
}

void camera_control_node::triggerCallback(const std_msgs::Bool::ConstPtr& msg)
{
  trigger_camera_ = true;
}

bool camera_control_node::lookUpTransform()
{
  try
  {
    listener.waitForTransform(fix_frame_, base_link_, ros::Time(0), ros::Duration(0.1));
    listener.lookupTransform(fix_frame_, base_link_, ros::Time(0), frame_transform_);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
    ros::Duration(0.1).sleep();
    return false;
  }

  current_pose_.pose.position.x = frame_transform_.getOrigin().x();
  current_pose_.pose.position.y = frame_transform_.getOrigin().y();
  current_pose_.pose.position.z = frame_transform_.getOrigin().z();

  current_pose_.pose.orientation.w = frame_transform_.getRotation().getW();
  current_pose_.pose.orientation.x = frame_transform_.getRotation().getX();
  current_pose_.pose.orientation.y = frame_transform_.getRotation().getY();
  current_pose_.pose.orientation.z = frame_transform_.getRotation().getZ();
  return true;
}

void camera_control_node::init()
{
  std::string ptp = "ptpcam";
  std::string set_property = "--set-property=";

  // std::string sync_time_cmd = ptp+ " " + set_property +"0x5011 --val=`date +%Y%m%dT%H%M%S`.`expr \`date +%N\` / 1000000`";
  // std::string set_single_shot_model = ptp+ " " + set_property +"0x5013 --val=0x0001";

  std::cout << "Send command: " << init_sh_ << std::endl;
  system(init_sh_.data());
}

void camera_control_node::capture_camera()
{
  std::string ptp = "ptpcam";
  std::string save = "--save-a-shot=";

  double camera_time = ros::Time::now().toSec();
  // std::stringstream ss;
  // ss << camera_time;

  std::stringstream trigger_count;
  trigger_count << pic_count_;
  std::string count = trigger_count.str();
  // std::string time = ss.str();
  //std::string trigger = ptp+ " " + save + ss.str() + ".jpg";
  std::string trigger = capture_sh_ + " " + count;
  std::cout << "Send command: " << trigger << std::endl;

  system(trigger.data());
  if (out_data_path_.is_open()) 
    out_data_path_ << std::setprecision(14) << pic_count_ << " " << camera_time << "\n";
  pic_count_++;

}

void camera_control_node::update()
{
  if(trigger_camera_)
  {
    capture_camera();
    last_pose_ = current_pose_;
    trigger_camera_ = false;
  }

  if(!lookUpTransform())
  {
    std::cout << "Didn't get pose.." << std::endl;
    return;
  }

  if(first_pose_)
  {
    last_pose_ = current_pose_;
    capture_camera();
    first_pose_ = false;
  }
  //std::cout << current_pose_.pose.position.x << " " << current_pose_.pose.position.y << " " << current_pose_.pose.position.z << std::endl;
  //std::cout << last_pose_.pose.position.x << " " << last_pose_.pose.position.y << " " << last_pose_.pose.position.z << std::endl;
  double distance = pow ((current_pose_.pose.position.x-last_pose_.pose.position.x)*(current_pose_.pose.position.x-last_pose_.pose.position.x) + 
                         (current_pose_.pose.position.y-last_pose_.pose.position.y)*(current_pose_.pose.position.y-last_pose_.pose.position.y) + 
                         (current_pose_.pose.position.z-last_pose_.pose.position.z)*(current_pose_.pose.position.z-last_pose_.pose.position.z), 1.0/3);
  std::cout << distance << std::endl;
  if(distance > pic_distance_threshold_)
  {
    capture_camera();
    last_pose_ = current_pose_;
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_control_node");
  camera_control_node node;

  ROS_INFO("slam front end ros node started...");

  ros::Rate rate(10);

  while(ros::ok())
  {
    ros::spinOnce();
    node.update();
    rate.sleep();
  }
  return 0;
}


