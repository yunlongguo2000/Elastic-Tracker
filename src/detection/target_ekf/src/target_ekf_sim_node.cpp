#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include "target_ekf/target_ekf.hpp"
#include <gazebo_msgs/ModelStates.h>
#include <rosgraph_msgs/Clock.h>

typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry>
  YoloOdomSyncPolicy; // 用于时间同步，只有输入的两个消息的时间戳近似匹配，才会调用回调函数
typedef message_filters::Synchronizer<YoloOdomSyncPolicy>
  YoloOdomSynchronizer;
ros::Publisher target_odom_pub_, yolo_odom_pub_, modelstate_pub_;
Eigen::Matrix3d cam2body_R_;
Eigen::Vector3d cam2body_p_;
double fx_, fy_, cx_, cy_, width_, height_;
ros::Time last_update_stamp_, gazebo_time;
double pitch_thr_ = 30;
bool check_fov_ = false;
Eigen::Vector3d last_target_pos_ = Eigen::Vector3d::Zero();

std::shared_ptr<Ekf> ekfPtr_;

//定时发布target的里程计信息（三维位置、速度、四元数），该里程计信息是通过EKF预测得到的
void predict_state_callback(const ros::TimerEvent& event) {
  double update_dt = (ros::Time::now() - last_update_stamp_).toSec();
  if (update_dt < 2.0) {
    ekfPtr_->predict();
  } else {
    //ROS_WARN("[ekf] too long time no update!");
    return;
  }
  // publish target odom
  nav_msgs::Odometry target_odom;
  target_odom.header.stamp = ros::Time::now();
  target_odom.header.frame_id = "world";
  target_odom.pose.pose.position.x = ekfPtr_->pos().x();
  target_odom.pose.pose.position.y = ekfPtr_->pos().y();
  target_odom.pose.pose.position.z = ekfPtr_->pos().z();
  target_odom.twist.twist.linear.x = ekfPtr_->vel().x();
  target_odom.twist.twist.linear.y = ekfPtr_->vel().y();
  target_odom.twist.twist.linear.z = ekfPtr_->vel().z();
  Eigen::Vector3d rpy = ekfPtr_->rpy();
  Eigen::Quaterniond q = euler2quaternion(rpy);
  target_odom.pose.pose.orientation.w = q.w();
  target_odom.pose.pose.orientation.x = q.x();
  target_odom.pose.pose.orientation.y = q.y();
  target_odom.pose.pose.orientation.z = q.z();
  target_odom_pub_.publish(target_odom);
}

void update_state_callback(const nav_msgs::OdometryConstPtr& target_msg, const nav_msgs::OdometryConstPtr& odom_msg) {
  // 只有输入的两个消息的时间戳近似匹配，才会调用回调函数
  // 为什么需要卡尔曼滤波：yolo得到目标的位姿信息，但速度信息不准确，通过EKF可以对目标的速度信息进行一个较为准确的估计？
  
  // std::cout << "yolo stamp: " << bboxes_msg->header.stamp << std::endl;
  // std::cout << "odom stamp: " << odom_msg->header.stamp << std::endl;
  Eigen::Vector3d odom_p, p;
  Eigen::Quaterniond odom_q, q;
  odom_p(0) = odom_msg->pose.pose.position.x; //无人机三维位置
  odom_p(1) = odom_msg->pose.pose.position.y;
  odom_p(2) = odom_msg->pose.pose.position.z;
  odom_q.w() = odom_msg->pose.pose.orientation.w; //无人机四元数
  odom_q.x() = odom_msg->pose.pose.orientation.x;
  odom_q.y() = odom_msg->pose.pose.orientation.y;
  odom_q.z() = odom_msg->pose.pose.orientation.z;
  
  Eigen::Vector3d cam_p = odom_q.toRotationMatrix() * cam2body_p_ + odom_p; //相机在机体坐标下的位置转换为在世界坐标系下的位置
  Eigen::Quaterniond cam_q = odom_q * Eigen::Quaterniond(cam2body_R_); //body2world，cam2body，相机在世界坐标系下的四元数

  p.x() = target_msg->pose.pose.position.x; //目标三维位置
  p.y() = target_msg->pose.pose.position.y;
  p.z() = target_msg->pose.pose.position.z;
  q.w() = target_msg->pose.pose.orientation.w; //目标四元数
  q.x() = target_msg->pose.pose.orientation.x;
  q.y() = target_msg->pose.pose.orientation.y;
  q.z() = target_msg->pose.pose.orientation.z;

  Eigen::Vector3d rpy = quaternion2euler(q); //目标欧拉角

  //根据目标上一时刻位置和的当前位置计算目标的欧拉角
  if(last_target_pos_ != Eigen::Vector3d::Zero()) 
  {
    Eigen::Vector3d direction_target = p - last_target_pos_;
    double yaw = atan2(direction_target.y(), direction_target.x());
    double pitch = 0;
    double roll = 0;
    rpy = Eigen::Vector3d(roll, pitch, yaw);
    std::cout<<"yaw:"<<yaw<<std::endl;
  }
  last_target_pos_ = p;

  // NOTE check whether it's in FOV
  // 检查目标是否在第一人称视角内
  if (check_fov_) { //需要检查
    Eigen::Vector3d p_in_body = cam_q.inverse() * (p - cam_p);
    if (p_in_body.z() < 0.1 || p_in_body.z() > 5.0) {
      return;
    }
    double x = p_in_body.x() * fx_ / p_in_body.z() + cx_; //像素坐标u
    if (x < 0 || x > height_) {
      return;
    }
    double y = p_in_body.y() * fy_ / p_in_body.z() + cy_; //像素坐标v
    if (y < 0 || y > width_) {
      return;
    }
  }

  // update target odom
  // 根据yolo传回的目标里程计信息，更新EKF参数
  double update_dt = (ros::Time::now() - last_update_stamp_).toSec();
  if (update_dt > 5.0) {
    ekfPtr_->reset(p, rpy);
    ROS_WARN("[ekf] reset!");
  } else if (ekfPtr_->update(p, rpy)) {
    // ROS_WARN("[ekf] update!");
  } else {
    ROS_ERROR("[ekf] update invalid!");
    return;
  }
  last_update_stamp_ = ros::Time::now();
}

void odom_callback(const nav_msgs::OdometryConstPtr& odom_msg) {
  // std::cout << "_now stamp: " << odom_msg->header.stamp << std::endl;
}

void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    double x, y, z, qw, qx, qy, qz, linear_x, linear_y, linear_z, angular_x, angular_y, angular_z;
    // 遍历模型列表
    for (int i = 0; i < msg->name.size(); i++)
    {
    	// 如果不确定模型的名字，可以先打印出来看一看
    	// std::cout << "model_name: " << msg->name[i] << std::endl;
        // 如果当前模型是 our_model
        if (msg->name[i] == "actor")
        {
            // 获取模型的位置和姿态
            geometry_msgs::Pose pose = msg->pose[i];
            // 处理信息
            x = pose.position.x;
            y = pose.position.y;
            z = pose.position.z;
            qw = pose.orientation.w;
            qx = pose.orientation.x;
            qy = pose.orientation.y;
            qz = pose.orientation.z;
            
            // 获取模型的速度和角速度
            geometry_msgs::Twist twist = msg->twist[i];
            // 处理信息
            linear_x = twist.linear.x;    // 模型线速度在 x 轴的分量
            linear_y = twist.linear.y;    // 模型线速度在 y 轴的分量
            linear_z = twist.linear.z;    // 模型线速度在 z 轴的分量
            angular_x = twist.angular.x;  // 模型角速度在 x 轴的分量
            angular_y = twist.angular.y;  // 模型角速度在 y 轴的分量
            angular_z = twist.angular.z;  // 模型角速度在 z 轴的分量
            break;
        }
    }
    // publish target odom
    nav_msgs::Odometry target_odom;
    target_odom.header.stamp = gazebo_time;
    target_odom.header.frame_id = "world";
    target_odom.pose.pose.position.x = x;
    target_odom.pose.pose.position.y = y;
    target_odom.pose.pose.position.z = z;
    //target_odom.twist.twist.linear.x = linear_x;
    //target_odom.twist.twist.linear.y = linear_y;
    //target_odom.twist.twist.linear.z = linear_z;
    //target_odom.pose.pose.orientation.w = qw;
    //target_odom.pose.pose.orientation.x = qx;
    //target_odom.pose.pose.orientation.y = qy;
    //target_odom.pose.pose.orientation.z = qz;
    //modelstate_pub_.publish(target_odom);
}

void gazebo_clock_Callback(const rosgraph_msgs::ClockConstPtr& msg)
{
  //std::cout<<"time: "<<msg->clock.toSec()<<std::endl;
  gazebo_time = msg->clock;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "target_ekf");
  ros::NodeHandle nh("~");
  last_update_stamp_ = ros::Time::now() - ros::Duration(10.0);
  
  //相机外参：相机相对于机体的旋转矩阵和位置坐标（机体坐标系）
  std::vector<double> tmp;
  if (nh.param<std::vector<double>>("cam2body_R", tmp, std::vector<double>())) {
    cam2body_R_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(tmp.data(), 3, 3);
  }
  if (nh.param<std::vector<double>>("cam2body_p", tmp, std::vector<double>())) {
    cam2body_p_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(tmp.data(), 3, 1);
  }
  //相机内参：cx，cy为nav_msgs::Odometry像素坐标系原点到图像中心点的距离，相当于u0，v0
  nh.getParam("cam_fx", fx_);
  nh.getParam("cam_fy", fy_);
  nh.getParam("cam_cx", cx_);
  nh.getParam("cam_cy", cy_);
  //相机摄像头宽度和高度的分辨率
  nh.getParam("cam_width", width_);
  nh.getParam("cam_height", height_);

  nh.getParam("pitch_thr", pitch_thr_);
  nh.getParam("check_fov", check_fov_);

  message_filters::Subscriber<nav_msgs::Odometry> yolo_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;

  std::shared_ptr<YoloOdomSynchronizer> yolo_odom_sync_Ptr_;
  ros::Timer ekf_predict_timer_;
  ros::Subscriber single_odom_sub = nh.subscribe("odom", 100, &odom_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber modelstate_sub = nh.subscribe("/gazebo/model_states", 1000, modelStatesCallback);
  ros::Subscriber gazebo_clock_sub = nh.subscribe("/clock", 1, gazebo_clock_Callback);

  modelstate_pub_ = nh.advertise<nav_msgs::Odometry>("yolo", 1);

  target_odom_pub_ = nh.advertise<nav_msgs::Odometry>("target_odom", 1);
  yolo_odom_pub_ = nh.advertise<nav_msgs::Odometry>("yolo_odom", 1);

  int ekf_rate = 20;
  nh.getParam("ekf_rate", ekf_rate);
  ekfPtr_ = std::make_shared<Ekf>(1.0 / ekf_rate);

  yolo_sub_.subscribe(nh, "yolo", 1, ros::TransportHints().tcpNoDelay()); //这里的yolo实际是/target/odom，在xtdrone中使用的话应将其换为真正的yolo话题
  odom_sub_.subscribe(nh, "odom", 100, ros::TransportHints().tcpNoDelay());
  yolo_odom_sync_Ptr_ = std::make_shared<YoloOdomSynchronizer>(YoloOdomSyncPolicy(200), yolo_sub_, odom_sub_); //创建一个用于同步处理Yolo检测结果和里程计信息的对象，并设置回调函数来处理同步后的数据。
  yolo_odom_sync_Ptr_->registerCallback(boost::bind(&update_state_callback, _1, _2));
  ekf_predict_timer_ = nh.createTimer(ros::Duration(1.0 / ekf_rate), &predict_state_callback); //回调函数中定时发布target的里程计信息

  ros::spin();
  return 0;
}
