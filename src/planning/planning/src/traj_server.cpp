#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PolyTraj.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>

#include <traj_opt/poly_traj_utils.hpp>
#include <geometry_msgs/PoseStamped.h>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

ros::Publisher pos_cmd_pub_, pose_enu_pub, cmd_pub;
ros::Time heartbeat_time_;
std::string vehicle_id;
bool receive_traj_ = false;
bool flight_start_ = false;
quadrotor_msgs::PolyTraj trajMsg_, trajMsg_last_;
Eigen::Vector3d last_p_;
double last_yaw_ = 0;

//
bool task_received_ = true;
bool first_change = true;
//

struct Quaternion {
    double w, x, y, z;
};
 
struct EulerAngles {
    double roll, pitch, yaw;
};
//欧拉角转化四元数
Quaternion ToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    Quaternion q;
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;
    return q;
}
//四元数转化为欧拉角
EulerAngles ToEulerAngles(Quaternion q) {
    EulerAngles angles;
 
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);
 
    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);
 
    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);
 
    return angles;
}

void publish_cmd(int traj_id,
                 const Eigen::Vector3d &p,
                 const Eigen::Vector3d &v,
                 const Eigen::Vector3d &a,
                 double y, double yd) {
  quadrotor_msgs::PositionCommand cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id;

  cmd.position.x = p(0);
  cmd.position.y = p(1);
  cmd.position.z = p(2);
  cmd.velocity.x = v(0);
  cmd.velocity.y = v(1);
  cmd.velocity.z = v(2);
  cmd.acceleration.x = a(0);
  cmd.acceleration.y = a(1);
  cmd.acceleration.z = a(2);
  cmd.yaw = y;
  cmd.yaw_dot = yd;
  //pos_cmd_pub_.publish(cmd);
  last_p_ = p;
  geometry_msgs::Pose pose;
  pose.position.x = p(0);
  pose.position.y = p(1);
  pose.position.z = p(2);
  Quaternion q;
  q = ToQuaternion(cmd.yaw, 0 , 0);
  pose.orientation.w = q.w;
  pose.orientation.x = q.x;
  pose.orientation.y = q.y;
  pose.orientation.z = q.z;

  // std::cout << "task_received_: " << task_received_ << std::endl;
  
  if(task_received_) //追踪
  {
    first_change = true;
    pos_cmd_pub_.publish(cmd);
    pose_enu_pub.publish(pose);
    std::cout << "Following task!!!!!!!!!" << std::endl;
  }
  // else
  // {
  //   std::cout<<"Cruising task, stop publishing position command."<<std::endl;
  //   if(first_change)
  //   {
  //     std_msgs::String cmd_msg;
  //     cmd_msg.data = "HOVER";
  //     cmd_pub.publish(cmd_msg);
  //     first_change = false;
  //   }
  // }
}

bool exe_traj(const quadrotor_msgs::PolyTraj &trajMsg) {
  double t = (ros::Time::now() - trajMsg.start_time).toSec();
  if (t > 0) {
    if (trajMsg.hover) {
      if (trajMsg.hover_p.size() != 3) {
        ROS_ERROR("[traj_server] hover_p is not 3d!");
      }
      Eigen::Vector3d p, v0;
      p.x() = trajMsg.hover_p[0];
      p.y() = trajMsg.hover_p[1];
      p.z() = trajMsg.hover_p[2];
      v0.setZero();
      publish_cmd(trajMsg.traj_id, p, v0, v0, last_yaw_, 0);  // TODO yaw
      return true;
    }
    if (trajMsg.order != 5) {
      ROS_ERROR("[traj_server] Only support trajectory order equals 5 now!");
      return false;
    }
    if (trajMsg.duration.size() * (trajMsg.order + 1) != trajMsg.coef_x.size()) {
      ROS_ERROR("[traj_server] WRONG trajectory parameters!");
      return false;
    }
    int piece_nums = trajMsg.duration.size();
    std::vector<double> dura(piece_nums);
    std::vector<CoefficientMat> cMats(piece_nums);
    for (int i = 0; i < piece_nums; ++i) {
      int i6 = i * 6;
      cMats[i].row(0) << trajMsg.coef_x[i6 + 0], trajMsg.coef_x[i6 + 1], trajMsg.coef_x[i6 + 2],
          trajMsg.coef_x[i6 + 3], trajMsg.coef_x[i6 + 4], trajMsg.coef_x[i6 + 5];
      cMats[i].row(1) << trajMsg.coef_y[i6 + 0], trajMsg.coef_y[i6 + 1], trajMsg.coef_y[i6 + 2],
          trajMsg.coef_y[i6 + 3], trajMsg.coef_y[i6 + 4], trajMsg.coef_y[i6 + 5];
      cMats[i].row(2) << trajMsg.coef_z[i6 + 0], trajMsg.coef_z[i6 + 1], trajMsg.coef_z[i6 + 2],
          trajMsg.coef_z[i6 + 3], trajMsg.coef_z[i6 + 4], trajMsg.coef_z[i6 + 5];

      dura[i] = trajMsg.duration[i];
    }
    Trajectory traj(dura, cMats);
    if (t > traj.getTotalDuration()) {
      ROS_ERROR("[traj_server] trajectory too short left!");
      return false;
    }
    Eigen::Vector3d p, v, a;
    p = traj.getPos(t);
    v = traj.getVel(t);
    a = traj.getAcc(t);
    // NOTE yaw
    double yaw = trajMsg.yaw;
    double d_yaw = yaw - last_yaw_;
    d_yaw = d_yaw >= M_PI ? d_yaw - 2 * M_PI : d_yaw;
    d_yaw = d_yaw <= -M_PI ? d_yaw + 2 * M_PI : d_yaw;
    double d_yaw_abs = fabs(d_yaw);
    if (d_yaw_abs >= 0.02) {
      yaw = last_yaw_ + d_yaw / d_yaw_abs * 0.02;
    }
    publish_cmd(trajMsg.traj_id, p, v, a, yaw, 0);  // TODO yaw
    last_yaw_ = yaw;
    return true;
  }
  return false;
}

void heartbeatCallback(const std_msgs::EmptyConstPtr &msg) {
  heartbeat_time_ = ros::Time::now();
}

void polyTrajCallback(const quadrotor_msgs::PolyTrajConstPtr &msgPtr) {
  trajMsg_ = *msgPtr;
  std::cout << "receive_traj_=======================: " << receive_traj_ << std::endl;
  if (!receive_traj_) {
    trajMsg_last_ = trajMsg_;
    receive_traj_ = true;
    std::cout << "receive_traj_: " << receive_traj_ << std::endl;
  }
}

void cmdCallback(const ros::TimerEvent &e) {
  // std::cout << "receive_traj: " << receive_traj_ << std::endl;
  if (!receive_traj_) {
    return;
  }
  ros::Time time_now = ros::Time::now();
  std::cout << "task_received_: " << task_received_ << std::endl;
  // if(task_received_) //追踪
  // {
  //   first_change = true;
  //   pos_cmd_pub_.publish(cmd);
  //   pose_enu_pub.publish(pose);
  //   std::cout << "Following task!!!!!!!!!" << std::endl;
  // }
  if(!task_received_)
  {
    std::cout<<"Cruising task, stop publishing position command."<<std::endl;
    if(first_change)
    {
      std_msgs::String cmd_msg;
      cmd_msg.data = "HOVER";
      cmd_pub.publish(cmd_msg);
      first_change = false;
    }
  }
  
  if ((time_now - heartbeat_time_).toSec() > 0.5) {
    ROS_ERROR_ONCE("[traj_server] Lost heartbeat from the planner, is he dead?");
    publish_cmd(trajMsg_.traj_id, last_p_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0, 0);  // TODO yaw
    return;
  }
  if (exe_traj(trajMsg_)) 
  {
    trajMsg_last_ = trajMsg_;
    return;
  } 
  else if (exe_traj(trajMsg_last_)) 
  {
    return;
  }
}

//
void taskCallback(const std_msgs::Bool::ConstPtr& msg)
{
  task_received_ = msg->data;
  
}
//

int main(int argc, char **argv) {
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle nh("~");

  std::cout<<"traj_server start!"<<std::endl;
  
  nh.param<std::string>("vehicle_id", vehicle_id, "7");
  // ROS_INFO("vehicle_id: %s\n", vehicle_id.c_str());

  ros::Subscriber poly_traj_sub = nh.subscribe("trajectory", 10, polyTrajCallback);
  ros::Subscriber heartbeat_sub = nh.subscribe("heartbeat", 10, heartbeatCallback);

  pos_cmd_pub_ = nh.advertise<quadrotor_msgs::PositionCommand>("position_cmd", 50);

  std::string pose_pub_name = "/xtdrone/typhoon_h480_" + vehicle_id + "/cmd_pose_enu";
  // ROS_INFO("pose_pub_name: %s\n", pose_pub_name)
  pose_enu_pub = nh.advertise<geometry_msgs::Pose>(pose_pub_name, 1); 

  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);
  
  //
  ros::Subscriber task_sub_ = nh.subscribe("task_flag", 1, taskCallback);
  std::string cmd_pub_name = "/xtdrone/typhoon_h480_" + vehicle_id + "/cmd";
  // ROS_INFO("cmd_pub_name: %s\n", cmd_pub_name)
  cmd_pub = nh.advertise<std_msgs::String>(cmd_pub_name, 3);
  //

  ros::Duration(1.0).sleep();

  ROS_WARN("[Traj server]: ready.");

  ros::spin();

  return 0;
}