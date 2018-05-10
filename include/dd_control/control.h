#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/RCIn.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace dd_control
{
class Control
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  // Transform
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener* tf_listener_;

  ros::Subscriber velocity_sub_;
  ros::Subscriber setpoint_sub_;
  ros::Subscriber cancel_sub_;
  ros::Subscriber rc_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber current_velocity_sub_;
  ros::Subscriber odom_sub_;

  ros::Publisher control_pub_;

  ros::Timer pub_timer_;

  int mode_;

  std::string output_frame_;

  geometry_msgs::PoseStamped last_setpoint_;
  geometry_msgs::PoseStamped current_pose_;
  geometry_msgs::TwistStamped current_global_velocity_;
  geometry_msgs::TwistStamped current_local_velocity_;

  // Max velocity
  double max_x_vel_;
  double max_y_vel_;
  double max_z_vel_;

  // P-controller
  double k_p_x_;
  double k_p_y_;
  double k_p_z_;

public:
  Control(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);

private:
  void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);

  void setpointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

  void cancelCallback(const std_msgs::Header::ConstPtr& msg);

  void rcCallback(const mavros_msgs::RCIn::ConstPtr& msg);

  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

  void currentVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

  void timerPublish(const ros::TimerEvent& event);

  void setpointPublish(const geometry_msgs::PoseStamped& setpoint);

  double clamp(double value, double min, double max);
};
}
