#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
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

  ros::Publisher control_pub_;

  ros::Timer velocity_pub_timer_;
  ros::Timer setpoint_pub_timer_;

  int mode_;

  std::string robot_frame_;

  geometry_msgs::PoseStamped last_setpoint_;
  geometry_msgs::PoseStamped current_pose_;

public:
  Control(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);

private:
  void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);

  void setpointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

  void cancelCallback(const std_msgs::Header::ConstPtr& msg);

  void rcCallback(const mavros_msgs::RCIn::ConstPtr& msg);

  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

  void velocityTimerPublish(const ros::TimerEvent& event);

  void setpointTimerPublish(const ros::TimerEvent& event);

  void setpointPublish(const geometry_msgs::PoseStamped& setpoint);
};
}
