#include <dd_control/control.h>
#include <tf2/utils.h>

namespace dd_control
{
Control::Control(ros::NodeHandle& nh) : nh_(nh), mode_(0)
{
  collision_free_control_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>(
      "/collision_free_control", 1, &Control::collisionFreeControlCallback,
      this);
  setpoint_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
      "/setpoint", 1, &Control::setpointCallback, this);
  rc_sub_ = nh_.subscribe<mavros_msgs::RCIn>("/mavros/rc/in", 1,
                                             &Control::rcCallback, this);

  control_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(
      "/mavros/setpoint_velocity/cmd_vel", 1);
}

void Control::collisionFreeControlCallback(
    const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  if (mode_ == 0)
  {
    control_pub_.publish(msg);
  }
}

void Control::setpointCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  if (mode_ == 1)
  {
    geometry_msgs::TwistStamped out;
    out.header = msg->header;
    out.twist.linear.x = std::min(msg->pose.position.x, 0.25);
    out.twist.linear.x = std::max(out.twist.linear.x, -0.25);
    out.twist.linear.y = std::min(msg->pose.position.y, 0.25);
    out.twist.linear.y = std::max(out.twist.linear.y, -0.25);
    out.twist.linear.z = std::min(msg->pose.position.z, 0.25);
    out.twist.linear.z = std::max(out.twist.linear.z, -0.25);
    out.twist.angular.z = std::min(tf2::getYaw(msg->pose.orientation), 0.17);
    out.twist.angular.z = std::max(out.twist.angular.z, -0.17);

    control_pub_.publish(out);
  }
}

void Control::rcCallback(const mavros_msgs::RCIn::ConstPtr& msg)
{
  if (msg->channels[6] < 1100)
  {
    mode_ = 0;
  }
  else if (msg->channels[6] >= 1100 && msg->channels[6] <= 1900)
  {
    mode_ = 1;
  }
  else
  {
    mode_ = 2;
  }
}
}
