#include <dd_control/control.h>

namespace dd_control
{
Control::Control(ros::NodeHandle& nh) : nh_(nh)
{
  send_raw_control = false;

  collision_free_control_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>(
      "/collision_free_control", 1, &Control::collisionFreeControlCallback,
      this);
  raw_control_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>(
      "/raw_control", 1, &Control::rawControlCallback, this);
  rc_sub_ = nh_.subscribe<mavros_msgs::RCIn>("/mavros/rc/in", 1,
                                             &Control::rcCallback, this);

  control_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(
      "/mavros/setpoint_velocity/cmd_vel", 1);
}

void Control::collisionFreeControlCallback(
    const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  if (!send_raw_control)
  {
    control_pub_.publish(msg);
  }
}

void Control::rawControlCallback(
    const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  if (send_raw_control)
  {
    geometry_msgs::TwistStamped out;
    out = *msg;
    //out.twist.linear.x =
    //out.twist.angular.z = std::min(out.twist.angular.z

    control_pub_.publish(out);
  }
}

void Control::rcCallback(const mavros_msgs::RCIn::ConstPtr& msg)
{
  send_raw_control = msg->channels[0] > 1400;
}
}
