#include <dd_control/control.h>
#include <tf2/utils.h>

namespace dd_control
{
Control::Control(ros::NodeHandle& nh) : nh_(nh), mode_(1)
{
  tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);

  robot_frame_ = "base_link";

  collision_free_control_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>(
      "/collision_free_control", 10, &Control::collisionFreeControlCallback,
      this);
  setpoint_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
      "/setpoint", 10, &Control::setpointCallback, this);
  rc_sub_ = nh_.subscribe<mavros_msgs::RCIn>("/mavros/rc/in", 10,
                                             &Control::rcCallback, this);

  control_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(
      "/mavros/setpoint_velocity/cmd_vel", 10);
}

void Control::collisionFreeControlCallback(
    const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  if (mode_ == 0)
  {
    control_pub_.publish(msg);
  }
}

void Control::setpointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  if (mode_ == 1)
  {
    geometry_msgs::TwistStamped out;
    try
    {
      geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
          robot_frame_, msg->header.frame_id, ros::Time(0));

      geometry_msgs::PoseStamped setpoint_transformed;
      tf2::doTransform(*msg, setpoint_transformed, transform);

      out.header = setpoint_transformed.header;

      out.twist.linear.x = std::min(setpoint_transformed.pose.position.x, 0.25);
      out.twist.linear.x = std::max(out.twist.linear.x, -0.25);
      out.twist.linear.y = std::min(setpoint_transformed.pose.position.y, 0.25);
      out.twist.linear.y = std::max(out.twist.linear.y, -0.25);
      out.twist.linear.z = std::min(setpoint_transformed.pose.position.z, 0.25);
      out.twist.linear.z = std::max(out.twist.linear.z, -0.25);
      out.twist.angular.z =
          std::min(tf2::getYaw(setpoint_transformed.pose.orientation), 0.17);
      out.twist.angular.z = std::max(out.twist.angular.z, -0.17);
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN_THROTTLE(1, "DD Control: %s", ex.what());

      out.header = msg->header;
    }

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
