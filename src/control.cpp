#include <dd_control/control.h>
#include <tf2/utils.h>

namespace dd_control
{
Control::Control(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
    : nh_(nh), nh_priv_(nh_priv), mode_(1)
{
  robot_frame_ = nh_priv_.param<std::string>("robot_frame", "base_link");
  std::string velocity_topic =
      nh_priv_.param<std::string>("velocity_topic", "/collision_free_control");
  std::string setpoint_topic =
      nh_priv_.param<std::string>("setpoint_topic", "/setpoint");
  std::string rc_topic =
      nh_priv_.param<std::string>("rc_topic", "/mavros/rc/in");
  std::string pose_topic =
      nh_priv_.param<std::string>("pose_topic", "/mavros/local_position/pose");
  std::string out_topic = nh_priv_.param<std::string>(
      "out_topic", "/mavros/setpoint_velocity/cmd_vel");

  double velocity_frequency = nh_priv_.param<double>("velocity_frequency", 10);
  double setpoint_frequency = nh_priv_.param<double>("setpoint_frequency", 10);

  tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);

  robot_frame_ = "base_link";

  velocity_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>(
      velocity_topic, 10, &Control::velocityCallback, this);
  setpoint_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
      setpoint_topic, 10, &Control::setpointCallback, this);
  cancel_sub_ = nh_.subscribe<std_msgs::Header>(setpoint_topic + "/cancel", 10,
                                                &Control::cancelCallback, this);
  rc_sub_ = nh_.subscribe<mavros_msgs::RCIn>(rc_topic, 10, &Control::rcCallback,
                                             this);
  pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
      pose_topic, 10, &Control::poseCallback, this);

  control_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(out_topic, 10);

  velocity_pub_timer_ =
      nh_.createTimer(ros::Rate(velocity_frequency),
                      &Control::velocityTimerPublish, this, false, false);
  setpoint_pub_timer_ =
      nh_.createTimer(ros::Rate(setpoint_frequency),
                      &Control::setpointTimerPublish, this, false, false);
}

void Control::velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  velocity_pub_timer_.stop();
  if (0 == mode_)
  {
    control_pub_.publish(msg);
  }
  velocity_pub_timer_.start();
}

void Control::setpointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  last_setpoint_ = *msg;

  setpoint_pub_timer_.stop();
  if (1 == mode_)
  {
    setpointPublish(last_setpoint_);
  }
  setpoint_pub_timer_.start();
}

void Control::cancelCallback(const std_msgs::Header::ConstPtr& msg)
{
  last_setpoint_ = current_pose_;
}

void Control::rcCallback(const mavros_msgs::RCIn::ConstPtr& msg)
{
  if (msg->channels[6] < 1100)
  {
    mode_ = 0;
    setpoint_pub_timer_.stop();
  }
  else if (msg->channels[6] >= 1100 && msg->channels[6] <= 1900)
  {
    mode_ = 1;
    velocity_pub_timer_.stop();
  }
  else
  {
    mode_ = 2;
    velocity_pub_timer_.stop();
    setpoint_pub_timer_.stop();
  }
}

void Control::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  current_pose_ = *msg;
}

void Control::velocityTimerPublish(const ros::TimerEvent& event)
{
  if (0 == mode_)
  {
    geometry_msgs::TwistStamped msg;
    msg.header.frame_id = robot_frame_;
    msg.header.stamp = ros::Time::now();
    control_pub_.publish(msg);
  }
}

void Control::setpointTimerPublish(const ros::TimerEvent& event)
{
  if (1 == mode_)
  {
    last_setpoint_.header.stamp = ros::Time::now();
    setpointPublish(last_setpoint_);
  }
}

void Control::setpointPublish(const geometry_msgs::PoseStamped& setpoint)
{
  geometry_msgs::TwistStamped out;
  try
  {
    geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
        robot_frame_, setpoint.header.frame_id, ros::Time(0));

    geometry_msgs::PoseStamped setpoint_transformed;
    tf2::doTransform(setpoint, setpoint_transformed, transform);

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

    out.header = setpoint.header;
  }

  control_pub_.publish(out);
}
}
