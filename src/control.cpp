#include <dd_control/control.h>
#include <tf2/utils.h>

namespace dd_control
{
Control::Control(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
    : nh_(nh), nh_priv_(nh_priv), mode_(1)
{
  output_frame_ = nh_priv_.param<std::string>("output_frame", "map");
  std::string velocity_topic =
      nh_priv_.param<std::string>("velocity_topic", "/collision_free_control");
  std::string setpoint_topic =
      nh_priv_.param<std::string>("setpoint_topic", "/setpoint");
  std::string rc_topic =
      nh_priv_.param<std::string>("rc_topic", "/mavros/rc/in");
  std::string pose_topic =
      nh_priv_.param<std::string>("pose_topic", "/mavros/local_position/pose");
  std::string current_velocity_topic = nh_priv_.param<std::string>(
      "current_velocity_topic", "/mavros/local_position/velocity");
  std::string odom_topic =
      nh_priv_.param<std::string>("odom_topic", "/mavros/local_position/odom");
  std::string out_topic = nh_priv_.param<std::string>(
      "out_topic", "/mavros/setpoint_position/local");

  max_x_vel_ = nh_priv_.param<double>("max_x_vel", 1.0);
  max_y_vel_ = nh_priv_.param<double>("max_y_vel", 1.0);
  max_z_vel_ = nh_priv_.param<double>("max_z_vel", 1.0);

  k_p_x_ = nh_priv_.param<double>("k_p_x", 1.0);
  k_p_y_ = nh_priv_.param<double>("k_p_y", 1.0);
  k_p_z_ = nh_priv_.param<double>("k_p_z", 1.0);

  double frequency = nh_priv_.param<double>("frequency", 10);

  tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);

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
  current_velocity_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>(
      current_velocity_topic, 10, &Control::currentVelocityCallback, this);
  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(odom_topic, 10,
                                                &Control::odomCallback, this);

  control_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(out_topic, 10);

  pub_timer_ = nh_.createTimer(ros::Rate(frequency), &Control::timerPublish,
                               this, false, false);
}

void Control::velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  pub_timer_.stop();
  if (0 == mode_)
  {
    geometry_msgs::PoseStamped setpoint;
    setpoint.header = msg->header;

    // http://robotsforroboticists.com/pid-control/
    double desired_x_vel = clamp(msg->twist.linear.x, -max_x_vel_, max_x_vel_);
    double desired_y_vel = clamp(msg->twist.linear.y, -max_y_vel_, max_y_vel_);
    double desired_z_vel = clamp(msg->twist.linear.z, -max_z_vel_, max_z_vel_);

    double error_x_vel = desired_x_vel - current_local_velocity_.twist.linear.x;
    double error_y_vel = desired_y_vel - current_local_velocity_.twist.linear.y;
    double error_z_vel = desired_z_vel - current_local_velocity_.twist.linear.z;

    setpoint.pose.position.x = k_p_x_ * error_x_vel;
    setpoint.pose.position.y = k_p_y_ * error_y_vel;
    setpoint.pose.position.z = k_p_z_ * error_z_vel;

    tf2::Quaternion q;
    q.setRPY(msg->twist.angular.y, msg->twist.angular.x, msg->twist.angular.z);

    setpoint.pose.orientation.x = q.x();
    setpoint.pose.orientation.y = q.y();
    setpoint.pose.orientation.z = q.z();
    setpoint.pose.orientation.w = q.w();

    geometry_msgs::PoseStamped out;
    try
    {
      geometry_msgs::TransformStamped transform =
          tf_buffer_.lookupTransform(current_pose_.header.frame_id,
                                     setpoint.header.frame_id, ros::Time(0));

      tf2::doTransform(setpoint, out, transform);
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN_THROTTLE(1, "DD Control: %s", ex.what());

      out.header = setpoint.header;
      out.pose = current_pose_.pose;
    }

    control_pub_.publish(out);
  }
  pub_timer_.start();
}

void Control::setpointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  pub_timer_.stop();
  if (1 == mode_)
  {
    last_setpoint_ = *msg;

    setpointPublish(last_setpoint_);
  }
  pub_timer_.start();
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

void Control::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  current_pose_ = *msg;
}

void Control::currentVelocityCallback(
    const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  current_global_velocity_ = *msg;
}

void Control::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_pose_.header = msg->header;
  current_pose_.pose = msg->pose.pose;

  current_local_velocity_.header = msg->header;
  current_local_velocity_.header.frame_id = msg->child_frame_id;
  current_local_velocity_.twist = msg->twist.twist;
}

void Control::timerPublish(const ros::TimerEvent& event)
{
  last_setpoint_.header.stamp = ros::Time::now();
  setpointPublish(last_setpoint_);
}

void Control::setpointPublish(const geometry_msgs::PoseStamped& setpoint)
{
  geometry_msgs::PoseStamped out;
  try
  {
    geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
        current_pose_.header.frame_id, setpoint.header.frame_id, ros::Time(0));

    tf2::doTransform(setpoint, out, transform);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE(1, "DD Control: %s", ex.what());

    out.header = setpoint.header;
    out.pose = current_pose_.pose;
  }

  // http://robotsforroboticists.com/pid-control/
  double desired_x_vel =
      clamp(out.pose.position.x - current_pose_.pose.position.x, -max_x_vel_,
            max_x_vel_);
  double desired_y_vel =
      clamp(out.pose.position.y - current_pose_.pose.position.y, -max_y_vel_,
            max_y_vel_);
  double desired_z_vel =
      clamp(out.pose.position.z - current_pose_.pose.position.z, -max_z_vel_,
            max_z_vel_);

  double error_x_vel = desired_x_vel - current_global_velocity_.twist.linear.x;
  double error_y_vel = desired_y_vel - current_global_velocity_.twist.linear.y;
  double error_z_vel = desired_z_vel - current_global_velocity_.twist.linear.z;

  out.pose.position.x = current_pose_.pose.position.x + (k_p_x_ * error_x_vel);
  out.pose.position.y = current_pose_.pose.position.y + (k_p_y_ * error_y_vel);
  out.pose.position.z = current_pose_.pose.position.z + (k_p_z_ * error_z_vel);

  control_pub_.publish(out);
}

double Control::clamp(double value, double min, double max)
{
  return std::max(std::min(value, max), min);
}
}
