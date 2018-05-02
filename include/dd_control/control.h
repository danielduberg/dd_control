#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/RCIn.h>

namespace dd_control
{
class Control
{
private:
  ros::NodeHandle nh_;

  ros::Subscriber collision_free_control_sub_;
  ros::Subscriber setpoint_sub_;
  ros::Subscriber rc_sub_;

  ros::Publisher control_pub_;

  int mode_;

public:
  Control(ros::NodeHandle& nh);

private:
  void collisionFreeControlCallback(
      const geometry_msgs::TwistStamped::ConstPtr& msg);

  void setpointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

  void rcCallback(const mavros_msgs::RCIn::ConstPtr& msg);
};
}
