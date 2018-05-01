#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/RCIn.h>

namespace dd_control
{
class Control
{
private:
  ros::NodeHandle nh_;

  ros::Subscriber collision_free_control_sub_;
  ros::Subscriber raw_control_sub_;
  ros::Subscriber rc_sub_;

  ros::Publisher control_pub_;

  bool send_raw_control;

public:
  Control(ros::NodeHandle& nh);

private:
  void collisionFreeControlCallback(
      const geometry_msgs::TwistStamped::ConstPtr& msg);

  void rawControlCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);

  void rcCallback(const mavros_msgs::RCIn::ConstPtr& msg);
};
}
