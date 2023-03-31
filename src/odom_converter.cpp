#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <tf2/utils.h>
#include <tsuten_msgs/Odometry.h>

namespace tsuten_real_robot
{
  class OdomConverter
  {
  public:
    OdomConverter()
    {
      ros::NodeHandle nh;

      odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 10);

      odom_raw_sub_ = nh.subscribe("odom_raw", 10, &OdomConverter::odomRawCallback, this);

      odom_.header.frame_id = "odom";
      odom_.child_frame_id = "base_link";
    }

  private:
    void odomRawCallback(const tsuten_msgs::Odometry &odom_raw)
    {
      odom_.header.stamp = ros::Time::now();

      odom_.pose.pose.position.x = odom_raw.x;
      odom_.pose.pose.position.y = odom_raw.y;

      tf2::Quaternion quat;
      quat.setRPY(0, 0, odom_raw.theta);
      odom_.pose.pose.orientation = tf2::toMsg(quat);

      odom_.twist.twist.linear.x = odom_raw.v_x;
      odom_.twist.twist.linear.y = odom_raw.v_y;
      odom_.twist.twist.angular.z = odom_raw.v_theta;

      odom_pub_.publish(odom_);
    }

    ros::Publisher odom_pub_;

    ros::Subscriber odom_raw_sub_;

    nav_msgs::Odometry odom_;
  };
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_converter");

  tsuten_real_robot::OdomConverter odom_converter;

  ros::spin();

  return 0;
}