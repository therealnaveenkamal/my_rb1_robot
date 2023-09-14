#include "geometry_msgs/Twist.h"
#include "my_rb1_ros/Rotate.h"
#include "ros/init.h"
#include "ros/ros.h"
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

ros::Publisher pub;
double current_angular_z = 0.0;

bool rotateCallback(my_rb1_ros::Rotate::Request &req,
                    my_rb1_ros::Rotate::Response &res) {

  ROS_INFO("Service /rotate_robot has been called.");
  float required = req.degrees * (M_PI / 180);

  float actual = current_angular_z;

  float val = required + current_angular_z;

  if (val > M_PI) {
    val = val - (2 * M_PI);
  }

  geometry_msgs::Twist vel;

  ros::Rate loop_rate(2);

  while (ros::ok()) {

    /*ROS_INFO("Robot rotated by %f radians, Current Degrees: %f", required,
             current_angular_z * (180 / M_PI));*/

    if (fabs(val - current_angular_z) < 0.01) {
      vel.angular.z = 0;
      pub.publish(vel);
      ros::spinOnce();
      loop_rate.sleep();

      break;
    }

    vel.angular.z = -0.5 * (val - current_angular_z);

    /*
        if (fabs(current_angular_z - requiredDegree) < 0.01) {
          vel.angular.z = 0.2;
          pub.publish(vel);
          ros::spinOnce();
          loop_rate.sleep();
          break;
        }
        */

    pub.publish(vel);
    ros::spinOnce();
    loop_rate.sleep();
  }

  ROS_INFO(
      "Service execution completed. Current Degree: %f, Initial Degree: %f",
      current_angular_z * (180 / M_PI), actual * (180 / M_PI));
  res.result = "Rotation completed";
  return true;
}

/*
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  double yaw = atan2(
      2 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z +
           msg->pose.pose.orientation.x * msg->pose.pose.orientation.y),
      -1 + 2 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y +
                msg->pose.pose.orientation.z * msg->pose.pose.orientation.z));

  // yaw = yaw * (180.0 / M_PI);

  // ROS_INFO("Yaw Angle: %.2f", yaw);
  current_angular_z = yaw;
}*/

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
  double yaw = tf::getYaw(quat);

  // Yaw is now in radians.
  // ROS_INFO("Yaw Angle: %.2f", yaw);
  current_angular_z = yaw;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "rotate_server");
  ros::NodeHandle nh;

  pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  ros::ServiceServer my_service =
      nh.advertiseService("/rotate_robot", rotateCallback);

  ROS_INFO("Service /rotate_robot is now Ready!");

  ros::Subscriber sub = nh.subscribe("/odom", 1, odomCallback);

  ros::spin();

  return 0;
}
