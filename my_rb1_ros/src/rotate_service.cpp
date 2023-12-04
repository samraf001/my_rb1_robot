#include "my_rb1_ros/Rotate.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include <iostream>

class RotateService {
private:
  ros::NodeHandle nh_;
  ros::ServiceServer service_;
  ros::Subscriber odom_sub_;
  ros::Publisher twist_pub_; // Declare the twist publisher
  nav_msgs::Odometry
      last_odom_; // Declare the last_odom_ to store odometry data

public:
  RotateService() {
    service_ = nh_.advertiseService("/rotate_robot",
                                    &RotateService::rotateCallback, this);
    odom_sub_ = nh_.subscribe("/odom", 1, &RotateService::odomCallback, this);
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>(
        "/cmd_vel", 1); // Initialize the twist publisher
  }

  bool rotateCallback(my_rb1_ros::Rotate::Request &req,
                      my_rb1_ros::Rotate::Response &res) {

    // Calculate the current orientation from the odometry data
    tf::Quaternion quat;
    tf::quaternionMsgToTF(last_odom_.pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // Calculate the target orientation in radians
    double target_yaw = yaw + (req.degrees * M_PI / 180.0);

    // Create an empty twist message to set the rotation
    geometry_msgs::Twist twist;

    // Define a proportional gain
    double kp = .5;

    // rotating until the target orientation is reached
    while (ros::ok() && std::abs(yaw - target_yaw) > .001) {

      // Calculate the error between the current and target yaw
      double error = (target_yaw - yaw) * 0.5;

      if (std::fabs(twist.angular.z) > 0.05) {
        std::cout << twist.angular.z << std::endl;
        twist.angular.z = error;

      } else {
        // Calculate the angular velocity using the proportional controller
        twist.angular.z = kp * error;
        std::cout << twist.angular.z << std::endl;
      }

      // Publish the twist message to control the rotation
      twist_pub_.publish(twist);
      ros::spinOnce();

      // Update the current orientation from the odometry data
      tf::quaternionMsgToTF(last_odom_.pose.pose.orientation, quat);
      tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    }

    // Stop the robot
    twist.angular.z = 0.0;
    twist_pub_.publish(twist);

    res.result = "Rotation completed successfully.";
    return true;
  }

  void odomCallback(const nav_msgs::Odometry &odom_msg) {
    last_odom_ = odom_msg;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "rotate_service");
  RotateService rotate_service;
  ros::spin();
  return 0;
}
