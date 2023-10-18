#include "my_rb1_ros/Rotate.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "tf/transform_datatypes.h"

class RotateService {
private:
  ros::NodeHandle nh_;
  ros::ServiceServer service_;
  ros::Subscriber odom_sub_;

public:
  RotateService() {
    service_ = nh_.advertiseService("/rotate_robot",
                                    &RotateService::rotateCallback, this);
    odom_sub_ = nh_.subscribe("/odom", 1, &RotateService::odomCallback, this);
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

    // Construct a new quaternion for the target orientation
    tf::Quaternion target_quat = tf::createQuaternionFromRPY(0, 0, target_yaw);

    // Create an empty twist message to set the rotation
    geometry_msgs::Twist twist;
    twist.angular.z = 0.5; // Adjust the angular velocity as needed

    // Continue rotating until the target orientation is reached
    while (ros::ok() && std::abs(yaw - target_yaw) > 0.1) {
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
