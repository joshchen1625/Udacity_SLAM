#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

// ROS::Pbulisher motor commands;
ros::Publisher motor_command_publisher;


// Callback function for drive_bot service command_robot
bool handle_drive_bot_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res)
{
  float linearX = (float)req.linear_x;
  float angularZ = (float)(float)req.angular_z;

  ROS_INFO("handle_drive_bot_request received - x linear velocity:%1.2f, z angular velocity:%1.2f", linearX, angularZ);

  // Publish linear x velocity and angular z velocity to the drive_bot

  // Create a motor_command object of type geometry_msgs::Twist
  geometry_msgs::Twist motor_command;

  motor_command.linear.x = linearX;
  motor_command.angular.z = angularZ;

  // Publish angles to drive the robot
  motor_command_publisher.publish(motor_command);

  // Return a response message
  res.msg_feedback = "Linear X velocity set: " + std::to_string(linearX) + " , angular Z velocity: " + std::to_string(angularZ);
  ROS_INFO_STREAM(res.msg_feedback);
  
  return true;
}


int main(int argc, char** argv)
{
  // Initialize the drive_bot node and create a handle to it
  ros::init(argc, argv, "drive_bot");
  ros::NodeHandle n;

  // Define the publisher to publish geometry_msgs::Twist messages on the cmd_vel topic with queue size 10
  motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  // Define a ball_chaser/command_robot service with a callback function handle_drive_bot_request
  ros::ServiceServer service = n.advertiseService("/drive_bot/command_robot", handle_drive_bot_request);
  ROS_INFO("Ready to send cmd_vel commands");

  // Handle ROS communication events
  ros::spin();

  return 0;
}
