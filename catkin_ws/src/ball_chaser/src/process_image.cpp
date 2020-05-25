#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
  ROS_INFO_STREAM("Going to drive the robot");

  // Request the linear x and angular z velocity
  ball_chaser::DriveToTarget srv;
  srv.request.linear_x = lin_x;
  srv.request.angular_z = ang_z;

  // Call the command_robot service and pass in the requested velocity
  if (!client.call(srv)){
    ROS_ERROR("Failed to call service command_robot");
  }
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
  int white_pixel = 255;
  int widthEachSection = img.step / 3;   // Used to determine which section (left, middle or right) the white ball is in
  int middleLine = img.step / 2;    // Used to determine the angular velocity Z
  int stepMod;
  int zVelScale;
  float lin_x = 0.2;
  float ang_z = 0.0;
  bool isGoingToMove = false;

  // Loop through each pixel in the image and check if there's a bright white pixel
  // Then, identify if this pixel falls in the left, mid or right side of the image
  // Depending on the white ball position, call the drive_bot function and pass velocities to it
  // Request a stop when there's no white ball seen by the camera
  for (int i = 0; i < img.height * img.step; i += 3){
    // ROS_INFO("Checking i: %d", i);
    if (img.data[i] == white_pixel && img.data[i + 1] == white_pixel && img.data[i + 2] == white_pixel){  // Each R-G-B value takes up one byte
      stepMod = i % img.step;
      ROS_INFO("stepMod is: %d", stepMod);
      
      // The method below doesn't work well
      /*
      zVelScale = stepMod - middleLine;
      ROS_INFO("middleLine, zVelScale: %d, %d", middleLine, zVelScale);

      ang_z = (-1) * ang_z * zVelScale;   // right-hand rule
      */

      if (stepMod < widthEachSection){
        ang_z = 0.3;  // On the left
      }
      else if (stepMod > 2 * widthEachSection && stepMod <= img.step){
        ang_z = -0.3;  // On the right
      }
      else{
        ang_z = 0.0;  // In the middle
      }
      
      isGoingToMove = true;
      ROS_INFO("lin_x, ang_z: %f1.2, %f1.2", lin_x, ang_z);
      break;
    }
  }

  if (isGoingToMove == true){
    drive_robot(lin_x, ang_z);
    isGoingToMove = false;   // Make sure to reset here
  }
  else{
    drive_robot(0.0, 0.0);
  }
}

int main(int argc, char** argv)
{
  // Initialize the process_image node and create a handle to it
  ros::init(argc, argv, "process_image");
  ros::NodeHandle n;

  // Define a client service capable of requesting services from command_robot
  client = n.serviceClient<ball_chaser::DriveToTarget>("/drive_bot/command_robot");

  // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
  ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

  // Handle ROS communication
  ros::spin();

  return 0;
}
