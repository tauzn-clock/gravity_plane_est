#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    // Process the IMU data here
    ROS_INFO("IMU Data: Orientation: [%f, %f, %f, %f], Angular Velocity: [%f, %f, %f], Linear Acceleration: [%f, %f, %f]",
             msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w,
             msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
             msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
}

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "avg_imu");

    // Create a node handle
    ros::NodeHandle nh;

    ros::Subscriber camera_info_sub = nh.subscribe("/camera/depth/camera_info", 100, imuCallback);

    // Spin to keep the node alive
    ros::spin();

    return 0;
}