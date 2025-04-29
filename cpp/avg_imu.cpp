#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

sensor_msgs::Imu imu_msg;
float alpha;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    imu_msg.angular_velocity.x = (1-alpha) * imu_msg.angular_velocity.x + (alpha) * msg->angular_velocity.x;
    imu_msg.angular_velocity.y = (1-alpha) * imu_msg.angular_velocity.y + (alpha) * msg->angular_velocity.y;
    imu_msg.angular_velocity.z = (1-alpha) * imu_msg.angular_velocity.z + (alpha) * msg->angular_velocity.z;
    imu_msg.linear_acceleration.x = (1-alpha) * imu_msg.linear_acceleration.x + (alpha) * msg->linear_acceleration.x;
    imu_msg.linear_acceleration.y = (1-alpha) * imu_msg.linear_acceleration.y + (alpha) * msg->linear_acceleration.y;
    imu_msg.linear_acceleration.z = (1-alpha) * imu_msg.linear_acceleration.z + (alpha) * msg->linear_acceleration.z;

}

int main(int argc, char** argv)
{    
    imu_msg.angular_velocity.x = 0;
    imu_msg.angular_velocity.y = 0;
    imu_msg.angular_velocity.z = 0;
    imu_msg.linear_acceleration.x = 0;
    imu_msg.linear_acceleration.y = 0;
    imu_msg.linear_acceleration.z = 0;
    imu_msg.orientation.x = 0;

    // Initialize ROS
    ros::init(argc, argv, "avg_imu");

    // Create a node handle
    ros::NodeHandle nh;

    std::string imu_topic;
    nh.getParam("imu_topic", imu_topic);
    nh.getParam("alpha", alpha);

    std::cout<<imu_topic<<std::endl;

    ros::Subscriber camera_info_sub = nh.subscribe(imu_topic, 100, imuCallback);

    // Spin to keep the node alive
    ros::spin();

    return 0;
}