#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <vector>
#include <yaml-cpp/yaml.h>

#include "utils/math_utils.cpp"
#include "utils/process_depthmsg.cpp"
#include "utils/get_normal.cpp"

YAML::Node config;
sensor_msgs::Imu imu;
sensor_msgs::CameraInfo camera_info;

bool imuReceived = false;
bool cameraInfoReceived = false;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    imu.angular_velocity.x = msg->angular_velocity.x;
    imu.angular_velocity.y = msg->angular_velocity.y;
    imu.angular_velocity.z = msg->angular_velocity.z;
    imu.linear_acceleration.x = msg->linear_acceleration.x;
    imu.linear_acceleration.y = msg->linear_acceleration.y;
    imu.linear_acceleration.z = msg->linear_acceleration.z;
    imu.orientation.x = msg->orientation.x;
    imu.orientation.y = msg->orientation.y;
    imu.orientation.z = msg->orientation.z;
    imu.orientation.w = msg->orientation.w;

    imuReceived = true;
}

void depthIntrinsicCallback(const sensor_msgs::CameraInfo::ConstPtr& msg){
    camera_info.header = msg->header;
    camera_info.width = msg->width;
    camera_info.height = msg->height;
    camera_info.distortion_model = msg->distortion_model;
    camera_info.D = msg->D;
    camera_info.K = msg->K;
    camera_info.R = msg->R;
    camera_info.P = msg->P;

    cameraInfoReceived = true;
}

void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg){

    if (!imuReceived || !cameraInfoReceived) {
        ROS_WARN("IMU or Camera Info not received yet.");
        return;
    }

    int W = msg->width;
    int H = msg->height;

    std::vector< std::array<float, 3> > points = depthmsg_to_3d(msg, camera_info, config["rescale_depth"].as<float>());

    std::array<float, 3> gravity_vector = {(float)imu.linear_acceleration.x, (float)imu.linear_acceleration.y, (float)imu.linear_acceleration.z};
    gravity_vector = normalise(gravity_vector);

    std::vector< std::array<float, 3> > img_normals = get_normal(points, W, H);
    
}

int main(int argc, char** argv)
{    
    // Initialize ROS
    ros::init(argc, argv, "find_planes");

    // Create a node handle
    ros::NodeHandle nh;

    std::string imu_topic, depth_img_topic, depth_intrinsic_topic;
    nh.getParam("imu_topic", imu_topic);
    nh.getParam("depth_img_topic", depth_img_topic);
    nh.getParam("depth_intrinsic_topic", depth_intrinsic_topic);

    std::string yaml_file_path;
    nh.getParam("yaml_file_path", yaml_file_path);
    std::cout << "YAML file path: " << yaml_file_path << std::endl;
    config = YAML::LoadFile(yaml_file_path);

    ros::Subscriber imu_sub = nh.subscribe(imu_topic, 100, imuCallback);
    ros::Subscriber depth_img_sub = nh.subscribe(depth_img_topic, 24, depthImageCallback);
    ros::Subscriber camera_info_sub = nh.subscribe(depth_intrinsic_topic, 1, depthIntrinsicCallback);

    // Spin to keep the node alive
    ros::spin();

    return 0;
}