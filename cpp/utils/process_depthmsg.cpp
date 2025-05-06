#include <ros/ros.h>
#include <vector>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

std::vector<float> depthmsg_to_vector(const sensor_msgs::Image::ConstPtr& msg, float depth_scale) {
    int W = msg->width;
    int H = msg->height;

    std::vector<float> depth_data(W * H);

    if (msg->encoding == "16UC1") {
        for (int i = 0; i < W * H; i+=2) {
            depth_data[i] = (uint16_t(msg->data[i]) + (uint16_t(msg->data[i+1]) << 8)) / depth_scale;
        }
    }

    return depth_data;
}

std::vector< std::array<float, 3> > depthmsg_to_3d(const sensor_msgs::Image::ConstPtr& msg, const sensor_msgs::CameraInfo& camera_info, float depth_scale) {
    std::vector<float> depth_data = depthmsg_to_vector(msg, depth_scale);

    int W = msg->width;
    int H = msg->height;

    float fx = camera_info.K[0];
    float fy = camera_info.K[4];
    float cx = camera_info.K[2];
    float cy = camera_info.K[5];

    std::vector< std::array<float, 3> > points(depth_data.size());
    
    for (int i = 0; i < H; ++i) {
        for (int j = 0; j < W; ++j) {
            int index = i * W + j;
            float z = depth_data[index];
            if (z > 0) {
                points[index][0] = (j - cx) * z / fx;
                points[index][1] = (i - cy) * z / fy;
                points[index][2] = z;
            } 
        }
    }

    return points;
}