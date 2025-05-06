#include <ros/ros.h>
#include <vector>

#include <sensor_msgs/Image.h>

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