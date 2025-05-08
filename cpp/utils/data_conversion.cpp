std::vector<float> depthmsg_to_vector(const sensor_msgs::Image::ConstPtr& msg, float depth_scale) {
    int W = msg->width;
    int H = msg->height;

    std::vector<float> depth_data(W * H);

    if (msg->encoding == "16UC1") {
        for (int i = 0; i < 2 * W * H; i+=2) {
            depth_data[i/2] = (uint16_t(msg->data[i]) + (uint16_t(msg->data[i+1]) << 8)) / depth_scale;
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
            points[index][0] = (j - cx) * z / fx;
            points[index][1] = (i - cy) * z / fy;
            points[index][2] = z;
        }
    }

    return points;
}

sensor_msgs::PointCloud2 create_masked_pcd(const std::vector< std::array<float, 3> >& points, const std::vector<int>& mask, int W, int H) {
    sensor_msgs::PointCloud2 point_cloud;
    point_cloud.header.frame_id = "map"; //TODO: change to camera frame
    point_cloud.header.stamp = ros::Time::now();
    point_cloud.height = H;
    point_cloud.width = W;
    point_cloud.is_dense = false;
    point_cloud.is_bigendian = false;

    sensor_msgs::PointField x_field, y_field, z_field, rgb_field;

    x_field.name = "x";
    x_field.offset = 0;
    x_field.datatype = sensor_msgs::PointField::FLOAT32;
    x_field.count = 1;

    y_field.name = "y";
    y_field.offset = 4;
    y_field.datatype = sensor_msgs::PointField::FLOAT32;
    y_field.count = 1;

    z_field.name = "z";
    z_field.offset = 8;
    z_field.datatype = sensor_msgs::PointField::FLOAT32;
    z_field.count = 1;

    rgb_field.name = "rgb";
    rgb_field.offset = 12;
    rgb_field.datatype = sensor_msgs::PointField::FLOAT32;
    rgb_field.count = 1;

    point_cloud.fields = {x_field, y_field, z_field, rgb_field};
    point_cloud.point_step = 16; // 4 bytes for each field
    point_cloud.row_step = point_cloud.point_step * W;
    point_cloud.data.resize(point_cloud.row_step * H);
    
    for (size_t i = 0; i < points.size(); ++i) {
        if (points[i][2] != 0) {
            float* data_ptr = reinterpret_cast<float*>(&point_cloud.data[i * point_cloud.point_step]);
            data_ptr[0] = points[i][0];
            data_ptr[1] = points[i][1];
            data_ptr[2] = points[i][2];

            uint32_t rgb;
            if (mask[i] == 0) {
                // Set color to white
                rgb = (255 << 16) | (255 << 8) | 255; // White color
            } else {
                std::array<int,3> color = hsv(mask[i], 4);
                rgb = (color[2] << 16) | (color[1] << 8) | color[0]; // RGB color
            }
            std::memcpy(data_ptr + 3, &rgb, sizeof(uint32_t));
        }
    }

    return point_cloud;
}