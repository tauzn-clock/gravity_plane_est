#include <opencv2/opencv.hpp>

void save_normal(std::vector<std::array<float, 3> > normal, int W, int H, std::string path){
    cv::Mat image(H,W,CV_8UC3, cv::Scalar(0,0,0));

    for (int i = 0; i < H; ++i) {
        for (int j = 0; j < W; ++j) {
            int index = i * W + j;

            int r = (int)((normal[index][0] + 1) / 2 * 255);
            int b = (int)((normal[index][1] + 1) / 2 * 255);
            int g = (int)((normal[index][2] + 1) / 2 * 255);

        
            image.at<cv::Vec3b>(i,j) = cv::Vec3b(b,g,r);  // Blue, Green, Red
        }
    }

    cv::imwrite(path,image);
}