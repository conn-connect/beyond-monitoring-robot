#pragma once
#include <cstdlib>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace cv;

class Sensor {
public:
    Sensor() {};
    ~Sensor() {};

    void start(std::string serial_num, int image_size = 0, bool isD405_ = false);
    void capture(cv::Mat& color, cv::Mat& depth);
    void capture_cloud(cv::Mat& color, cv::Mat& depth, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    void generate_pointcloud(cv::Mat color, cv::Mat depth, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
    void capture_cloud_average(cv::Mat& color, cv::Mat& depth, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

public:
    rs2_intrinsics intrinsic_color;
    rs2_intrinsics intrinsic_depth;
    bool isD405;

private:
    rs2::pipeline pipe;
};