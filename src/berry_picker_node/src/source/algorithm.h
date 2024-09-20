#include <omp.h>

// OpenCV
#include <opencv2/opencv.hpp>

// PCL
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

int pctransform(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, Eigen::Matrix4f transform);

void fitEllipseToMask(const cv::Mat& mask, double& majorAxis, double& minorAxis, cv::Point& center, cv::Point& startPoint, cv::Point& endPoint, cv::Mat& outputImage);
void alignTemplateToImage(
    cv::Mat& templateMask,
    cv::Mat& targetMask,
    double& bestAngle,
    cv::Point& bestTranslation,
    cv::Mat& outputImage);