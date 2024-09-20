#pragma once 
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include "mmdeploy/detector.hpp"

using det_result = mmdeploy::cxx::Result_<mmdeploy::cxx::Detection>;

class AIdetector{
public:
    AIdetector(){};
    ~AIdetector(){};
    AIdetector(std::string model_path){
        init(model_path);
    };
    bool init(std::string model_path);

    det_result run_inference(cv::Mat& src_image);
    bool run_inference_mask(cv::Mat& src_image, cv::Mat& output_image, 
    std::vector<std::vector<cv::Point>>& masks, std::vector<std::vector<cv::Point>>& masks_unripe);

private:
    std::shared_ptr<mmdeploy::Detector> detector;
};
