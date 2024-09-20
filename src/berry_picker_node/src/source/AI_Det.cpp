#include "AI_Det.h"
#include <ctime>

using namespace std;
using namespace cv;

bool AIdetector::init(std::string model_path){
    const char* device_name = "cuda";
    int device_id = 0;
    mmdeploy::Model model(model_path);
    detector = make_shared<mmdeploy::Detector>(model, mmdeploy::Device{device_name, device_id});

    return true;
}

det_result AIdetector::run_inference(Mat& src_image){
    return detector->Apply(src_image);
}

bool AIdetector::run_inference_mask(Mat& src_image, Mat& output_image, vector<vector<Point>>& masks, std::vector<std::vector<cv::Point>>& masks_unripe){
    //init
    masks.clear();
    masks_unripe.clear();
    output_image = src_image.clone();
    
    //run AI
    auto dets = detector->Apply(src_image);

    for (int i = 0; i < dets.size(); ++i) {
        if (dets[i].score < 0.8) {
            continue;
        }
        mmdeploy_rect_t box = dets[i].bbox;
        int mask_w = dets[i].mask->width;
        int mask_h = dets[i].mask->height;

        //fprintf(stdout, "box %d, left=%.2f, top=%.2f, right=%.2f, bottom=%.2f, label=%d, score=%.4f\n",
        //       i, box.left, box.top, box.right, box.bottom, dets[i].label_id, dets[i].score);

        Vec3b mask_color;
        if(dets[i].label_id == 0)
            mask_color = Vec3b(255, 0, 0);
        if(dets[i].label_id == 1)
            mask_color = Vec3b(0, 255, 0);
        if(dets[i].label_id == 2)
            mask_color = Vec3b(0, 0, 255);
        if(dets[i].label_id == 3)
            mask_color = Vec3b(255, 255, 255);

        //only look for mask in bbox
        vector<Point> mask;
        for (int k = box.top; k < box.bottom; ++k) {
            for (int j = box.left; j < box.right; ++j) {
                int index = k * mask_w + j; 
                
                if (dets[i].mask->data[index]) {
                    mask.push_back(Point(j, k));
                    if(dets[i].label_id == 0)
                        output_image.at<Vec3b>(k, j) = output_image.at<Vec3b>(k, j) * 0.3 + mask_color * 0.7;
                    if(dets[i].label_id == 1)
                        output_image.at<Vec3b>(k, j) = output_image.at<Vec3b>(k, j) * 0.3 + mask_color * 0.7;
                    if(dets[i].label_id == 2)
                        output_image.at<Vec3b>(k, j) = output_image.at<Vec3b>(k, j) * 0.3 + mask_color * 0.7;
                    if(dets[i].label_id == 3)
                        output_image.at<Vec3b>(k, j) = output_image.at<Vec3b>(k, j) * 0.3 + mask_color * 0.7;
                }
            }
        }

        if(dets[i].label_id == 0)
            masks.push_back(mask);
        if(dets[i].label_id == 1)
            masks_unripe.push_back(mask);
            
        cv::rectangle(output_image, cv::Point{(int)box.left, (int)box.top},cv::Point{(int)box.right, (int)box.bottom}, cv::Scalar{0, 255, 0});
    }
    
    return true;
}

