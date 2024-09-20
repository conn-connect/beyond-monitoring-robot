// berry_picker_node.cpp

#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#include <omp.h>

// ROS 2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// OpenCV
#include <opencv2/opencv.hpp>

// PCL
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// AI
#include "AI_Det.h"

// Calibration
#include "constants.h"

// Robot
#include "ARM.h"

// RealSense
#include "sensor.h"

// Algorithm
#include "algorithm.h"

using namespace std::chrono_literals;

class BerryPickerNode : public rclcpp::Node
{
public:
    BerryPickerNode() : Node("berry_picker_node")
    {
        // Initialization code
        RCLCPP_INFO(this->get_logger(), "Berry Picker Node Initialized");

        // Initialize subscribers and publishers
        start_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "picking_command", 10, std::bind(&BerryPickerNode::startCallback, this, std::placeholders::_1));

        status_publisher_ = this->create_publisher<std_msgs::msg::String>("picking_status", 10);

        // Initialize variables
        picking_in_progress_ = false;
        
        // Add a timer to run your main picking loop
        timer_ = this->create_wall_timer(
            500ms, std::bind(&BerryPickerNode::processPicking, this));
    }

private:
    void startCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (msg->data == "start")
        {
            RCLCPP_INFO(this->get_logger(), "Received start signal");
            start_picking_ = true;
        }
    }

    void processPicking()
    {
        if (!start_picking_)
        {
            return;
        }

        // Your main picking logic goes here
        // Replace std::cout with RCLCPP_INFO for logging
        RCLCPP_INFO(this->get_logger(), "Processing picking...");
        
        int aaa;
        // Pre-define variables
        //  Robot work volume
        float min_x = 200;
        float max_x = 470;
        float min_y = -220;
        float max_y = 10;
        float min_z = 170;
        float max_z = 480;

        float approach_dist = 60; // mm
        float target_z_offset = 40; // mm
        float robot_vel_ratio = 1; // Robot velocity
        float robot_acc_ratio = 1; // Robot acceleration

        int cnt = 0;
        int cnt_max = 5;
        int retry_count = 0;
        int retry_max = 7;
        std::vector<float> yaw_offset_list = { 0, -15, 15, -5, 5, -20, 20, -10, 10, -30, 30 };

        ///////////////////////////////////////////////
        // Initialze

        // Realsense
        Sensor realsense;
        realsense.start("128422271578", 0, true);
        // realsense.start("243322074400", false);

        // Robot arm
        ARM arm("192.168.1.151");
        int a = arm.connect();

        // AI
        AIdetector det;
        det.init("/home/chanwoo/strawberry_0710");

        ///////////////////////////////////////////////
        // calibration
        // calibrateV2R_lite6(arm, realsense, true);
        // TCP calibration needs to be fixed!!!!!
        // calibrateTCP(arm);
        // teachingMode(arm);

        ///////////////////////////////////////////////

        // Calibration result loading
        tfmat tf_f2c;
        loadCalibrationResult(tf_f2c, "/home/chanwoo/berry_data/calibration/v2r_cal.txt");
        cout << "Calibration result flange 2 camera : " << tf_f2c << endl;

        // TCP calibration needs to be fixed!!!!!
        tfmat tf_f2tcp = get_f2tcp();
        // loadTCPCalibrationResult(tf_f2tcp, "/home/chanwoo/berry_data/calibration/tcp_calibration_result.txt");
        // cout << "Calibration result flange 2 tcp : " << tf_f2tcp << endl;

        // This is how you get Base to Camera transformation
        tfmat tf_b2c;
        tfmat tf_curr_flange;
        arm.getTransformationFlange(tf_curr_flange);
        tf_b2c = tf_f2c.Inversed() * tf_curr_flange.Inversed();

        std::vector<Eigen::VectorXf> teaching_joints;
        loadTeachingJoints(teaching_joints, "/home/chanwoo/berry_data/teaching/joints.txt");
        if (teaching_joints.size() < 5) { // 필요한 크기에 따라 조정
            std::cerr << "teaching_joints의 크기가 충분하지 않습니다 : " << teaching_joints.size() << std::endl;
            return -1;
        }

        // arm.moveJoint(teaching_joints[0], 50, 50, 0, true);

        // Initial pose
        tfmat tf_init_pose = xyzrpy_to_tfmat(300, -90, 380, 0, 0, 0);
        arm.moveTransformationTCP(tf_init_pose, get_f2tcp(), 300 * robot_vel_ratio, 100 * robot_acc_ratio, 0, true, true);

        std::vector<Eigen::VectorXf> teaching_xyzrpys;
        Eigen::VectorXf slot_target_xyzrpy;
        arm.getForwardKinematcis(teaching_joints[2], slot_target_xyzrpy);
        // slot #0
        teaching_xyzrpys.push_back(slot_target_xyzrpy);
        // slot #1
        slot_target_xyzrpy[1] += 45;
        teaching_xyzrpys.push_back(slot_target_xyzrpy);
        // slot #2
        slot_target_xyzrpy[1] += 45;
        teaching_xyzrpys.push_back(slot_target_xyzrpy);
        // slot #3
        slot_target_xyzrpy[1] += 45;
        teaching_xyzrpys.push_back(slot_target_xyzrpy);
        // slot #4
        slot_target_xyzrpy[1] += 45;
        teaching_xyzrpys.push_back(slot_target_xyzrpy);

        arm.getForwardKinematcis(teaching_joints[4], slot_target_xyzrpy);
        // slot #5
        slot_target_xyzrpy[1] += 45;
        teaching_xyzrpys.push_back(slot_target_xyzrpy);
        // slot #6
        slot_target_xyzrpy[1] += 45;
        teaching_xyzrpys.push_back(slot_target_xyzrpy);
        // slot #7
        slot_target_xyzrpy[1] += 45;
        teaching_xyzrpys.push_back(slot_target_xyzrpy);
        // slot #8
        slot_target_xyzrpy[1] += 45;
        teaching_xyzrpys.push_back(slot_target_xyzrpy);
        // slot #9
        slot_target_xyzrpy[1] += 45;
        teaching_xyzrpys.push_back(slot_target_xyzrpy);

        ///////////////////////////////////////////////
        if (0) {

            // Initial pose
            tfmat tf_init_pose = xyzrpy_to_tfmat(300, -90, 380, 0, 0, 0);
            cout << "Initial pose : " << tf_init_pose << endl;
            arm.moveTransformationTCP(tf_init_pose, get_f2tcp(), 300, 30000, 0, true, true);

            tf_init_pose = xyzrpy_to_tfmat(300, -90, 380, 15, 0, 0);
            arm.moveTransformationTCP(tf_init_pose, get_f2tcp(), 300, 30000, 0, true, true);

            tf_init_pose = xyzrpy_to_tfmat(300, -90, 380, -15, 0, 0);
            arm.moveTransformationTCP(tf_init_pose, get_f2tcp(), 300, 30000, 0, true, true);

            tf_init_pose = xyzrpy_to_tfmat(300, -90, 380, 0, 0, 0);
            arm.moveTransformationTCP(tf_init_pose, get_f2tcp(), 300, 30000, 0, true, true);

            tfmat current_pose;
            arm.getTransformationTCP(current_pose, get_f2tcp());
            cout << "Current pose : " << current_pose << endl;
            Eigen::VectorXf pose = tfmat_to_xyzrpy(current_pose);
            cout << "Current pose (RPYXYZ) : " << pose << endl;

            pose[3] = 0;
            pose[4] = 0;
            pose[5] = 30; // yaw
            pose[1] -= 30; // y
            current_pose = xyzrpy_to_tfmat(pose);
            arm.moveTransformationTCP(current_pose, get_f2tcp(), 300, 30000, 0, true, true);

            pose[5] = -30;
            pose[1] += 60;
            current_pose = xyzrpy_to_tfmat(pose);
            arm.moveTransformationTCP(current_pose, get_f2tcp(), 300, 30000, 0, true, true);

            pose[5] = 0;
            pose[1] -= 60;
            current_pose = xyzrpy_to_tfmat(pose);
            arm.moveTransformationTCP(current_pose, get_f2tcp(), 300, 30000, 0, true, true);
        }
        ///////////////////////////////////////////////
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ws(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cog(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pick_check_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        auto start = std::chrono::high_resolution_clock::now();
        auto start2 = std::chrono::high_resolution_clock::now();
        auto mid = std::chrono::high_resolution_clock::now();

        std::string timestamp; // Save file name

        cv::Mat mask_image, color, depth;
        std::vector<std::vector<Point>> masks, masks_unripe;

        tfmat tf_target_pose; // Robot target pose
        pcl::PointXYZRGB target_point; // Robot target pose

        std::vector<pcl::PointXYZRGB> v_cog; // Target point cloud center of gravity

        Eigen::Matrix4f t_b2c; // Base to Camera transformation matrix

        while (cnt < cnt_max) {
            cloud->clear();
            cloud_ws->clear();
            cloud_cog->clear();
            cloud_target->clear();
            pick_check_cloud->clear();

            if (retry_count > 10) {
                std::cout << "Failed cycle" << std::endl;
                break;
            }

            std::cout << "-----------------------------------------------------" << std::endl;

            if (retry_count == 0) {
                tf_init_pose = xyzrpy_to_tfmat(300, -90, 380, 0, 0, 0);
                arm.moveTransformationTCP(tf_init_pose, get_f2tcp(), 500 * robot_vel_ratio, 30000 * robot_acc_ratio, 0, true, true);

                // if (arm.getState() == 1) {
                //     std::cout << "[PROC] Waiting robot to stop" << std::endl;
                //     continue;
                // }

                timestamp = getCurrentDateTime();

                start2 = std::chrono::high_resolution_clock::now();

                // How to get current camera position
                tf_curr_flange;
                arm.getTransformationFlange(tf_curr_flange);
                tf_b2c = tf_f2c.Inversed() * tf_curr_flange.Inversed();
                t_b2c = tf_b2c.toEigenMatrix();

                realsense.capture(color, depth);

                mid = std::chrono::high_resolution_clock::now();
                std::cout << "[PROC] Capture : " << (float)std::chrono::duration_cast<std::chrono::milliseconds>(mid - start2).count() / 1000 << " seconds." << std::endl;
                start2 = std::chrono::high_resolution_clock::now();

                ///////////////////////////////////////////////

                det.run_inference_mask(color, mask_image, masks, masks_unripe);

                if (DEBUG)
                    cv::imwrite("/home/chanwoo/berry_data/debug_data/" + timestamp + "_0_raw.png", color);
                if (DEBUG)
                    cv::imwrite("/home/chanwoo/berry_data/debug_data/" + timestamp + "_1_mask.png", mask_image);

                if (DEBUG2) {
                    cv::imshow("Image", mask_image);
                    cv::waitKey(0);
                }

                mid = std::chrono::high_resolution_clock::now();
                std::cout << "[PROC] Inference : " << (float)std::chrono::duration_cast<std::chrono::milliseconds>(mid - start2).count() / 1000 << " seconds." << std::endl;
                start2 = std::chrono::high_resolution_clock::now();

                // If AI result is empty, skip the process
                if (masks.size() == 0)
                    continue;

                // Else continue the process
                realsense.generate_pointcloud(color, depth, cloud);
                pctransform(cloud, t_b2c.inverse());
                mid = std::chrono::high_resolution_clock::now();
                std::cout << "[PROC] Generate Pointcloud : " << (float)std::chrono::duration_cast<std::chrono::milliseconds>(mid - start2).count() / 1000 << " seconds." << std::endl;
                start2 = std::chrono::high_resolution_clock::now();

                if (DEBUG)
                    pcl::io::savePLYFileBinary("/home/chanwoo/berry_data/debug_data/" + timestamp + "_2_raw.ply", *cloud);

                v_cog.resize(masks.size());

                for (int idx = 0; idx < masks.size(); idx++) {
                    v_cog[idx].x = 0;
                    v_cog[idx].y = 0;
                    v_cog[idx].z = 0;
                    v_cog[idx].r = 0;
                    v_cog[idx].g = 255;
                    v_cog[idx].b = 0;
                    int valid_counter = 0;
                    for (int i = 0; i < masks[idx].size(); i++) {
                        int x = masks[idx][i].x;
                        int y = masks[idx][i].y;
                        int index = y * color.cols + x;

                        if (cloud->points[index].x == 0 && cloud->points[index].y == 0 && cloud->points[index].z == 0)
                            continue;
                        if (std::isnan(v_cog[idx].x) || std::isnan(v_cog[idx].y) || std::isnan(v_cog[idx].z)) {
                            std::cerr << "Warning: Invalid cog values at index " << idx << std::endl;
                            continue;
                        }

                        v_cog[idx].x += cloud->points[index].x;
                        v_cog[idx].y += cloud->points[index].y;
                        v_cog[idx].z += cloud->points[index].z;
                        valid_counter++;
                    }
                    if (valid_counter == 0)
                        continue;

                    v_cog[idx].x /= valid_counter;
                    v_cog[idx].y /= valid_counter;
                    v_cog[idx].z /= valid_counter;
                }

                if (masks.size() > 1) {
                    for (int i = 0; i < masks.size(); i++) {
                        for (int j = i + 1; j < masks.size(); j++) {
                            if (v_cog[i].x > v_cog[j].x) {
                                std::swap(v_cog[i], v_cog[j]);
                                std::swap(masks[i], masks[j]);
                            }
                        }
                    }
                }
                cloud_target->clear();
                for (int idx = 0; idx < masks.size(); idx++) {
                    // cloud_cog->push_back(v_cog[idx]);
                    if (idx == 0) {
                        for (int i = 0; i < masks[idx].size(); i++) {
                            int x = masks[idx][i].x;
                            int y = masks[idx][i].y;
                            int index = y * color.cols + x;
                            if (index >= cloud->points.size()) {
                                std::cout << "Index is out of boundary Error!" << std::endl;
                            }
                            if (index < cloud->points.size()) {
                                cloud->points[index].r = 255;
                                cloud->points[index].g = 0;
                                cloud->points[index].b = 0;
                                if (cloud->points[index].x != 0 && cloud->points[index].y != 0 && cloud->points[index].z != 0)
                                    cloud_target->push_back(cloud->points[index]);
                            }
                        }
                    } else {
                        for (int i = 0; i < masks[idx].size(); i++) {
                            int x = masks[idx][i].x;
                            int y = masks[idx][i].y;
                            int index = y * color.cols + x;
                            if (index >= cloud->points.size()) {
                                std::cout << "Index is out of boundary Error!" << std::endl;
                            }
                            if (index < cloud->points.size()) {
                                cloud->points[index].r = 0;
                                cloud->points[index].g = 0;
                                cloud->points[index].b = 255;
                            }
                        }
                    }
                }

                for (int i = 0; i < cloud->points.size(); i++) {
                    if (cloud->points[i].x > min_x && cloud->points[i].x < max_x && cloud->points[i].y > min_y && cloud->points[i].y < max_y && cloud->points[i].z > min_z && cloud->points[i].z < max_z) {
                        cloud_ws->points.push_back(cloud->points[i]);
                    }
                }

                if (DEBUG)
                    pcl::io::savePLYFileBinary("/home/chanwoo/berry_data/debug_data/" + timestamp + "_3_found.ply", *cloud_ws);

                mid = std::chrono::high_resolution_clock::now();
                std::cout << "[PROC] Post processing : " << (float)std::chrono::duration_cast<std::chrono::milliseconds>(mid - start2).count() / 1000 << " seconds." << std::endl;
                start2 = std::chrono::high_resolution_clock::now();

                ///////////////////////////////////////////////
                // Move close to COG of strawberry

                target_point.x = v_cog[0].x;
                target_point.y = v_cog[0].y;
                target_point.z = v_cog[0].z;
                target_point.r = 255;
                target_point.g = 255;
                target_point.b = 255;
                cloud_cog->push_back(target_point);
            }
            // approach
            arm.open_gripper();

            tf_target_pose = xyzrpy_to_tfmat(target_point.x, target_point.y, target_point.z, 0, 0, yaw_offset_list[retry_count]);
            tfmat tf_f2tcp_temp = get_f2tcp();
            tf_f2tcp_temp.trans_vec(1) += approach_dist;
            tf_f2tcp_temp.trans_vec(2) += 40;
            arm.moveTransformationTCP(tf_target_pose, tf_f2tcp_temp, 500 * robot_vel_ratio, 30000 * robot_acc_ratio, 0, true, true);

            ///////////////////////////////////////////////
            // Try capture closer
            ///////////////////////////////////////////////
            // sleep_milliseconds(500);

            // How to get current camera position
            arm.getTransformationFlange(tf_curr_flange);
            tf_b2c = tf_f2c.Inversed() * tf_curr_flange.Inversed();
            t_b2c = tf_b2c.toEigenMatrix();

            realsense.capture(color, depth);
            ///////////////////////////////////////////////
            masks.clear();
            masks_unripe.clear();
            det.run_inference_mask(color, mask_image, masks, masks_unripe);

            if (DEBUG)
                cv::imwrite("/home/chanwoo/berry_data/debug_data/" + timestamp + "_4_raw.png", color);
            if (DEBUG)
                cv::imwrite("/home/chanwoo/berry_data/debug_data/" + timestamp + "_5_mask.png", mask_image);

            if (DEBUG2) {
                cv::imshow("Image", mask_image);
                cv::waitKey(0);
            }

            mid = std::chrono::high_resolution_clock::now();
            std::cout << "[PROC] Inference2 : " << (float)std::chrono::duration_cast<std::chrono::milliseconds>(mid - start2).count() / 1000 << " seconds." << std::endl;
            start2 = std::chrono::high_resolution_clock::now();

            // If AI result is empty, skip the process
            if (masks.size() == 0) {
                retry_count++;
                std::cout << "Failed to detect Strawberry, Retry! : " << retry_count << std::endl;
                continue;
            }

            // Else continue the process
            cloud->clear();
            realsense.generate_pointcloud(color, depth, cloud);
            pctransform(cloud, t_b2c.inverse());
            mid = std::chrono::high_resolution_clock::now();
            std::cout << "[PROC] Generate Pointcloud : " << (float)std::chrono::duration_cast<std::chrono::milliseconds>(mid - start2).count() / 1000 << " seconds." << std::endl;
            start2 = std::chrono::high_resolution_clock::now();

            v_cog.clear();
            v_cog.resize(masks.size());

            std::vector<cv::Point> v_mask_center(masks.size());

            for (int idx = 0; idx < masks.size(); idx++) {
                v_cog[idx].x = 0;
                v_cog[idx].y = 0;
                v_cog[idx].z = 0;
                v_cog[idx].r = 0;
                v_cog[idx].g = 255;
                v_cog[idx].b = 0;

                cv::Point mask_center;
                mask_center.x = mask_center.y = 0;
                int valid_counter = 0;
                for (int i = 0; i < masks[idx].size(); i++) {
                    int x = masks[idx][i].x;
                    int y = masks[idx][i].y;
                    int index = y * color.cols + x;

                    if (cloud->points[index].x == 0 && cloud->points[index].y == 0 && cloud->points[index].z == 0)
                        continue;
                    if (std::isnan(v_cog[idx].x) || std::isnan(v_cog[idx].y) || std::isnan(v_cog[idx].z)) {
                        std::cerr << "Warning: Invalid cog values at index " << idx << std::endl;
                        continue;
                    }
                    mask_center.x += x;
                    mask_center.y += y;

                    v_cog[idx].x += cloud->points[index].x;
                    v_cog[idx].y += cloud->points[index].y;
                    v_cog[idx].z += cloud->points[index].z;
                    valid_counter++;
                }
                if (valid_counter == 0)
                    continue;

                mask_center.x /= valid_counter;
                mask_center.y /= valid_counter;
                v_mask_center[idx] = mask_center;
                v_cog[idx].x /= valid_counter;
                v_cog[idx].y /= valid_counter;
                v_cog[idx].z /= valid_counter;
            }
            // Sort by cog x coordinate
            //  if (masks.size() > 1) {
            //      for (int i = 0; i < masks.size(); i++) {
            //          for (int j = i + 1; j < masks.size(); j++) {
            //              if (v_cog[i].x > v_cog[j].x) {
            //                  std::swap(v_cog[i], v_cog[j]);
            //                  std::swap(masks[i], masks[j]);
            //              }
            //          }
            //      }
            //  }

            // Sort by how close is the mask center to the image center
            int target_mask_id = 0;
            if (masks.size() > 1) {
                float min_dist = 9999999;
                for (int i = 0; i < masks.size(); i++) {
                    float dist = sqrt(std::pow(color.cols / 2 - v_mask_center[i].x, 2) + std::pow(color.rows / 2 - v_mask_center[i].y, 2));
                    if (dist < min_dist) {
                        min_dist = dist;
                        target_mask_id = i;
                    }
                }
            }

            std::cout << "[DEBUG] Target mask id : " << target_mask_id << std::endl;
            cv::Mat target_image = color.clone();
            for (int i = 0; i < masks[target_mask_id].size(); i++) {
                int x, y;
                x = masks[target_mask_id][i].x;
                y = masks[target_mask_id][i].y;
                target_image.at<cv::Vec3b>(y, x)[0] = 0;
                target_image.at<cv::Vec3b>(y, x)[1] = 0;
                target_image.at<cv::Vec3b>(y, x)[2] = 255;
            }
            if (DEBUG)
                cv::imwrite("/home/chanwoo/berry_data/debug_data/" + timestamp + "_5-1_Target.png", target_image);
            if (DEBUG2) {
                cv::imshow("Image", target_image);
                cv::waitKey(0);
            }

            cloud_target->clear();
            for (int idx = 0; idx < masks.size(); idx++) {
                // cloud_cog->push_back(v_cog[idx]);
                if (idx == target_mask_id) {
                    for (int i = 0; i < masks[idx].size(); i++) {
                        int x = masks[idx][i].x;
                        int y = masks[idx][i].y;
                        int index = y * color.cols + x;
                        if (index >= cloud->points.size()) {
                            std::cout << "Index is out of boundary Error!" << std::endl;
                        }
                        if (index < cloud->points.size()) {
                            cloud->points[index].r = 255;
                            cloud->points[index].g = 0;
                            cloud->points[index].b = 0;
                            if (cloud->points[index].x != 0 && cloud->points[index].y != 0 && cloud->points[index].z != 0)
                                cloud_target->push_back(cloud->points[index]);
                        }
                    }
                } else {
                    for (int i = 0; i < masks[idx].size(); i++) {
                        int x = masks[idx][i].x;
                        int y = masks[idx][i].y;
                        int index = y * color.cols + x;
                        if (index >= cloud->points.size()) {
                            std::cout << "Index is out of boundary Error!" << std::endl;
                        }
                        if (index < cloud->points.size()) {
                            cloud->points[index].r = 0;
                            cloud->points[index].g = 0;
                            cloud->points[index].b = 255;
                        }
                    }
                }
            }

            pcl::PointXYZRGB target_point2;
            target_point2.x = v_cog[target_mask_id].x;
            target_point2.y = v_cog[target_mask_id].y;
            target_point2.z = v_cog[target_mask_id].z;
            // float distance = sqrt(pow(target_point2.x - target_point.x, 2) + pow(target_point2.y - target_point.y, 2) + pow(target_point2.z - target_point.z, 2));
            // std::cout << "Distance between two target cog points : " << distance << " mm" << std::endl;
            // if (distance > 10) {
            //     retry_count++;
            //     std::cout << "Failed to find matching target point, Retry! : " << retry_count << std::endl;
            //     continue;
            // }

            if (DEBUG)
                pcl::io::savePLYFileBinary("/home/chanwoo/berry_data/debug_data/" + timestamp + "_6_found.ply", *cloud);

            mid = std::chrono::high_resolution_clock::now();
            std::cout << "[PROC] Post processing : " << (float)std::chrono::duration_cast<std::chrono::milliseconds>(mid - start2).count() / 1000 << " seconds." << std::endl;
            start2 = std::chrono::high_resolution_clock::now();

            ///////////////////////////////////////////////
            // Find stem of strawberry

            cv::Mat target_mask_image = cv::Mat::zeros(color.size(), CV_8UC1);
            for (int i = 0; i < masks[target_mask_id].size(); i++) {
                int x = masks[target_mask_id][i].x;
                int y = masks[target_mask_id][i].y;
                if (x > 0 && x < target_mask_image.cols && y > 0 && y < target_mask_image.rows) {
                    target_mask_image.at<uchar>(y, x) = 255;
                }
            }
            double majorAxis, minorAxis;
            cv::Mat output_image;
            if (DEBUG2) {
                cv::imshow("Image", target_mask_image);
                cv::waitKey(0);
            }

            float target_point_roll = 0;
            cv::Point startPoint, endPoint, centerPoint;

            // Do ellipse fitting
            if (0) {
                fitEllipseToMask(target_mask_image, majorAxis, minorAxis, centerPoint, startPoint, endPoint, output_image);
                // cv::circle(color, centerPoint, 5, cv::Scalar(0, 255, 0), -1);
                cv::circle(color, startPoint, 5, cv::Scalar(0, 0, 255), -1);
                cv::circle(color, endPoint, 5, cv::Scalar(255, 0, 0), -1);

                int center_idx = centerPoint.y * color.cols + centerPoint.x;
                target_point.x = 0;
                target_point.y = 0;
                target_point.z = 0;
                int num_points = 0;
                for (int i = -3; i < 4; i++) {
                    for (int j = -3; j < 4; j++) {
                        int x = centerPoint.x + j;
                        int y = centerPoint.y + i;

                        if (x < 0 || x >= color.cols || y < 0 || y >= color.rows) {
                            continue; // 이미지 범위를 벗어나면 건너뜁니다.
                        }

                        int idx = y * color.cols + x;

                        if (idx < 0 || idx >= cloud->points.size()) {
                            continue; // 포인트 클라우드 범위를 벗어나면 건너뜁니다.
                        }

                        target_point.x += cloud->points[idx].x;
                        target_point.y += cloud->points[idx].y;
                        target_point.z += cloud->points[idx].z;
                        num_points++;

                        // 이미지 픽셀 색상 변경
                        color.at<cv::Vec3b>(y, x)[0] = 255;
                        color.at<cv::Vec3b>(y, x)[1] = 255;
                        color.at<cv::Vec3b>(y, x)[2] = 255;
                    }
                }

                if (DEBUG2) {
                    cv::imshow("Image", color);
                    cv::waitKey(0);
                }

                if (num_points == 0) {
                    std::cout << "Failed to find target point, Retry!" << std::endl;
                    continue;
                }

                target_point.x /= num_points;
                target_point.y /= num_points;
                target_point.z /= num_points;
                target_point.r = 0;
                target_point.g = 0;
                target_point.b = 255;
                cloud_cog->push_back(target_point);
                target_point_roll = -90 + atan2(endPoint.y - startPoint.y, endPoint.x - startPoint.x) * 180 / M_PI;
                std::cout << "Target point angle : " << target_point_roll << " degrees" << std::endl;
            }

            // Do Mask fitting
            if (1) {
                cv::Mat berry_mask_image = cv::imread("/home/chanwoo/berry_data/test.png");
                for (int i = 0; i < berry_mask_image.rows; i++) {
                    for (int j = 0; j < berry_mask_image.cols; j++) {
                        if (berry_mask_image.at<cv::Vec3b>(i, j)[0] == 0 && berry_mask_image.at<cv::Vec3b>(i, j)[1] == 0 && berry_mask_image.at<cv::Vec3b>(i, j)[2] == 0) {
                            berry_mask_image.at<cv::Vec3b>(i, j)[0] = 0;
                            berry_mask_image.at<cv::Vec3b>(i, j)[1] = 0;
                            berry_mask_image.at<cv::Vec3b>(i, j)[2] = 0;
                        } else {
                            berry_mask_image.at<cv::Vec3b>(i, j)[0] = 255;
                            berry_mask_image.at<cv::Vec3b>(i, j)[1] = 255;
                            berry_mask_image.at<cv::Vec3b>(i, j)[2] = 255;
                        }
                    }
                }

                cv::cvtColor(berry_mask_image, berry_mask_image, cv::COLOR_BGR2GRAY);
                double angle;
                alignTemplateToImage(berry_mask_image, target_mask_image, angle, centerPoint, output_image);

                if (DEBUG2) {
                    cv::imshow("Image", output_image);
                    cv::waitKey(0);
                }
                target_point_roll = -angle;
            }

            if (target_point_roll > 30)
                target_point_roll = 30;
            if (target_point_roll < -30)
                target_point_roll = -30;

            target_point = target_point2;
            float offset = 5;
            target_point.x += 0;
            target_point.y -= (27 + offset) * sin(target_point_roll * M_PI / 180);
            target_point.z += (27 + offset) * cos(target_point_roll * M_PI / 180);

            target_point.r = 255;
            target_point.g = 0;
            target_point.b = 0;
            cloud_cog->push_back(target_point);

            mid = std::chrono::high_resolution_clock::now();
            std::cout << "[PROC] Find pick points : " << (float)std::chrono::duration_cast<std::chrono::milliseconds>(mid - start2).count() / 1000 << " seconds." << std::endl;
            start2 = std::chrono::high_resolution_clock::now();

            if (DEBUG)
                pcl::io::savePLYFileBinary("/home/chanwoo/berry_data/debug_data/" + timestamp + "_7_cog.ply", *cloud_cog);

            ///////////////////////////////////////////////////////////////
            // Target pose
            tf_target_pose = xyzrpy_to_tfmat(target_point.x, target_point.y, target_point.z, target_point_roll, 0, yaw_offset_list[retry_count]);

            // Make approach pose by adjusting tcp transform
            tf_f2tcp_temp = get_f2tcp();
            tf_f2tcp_temp.trans_vec(1) += approach_dist;
            arm.moveTransformationTCP(tf_target_pose, tf_f2tcp_temp, 500 * robot_vel_ratio, 30000 * robot_acc_ratio, 0, true, true);

            if (DEBUG2) {
                realsense.capture(color, depth);
                masks.clear();
                masks_unripe.clear();
                det.run_inference_mask(color, mask_image, masks, masks_unripe);
                cv::imshow("Image", mask_image);
                cv::waitKey(0);
            }

            // Make approach pose by adjusting tcp transform
            tf_f2tcp_temp = get_f2tcp();
            tf_f2tcp_temp.trans_vec(1) += approach_dist / 2;
            arm.moveTransformationTCP(tf_target_pose, tf_f2tcp_temp, 500 * robot_vel_ratio, 30000 * robot_acc_ratio, 0, true, true);

            // Target pose
            arm.moveTransformationTCP(tf_target_pose, get_f2tcp(), 300 * robot_vel_ratio, 3000 * robot_acc_ratio, 0, true, true);
            ///////////////////////////////////////////////////////////////

            if (DEBUG2) {
                realsense.capture(color, depth);
                cv::imshow("Image", color);
                cv::waitKey(0);
            }

            sleep_milliseconds(200);
            arm.close_gripper();
            sleep_milliseconds(500);

            // move downward and deproach
            tf_target_pose = xyzrpy_to_tfmat(target_point.x, target_point.y, target_point.z - 10, 15, 0, 0);
            arm.moveTransformationTCP(tf_target_pose, get_f2tcp(), 500 * robot_vel_ratio, 30000 * robot_acc_ratio, 0, false, true);

            tf_target_pose = xyzrpy_to_tfmat(target_point.x - 60, target_point.y, target_point.z, 0, 0, 0);
            arm.moveTransformationTCP(tf_target_pose, get_f2tcp(), 500 * robot_vel_ratio, 30000 * robot_acc_ratio, 0, false, true);

            // Return to Initial pose
            arm.moveTransformationTCP(tf_init_pose, get_f2tcp(), 500 * robot_vel_ratio, 30000 * robot_acc_ratio, 0, true, true);

            // Eigen::VectorXf rotate_6axis;
            // arm.getCurrentJoint(rotate_6axis);
            // rotate_6axis[5] += DEG2RAD(90);
            // arm.moveJoint(rotate_6axis, 50, 50, 30, true);

            ///////////////////////////////////////////////
            // Check if berry is picked right
            // sleep_milliseconds(500);
            realsense.capture(color, depth);
            cloud->clear();
            realsense.generate_pointcloud(color, depth, cloud);
            if (DEBUG)
                pcl::io::savePLYFileBinary("/home/chanwoo/berry_data/debug_data/" + timestamp + "_8_picked.ply", *cloud);

            // Berry pick check volume
            float picked_min_x = -10, picked_max_x = 30, picked_min_y = -33, picked_max_y = 10, picked_min_z = 70, picked_max_z = 130;

            for (int i = 0; i < cloud->points.size(); i++) {
                if (isnan(cloud->points[i].x) || isnan(cloud->points[i].y) || isnan(cloud->points[i].z))
                    continue;

                if (abs(cloud->points[i].x) < 0.00001 && abs(cloud->points[i].y) < 0.00001 && abs(cloud->points[i].z) < 0.00001)
                    continue;

                if (cloud->points[i].x > picked_min_x && cloud->points[i].x < picked_max_x
                    && cloud->points[i].y > picked_min_y && cloud->points[i].y < picked_max_y
                    && cloud->points[i].z > picked_min_z && cloud->points[i].z < picked_max_z)
                    if (cloud->points[i].x != 0 && cloud->points[i].y != 0 && cloud->points[i].z != 0)
                        pick_check_cloud->points.push_back(cloud_target->points[i]);
            }
            std::cout << "Number of berry points : " << pick_check_cloud->points.size() << std::endl;

            if (DEBUG) {
                pcl::io::savePLYFileASCII("/home/chanwoo/berry_data/debug_data/" + timestamp + "_9_picked_check.ply", *pick_check_cloud);
                cv::imwrite("/home/chanwoo/berry_data/debug_data/" + timestamp + "_10_picked_check.png", color);
            }
            if (DEBUG2) {
                cv::imshow("Image", color);
                cv::waitKey(0);
            }

            if (pick_check_cloud->points.size() < 5000) {
                retry_count++;
                std::cout << "Failed to pick berry, Retry! : " << retry_count << std::endl;
                continue;
            }
            ///////////////////////////////////////////

            // If cnt is bigger than teaching_xyzrpys.size(), reset cnt
            if (cnt >= teaching_xyzrpys.size())
                cnt == 0;

            // Move to tray approach position
            arm.moveJoint(teaching_joints[0], 50, 50, 30, false);
            if (cnt < 5)
                arm.moveJoint(teaching_joints[1], 50, 50, 30, false);

            else
                arm.moveJoint(teaching_joints[3], 50, 50, 30, false);

            Eigen::VectorXf xyzrpy_temp;
            xyzrpy_temp.resize(6);

            xyzrpy_temp[0] = teaching_xyzrpys[cnt][0];
            xyzrpy_temp[1] = teaching_xyzrpys[cnt][1];
            xyzrpy_temp[2] = teaching_xyzrpys[cnt][2];
            xyzrpy_temp[3] = teaching_xyzrpys[cnt][3];
            xyzrpy_temp[4] = teaching_xyzrpys[cnt][4];
            xyzrpy_temp[5] = teaching_xyzrpys[cnt][5];

            // Slot approach
            xyzrpy_temp[2] += 30;
            tfmat tf_temp = xyzrpy_to_tfmat(xyzrpy_temp);
            arm.moveTransformationFlange(tf_temp, 500, 30000, 30, false, true);

            // Slot target
            tf_temp = xyzrpy_to_tfmat(teaching_xyzrpys[cnt]);
            arm.moveTransformationFlange(tf_temp, 300, 30000, 0, true, true);

            arm.open_gripper();
            sleep_milliseconds(300);

            // Slot deproach
            tf_temp = xyzrpy_to_tfmat(xyzrpy_temp);
            arm.moveTransformationFlange(tf_temp, 500, 30000, 30, false, true);

            if (cnt < 5)
                arm.moveJoint(teaching_joints[1], 50, 50, 30, false);
            else
                arm.moveJoint(teaching_joints[3], 50, 50, 30, false);

            arm.moveJoint(teaching_joints[0], 50, 50, 30, false);

            tf_init_pose;
            tf_init_pose = xyzrpy_to_tfmat(300, -90, 380, 0, 0, 0);
            arm.moveTransformationTCP(tf_init_pose, get_f2tcp(), 500, 30000, 0, false, true);

            mid = std::chrono::high_resolution_clock::now();
            std::cout << "[PROC] Harvest : " << (float)std::chrono::duration_cast<std::chrono::milliseconds>(mid - start2).count() / 1000 << " seconds." << std::endl;
            start2 = std::chrono::high_resolution_clock::now();

            ///////////////////////////////////////////////

            if (DEBUG)
                pcl::io::savePLYFileBinary("/home/chanwoo/berry_data/debug_data/" + timestamp + "_cog.ply", *cloud_cog);

            if (DEBUG) {
                std::cout << "Cycle finished : [" << cnt << "]" << std::endl;
            }

            cnt++;
            retry_count = 0;
        }
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        std::cout << std::endl;
        std::cout << "==========================================" << std::endl;
        std::cout << "[Elapsed time] : " << duration.count() / 1000.0 << " Seconds." << std::endl;
        std::cout << "[Average Cycle time] : " << duration.count() / (float)cnt_max / 1000.0 << " Seconds." << std::endl;
        std::cout << "==========================================" << std::endl;

        // After picking is complete
        RCLCPP_INFO(this->get_logger(), "Picking completed");

        // Send completion status
        publishStatus("picking_completed");

        // Reset flag
        picking_in_progress_ = false;

        return 0;
    }

    void publishStatus(const std::string &status)
    {
        auto message = std_msgs::msg::String();
        message.data = status;
        status_publisher_->publish(message);
    }
    // ROS 2 communication objects
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr start_subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;
    
    bool start_picking_ = false;
    // Internal state
    bool picking_in_progress_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BerryPickerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
