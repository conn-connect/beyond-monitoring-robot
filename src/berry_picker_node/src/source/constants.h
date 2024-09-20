#pragma once
#include "ARM.h"
#include "sensor.h"
#include "tf_mat.h"

typedef tf_mat<float> tfmat;

// v2r
void calibrateV2R_xarm5(ARM& rob, Sensor& rs);
void calibrateV2R_lite6(ARM& rob, Sensor& rs, bool b_load_file = true);
void loadCalibrationResult2(cv::Mat& R_f2c, cv::Mat& T_f2c, const std::string& filename);
void loadCalibrationResult(tfmat& tf_f2c, const std::string& filename);
tfmat get_f2v();
tfmat get_f2tcp();
tfmat xyzrpy_to_tfmat(float roll, float pitch, float yaw, float x, float y, float z);
tfmat xyzrpy_to_tfmat(Eigen::VectorXf pose);
Eigen::VectorXf tfmat_to_xyzrpy(const tfmat& transformation);
Eigen::Vector4f calibrateTCP(ARM& rob);
void loadTCPCalibrationResult(tfmat& tf_f2tcp, const std::string& filename);
void teachingMode(ARM& rob);
void loadTeachingJoints(std::vector<Eigen::VectorXf>& joints, std::string filename);
// tfmat get_tcp2v();