#include "constants.h"
#include "ARM.h"
#include "tf_mat.h"
#include <fstream>

using namespace std;
using namespace cv;

typedef tf_mat<float> tfmat;
void calibrateV2R_xarm5(ARM& rob, Sensor& rs)
{
    string debugloc = "/home/chanwoo/Desktop/calibration/";

    // camera and pattern info
    float fx = rs.intrinsic_color.fx;
    float fy = rs.intrinsic_color.fy;
    float px = rs.intrinsic_color.ppx;
    float py = rs.intrinsic_color.ppy;
    Mat camera = (Mat_<double>(3, 3) << fx, 0, px, 0, fy, py, 0, 0, 1);
    Mat distortion = (Mat_<double>(1, 5) << rs.intrinsic_color.coeffs[0], rs.intrinsic_color.coeffs[1], rs.intrinsic_color.coeffs[2], rs.intrinsic_color.coeffs[3], rs.intrinsic_color.coeffs[4]);

    vector<Point3f> objectPoints;
    for (int j = 0; j < 8; j++) {
        for (int i = 0; i < 5; i++) {
            objectPoints.push_back(Point3f(i * 30, j * 30 * 0.997333, 0));
        }
    }
    float first_axis_offset = 0;
    float fifth_axis_offset = 0;
    float six_axis_offset = 0;
    // saved robot poses
    std::vector<Eigen::VectorXf> joints;
    Eigen::VectorXf joint1;
    joint1.resize(7);
    joint1[0] = DEG2RAD(103 + first_axis_offset);
    joint1[1] = DEG2RAD(-58.5);
    joint1[2] = DEG2RAD(-34.1);
    joint1[3] = DEG2RAD(148);
    joint1[4] = DEG2RAD(-90.5 + fifth_axis_offset);
    joint1[5] = 0;
    joint1[6] = 0;
    joints.push_back(joint1);

    Eigen::VectorXf joint2;
    joint2.resize(7);
    joint2[0] = DEG2RAD(99 + first_axis_offset);
    joint2[1] = DEG2RAD(-47);
    joint2[2] = DEG2RAD(-50.5);
    joint2[3] = DEG2RAD(160.7);
    joint2[4] = DEG2RAD(-90.5 + fifth_axis_offset);
    joint2[5] = 0;
    joint2[6] = 0;
    joints.push_back(joint2);

    Eigen::VectorXf joint3;
    joint3.resize(7);
    joint3[0] = DEG2RAD(133 + first_axis_offset);
    joint3[1] = DEG2RAD(-5.5);
    joint3[2] = DEG2RAD(-64);
    joint3[3] = DEG2RAD(153.6);
    joint3[4] = DEG2RAD(-64.5 + fifth_axis_offset);
    joint3[5] = 0;
    joint3[6] = 0;
    joints.push_back(joint3);

    Eigen::VectorXf joint4;
    joint4.resize(7);
    joint4[0] = DEG2RAD(80.7 + first_axis_offset);
    joint4[1] = DEG2RAD(-18.4);
    joint4[2] = DEG2RAD(-58.2);
    joint4[3] = DEG2RAD(160.2);
    joint4[4] = DEG2RAD(-117.6 + fifth_axis_offset);
    joint4[5] = 0;
    joint4[6] = 0;
    joints.push_back(joint4);

    Eigen::VectorXf joint5;
    joint5.resize(7);
    joint5[0] = DEG2RAD(116 + first_axis_offset);
    joint5[1] = DEG2RAD(-26);
    joint5[2] = DEG2RAD(-54);
    joint5[3] = DEG2RAD(159);
    joint5[4] = DEG2RAD(-75 + fifth_axis_offset);
    joint5[5] = 0;
    joint5[6] = 0;
    joints.push_back(joint5);

    Eigen::VectorXf joint6;
    joint6.resize(7);
    joint6[0] = DEG2RAD(72.3 + first_axis_offset);
    joint6[1] = DEG2RAD(12.3);
    joint6[2] = DEG2RAD(-64);
    joint6[3] = DEG2RAD(145.2);
    joint6[4] = DEG2RAD(-117.6 + fifth_axis_offset);
    joint6[5] = 0;
    joint6[6] = 0;
    joints.push_back(joint6);

    Eigen::VectorXf joint7;
    joint7.resize(7);
    joint7[0] = DEG2RAD(104.3 + first_axis_offset);
    joint7[1] = DEG2RAD(22);
    joint7[2] = DEG2RAD(-93);
    joint7[3] = DEG2RAD(170);
    joint7[4] = DEG2RAD(-89.5 + fifth_axis_offset);
    joint7[5] = 0;
    joint7[6] = 0;
    joints.push_back(joint7);

    Eigen::VectorXf joint8;
    joint8.resize(7);
    joint8[0] = DEG2RAD(105 + first_axis_offset);
    joint8[1] = DEG2RAD(-49);
    joint8[2] = DEG2RAD(-45);
    joint8[3] = DEG2RAD(159);
    joint8[4] = DEG2RAD(-89.6 + fifth_axis_offset);
    joint8[5] = 0;
    joint8[6] = 0;
    joints.push_back(joint8);

    // move robot poses
    vector<Mat> R_b2f; // robot
    vector<Mat> T_b2f;
    vector<Mat> R_c2p; // image
    vector<Mat> T_c2p;
    for (int idx = 0; idx < joints.size(); idx++) {
        rob.moveJoint(joints[idx], DEG2RAD(60), DEG2RAD(2000), 0, true);
        sleep_milliseconds(3000);

        Mat image, depth;
        rs.capture(image, depth);
        // rs.save_pointcloud(debugloc + to_string(idx) + "_pointcloud.xyz", image, depth);

        vector<Point2f> imgpts;
        bool found = findChessboardCorners(image, Size(5, 8), imgpts);
        Mat gray;
        cvtColor(image, gray, COLOR_RGB2GRAY);
        if (found)
            cornerSubPix(gray, imgpts, Size(5, 5), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 40, 0.0001));

        drawChessboardCorners(image, Size(5, 8), imgpts, found);
        imwrite(debugloc + "image_" + to_string(idx) + ".jpg", image);
        cout << "found: " << found << endl;
        if (found == false)
            continue;

        // project to plane
        vector<Point3f> unprojpts;
        vector<Point3f> imgPt3d;
        for (int i = 0; i < imgpts.size(); i++) {
            float pt2d[2] = { imgpts[i].x, imgpts[i].y };
            float pt3d[3];
            rs2_deproject_pixel_to_point(pt3d, &rs.intrinsic_color, pt2d, 1);

            Point3f unproj(pt3d[0], pt3d[1], pt3d[2]);
            unprojpts.push_back(unproj);
        }
        vector<Point2f> opencv_imgpt;
        Mat rveczero = Mat::zeros(Size(3, 1), CV_32F);
        Mat distortionzero = Mat::zeros(Size(5, 1), CV_32F);
        projectPoints(unprojpts, rveczero, rveczero, camera, distortionzero, opencv_imgpt);

        Mat rvec, tvec_;
        solvePnP(objectPoints, opencv_imgpt, camera, distortionzero, rvec, tvec_);

        Mat rot;
        Rodrigues(rvec, rot);
        R_c2p.push_back(rot);
        T_c2p.push_back(tvec_);

        tfmat b2f;
        rob.getTransformationFlange(b2f);

        Mat b2f_rot = (Mat_<double>(3, 3) << b2f.rot_mat(0, 0), b2f.rot_mat(0, 1), b2f.rot_mat(0, 2),
            b2f.rot_mat(1, 0), b2f.rot_mat(1, 1), b2f.rot_mat(1, 2),
            b2f.rot_mat(2, 0), b2f.rot_mat(2, 1), b2f.rot_mat(2, 2));
        R_b2f.push_back(b2f_rot);
        Mat b2f_trans = (Mat_<double>(3, 1) << b2f.trans_vec.x(), b2f.trans_vec.y(), b2f.trans_vec.z());
        T_b2f.push_back(b2f_trans);
    }

    Mat R_f2c;
    Mat T_f2c;

    calibrateHandEye(R_b2f, T_b2f, R_c2p, T_c2p, R_f2c, T_f2c, HandEyeCalibrationMethod(0));
    cout << "R_f2c: " << R_f2c << endl;
    cout << "T_f2c: " << T_f2c << endl;
}

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

void calibrateV2R_lite6(ARM& rob, Sensor& rs, bool b_load_file)
{
    string debugloc = "/home/chanwoo/berry_data/calibration/";
    int calibration_x = 6;
    int calibration_y = 9;

    // camera and pattern info
    float fx = rs.intrinsic_color.fx;
    float fy = rs.intrinsic_color.fy;
    float px = rs.intrinsic_color.ppx;
    float py = rs.intrinsic_color.ppy;
    Mat camera = (Mat_<double>(3, 3) << fx, 0, px, 0, fy, py, 0, 0, 1);
    Mat distortion = (Mat_<double>(1, 5) << rs.intrinsic_color.coeffs[0], rs.intrinsic_color.coeffs[1], rs.intrinsic_color.coeffs[2], rs.intrinsic_color.coeffs[3], rs.intrinsic_color.coeffs[4]);

    std::cout << "Camera : " << camera << std::endl;
    std::cout << "Distortion : " << distortion << std::endl;

    vector<Point3f> objectPoints;
    for (int j = 0; j < calibration_y; j++) {
        for (int i = 0; i < calibration_x; i++) {
            objectPoints.push_back(Point3f(i * 25, j * 25, 0));
        }
    }
    float first_axis_offset = 0;
    float fifth_axis_offset = 0;
    float six_axis_offset = 0;

    std::vector<Eigen::VectorXf> joints;
    std::string filename = debugloc + "cal_pose.txt"; // Calibration pose saved file

    if (b_load_file == false) {
        int shot_counter = 0;
        while (shot_counter < 15) {
            cv::Mat color, depth;
            rs.capture(color, depth);

            vector<Point2f> imgpts_temp;
            bool found = findChessboardCorners(color, Size(calibration_x, calibration_y), imgpts_temp);
            Mat gray;
            cvtColor(color, gray, COLOR_RGB2GRAY);
            if (found)
                cornerSubPix(gray, imgpts_temp, Size(5, 5), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 40, 0.0001));

            drawChessboardCorners(color, Size(calibration_x, calibration_y), imgpts_temp, found);

            cv::imshow("Color image", color);
            char key = cv::waitKey(1);
            if (key == 'c') {
                Eigen::VectorXf joint_temp;
                rob.getCurrentJoint(joint_temp);
                joints.push_back(joint_temp);
                shot_counter++;
                std::cout << "Number of saved position : " << shot_counter << std::endl;
                std::cout << "Current robot pose : " << joint_temp << std::endl;
            }
        }
        cv::destroyWindow("Color image");

        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Unable to open file for writing: " << filename << std::endl;
            return;
        }

        // Save each joint vector to the file
        for (const auto& joint : joints) {
            for (int i = 0; i < joint.size(); ++i) {
                file << joint[i]; // Write each element of the vector
                if (i != joint.size() - 1) {
                    file << " "; // Separate elements with spaces
                }
            }
            file << "\n"; // Newline after each joint vector
        }

        file.close();
        std::cout << "Joint data saved to file: " << filename << std::endl;
        joints.clear();
    }

    {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Unable to open file for reading: " << filename << std::endl;
            return;
        }

        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::vector<float> joint_values((std::istream_iterator<float>(iss)), std::istream_iterator<float>());

            // Create Eigen::VectorXf from the joint values
            Eigen::VectorXf joint_vector(joint_values.size());
            for (size_t i = 0; i < joint_values.size(); ++i) {
                joint_vector[i] = joint_values[i];
            }

            // Add the joint vector to the vector of joints
            joints.push_back(joint_vector);
        }

        file.close();
        std::cout << "Joint data loaded from file: " << filename << std::endl;
    }

    // move robot poses
    vector<Mat> R_b2f; // robot
    vector<Mat> T_b2f;
    vector<Mat> R_c2p; // image
    vector<Mat> T_c2p;
    for (int idx = 0; idx < joints.size(); idx++) {
        rob.moveJoint(joints[idx], DEG2RAD(60), DEG2RAD(2000), 0, true);
        sleep_milliseconds(3000);

        Mat image, depth;
        rs.capture(image, depth);
        // rs.save_pointcloud(debugloc + to_string(idx) + "_pointcloud.xyz", image, depth);

        vector<Point2f> imgpts;
        bool found = findChessboardCorners(image, Size(calibration_x, calibration_y), imgpts);
        Mat gray;
        cvtColor(image, gray, COLOR_RGB2GRAY);
        if (found)
            cornerSubPix(gray, imgpts, Size(5, 5), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 40, 0.0001));

        drawChessboardCorners(image, Size(calibration_x, calibration_y), imgpts, found);
        imwrite(debugloc + "image_" + to_string(idx) + ".jpg", image);
        cv::imshow("calibration image", image);
        cv::waitKey(100);
        if (found == false)
            continue;

        // project to plane
        vector<Point3f> unprojpts;
        vector<Point3f> imgPt3d;
        for (int i = 0; i < imgpts.size(); i++) {
            float pt2d[2] = { imgpts[i].x, imgpts[i].y };
            float pt3d[3];
            rs2_deproject_pixel_to_point(pt3d, &rs.intrinsic_color, pt2d, 1);

            Point3f unproj(pt3d[0], pt3d[1], pt3d[2]);
            unprojpts.push_back(unproj);
            // N*(X-P) = 0 => N*X = N*P => z*(nx*x + ny*y + nz) = NP => z = NP / (nx*x - ny*y + nz)
            // float z = N.dot(P) / N.dot(unproj);

            // Point3f img3d = unproj * z;
            // imgPt3d.push_back(img3d);
        }
        // save_pointcloud(debugloc + to_string(idx)+"_imgPt3d.xyz", imgPt3d);
        vector<Point2f> opencv_imgpt;
        Mat rveczero = Mat::zeros(Size(3, 1), CV_32F);
        Mat distortionzero = Mat::zeros(Size(5, 1), CV_32F);
        projectPoints(unprojpts, rveczero, rveczero, camera, distortionzero, opencv_imgpt);

        Mat rvec, tvec_;
        solvePnP(objectPoints, opencv_imgpt, camera, distortionzero, rvec, tvec_);

        Mat rot;
        Rodrigues(rvec, rot);
        // cout << "rot_: " << rot << endl;
        // cout << "tvec_: " << tvec_ << endl;
        R_c2p.push_back(rot);
        T_c2p.push_back(tvec_);

        tfmat b2f;
        rob.getTransformationFlange(b2f);

        cout << b2f << endl;

        Mat b2f_rot = (Mat_<double>(3, 3) << b2f.rot_mat(0, 0), b2f.rot_mat(0, 1), b2f.rot_mat(0, 2),
            b2f.rot_mat(1, 0), b2f.rot_mat(1, 1), b2f.rot_mat(1, 2),
            b2f.rot_mat(2, 0), b2f.rot_mat(2, 1), b2f.rot_mat(2, 2));
        R_b2f.push_back(b2f_rot);
        Mat b2f_trans = (Mat_<double>(3, 1) << b2f.trans_vec.x(), b2f.trans_vec.y(), b2f.trans_vec.z());
        T_b2f.push_back(b2f_trans);
    }

    Mat R_f2c;
    Mat T_f2c;

    calibrateHandEye(R_b2f, T_b2f, R_c2p, T_c2p, R_f2c, T_f2c, HandEyeCalibrationMethod(2));
    cout << "R_f2c: " << R_f2c << endl;
    cout << "T_f2c: " << T_f2c << endl;

    std::string filename2 = debugloc + "v2r_cal.txt"; // Calibration pose saved file
    std::ofstream file2(filename2);

    if (!file2.is_open()) {
        std::cerr << "Unable to open file for writing: " << filename2 << std::endl;
        return;
    }

    // 회전 행렬 저장 (R_f2c)
    file2 << "Rotation Matrix (R_f2c):" << std::endl;
    for (int i = 0; i < R_f2c.rows; i++) {
        for (int j = 0; j < R_f2c.cols; j++) {
            file2 << R_f2c.at<double>(i, j);
            if (j != R_f2c.cols - 1)
                file2 << " "; // 요소들 사이에 공백
        }
        file2 << std::endl; // 행 끝에는 새 줄
    }

    // 이동 벡터 저장 (T_f2c)
    file2 << "Translation Vector (T_f2c):" << std::endl;
    for (int i = 0; i < T_f2c.rows; i++) {
        file2 << T_f2c.at<double>(i, 0) << std::endl;
    }

    file2.close();
    std::cout << "Calibration result saved to file: " << filename2 << std::endl;
}

void loadCalibrationResult2(cv::Mat& R_f2c, cv::Mat& T_f2c, const std::string& filename)
{
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Unable to open file for reading: " << filename << std::endl;
        return;
    }

    std::string line;
    // 회전 행렬 읽기
    std::getline(file, line); // "Rotation Matrix (R_f2c):"
    for (int i = 0; i < R_f2c.rows; i++) {
        for (int j = 0; j < R_f2c.cols; j++) {
            file >> R_f2c.at<double>(i, j);
        }
    }

    // 이동 벡터 읽기
    std::getline(file, line); // "Translation Vector (T_f2c):"
    std::getline(file, line); // 빈 줄 넘기기
    for (int i = 0; i < T_f2c.rows; i++) {
        file >> T_f2c.at<double>(i, 0);
    }

    file.close();
    std::cout << "Calibration result loaded from file: " << filename << std::endl;
}

void loadCalibrationResult(tfmat& tf_f2c, const std::string& filename)
{
    cv::Mat R_f2c = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat T_f2c = cv::Mat::zeros(3, 1, CV_64F);
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Unable to open file for reading: " << filename << std::endl;
        return;
    }

    std::string line;
    // 회전 행렬 읽기
    std::getline(file, line); // "Rotation Matrix (R_f2c):"
    for (int i = 0; i < R_f2c.rows; i++) {
        for (int j = 0; j < R_f2c.cols; j++) {
            file >> R_f2c.at<double>(i, j);
        }
    }

    // 이동 벡터 읽기
    std::getline(file, line); // "Translation Vector (T_f2c):"
    std::getline(file, line); // 빈 줄 넘기기
    for (int i = 0; i < T_f2c.rows; i++) {
        file >> T_f2c.at<double>(i, 0);
    }

    file.close();
    tf_f2c.rot_mat(0, 0) = R_f2c.at<double>(0, 0);
    tf_f2c.rot_mat(0, 1) = R_f2c.at<double>(0, 1);
    tf_f2c.rot_mat(0, 2) = R_f2c.at<double>(0, 2);
    tf_f2c.rot_mat(1, 0) = R_f2c.at<double>(1, 0);
    tf_f2c.rot_mat(1, 1) = R_f2c.at<double>(1, 1);
    tf_f2c.rot_mat(1, 2) = R_f2c.at<double>(1, 2);
    tf_f2c.rot_mat(2, 0) = R_f2c.at<double>(2, 0);
    tf_f2c.rot_mat(2, 1) = R_f2c.at<double>(2, 1);
    tf_f2c.rot_mat(2, 2) = R_f2c.at<double>(2, 2);
    tf_f2c.trans_vec(0) = T_f2c.at<double>(0, 0);
    tf_f2c.trans_vec(1) = T_f2c.at<double>(1, 0);
    tf_f2c.trans_vec(2) = T_f2c.at<double>(2, 0);

    std::cout << "Calibration result loaded from file: " << filename << std::endl;
}

tfmat get_f2v()
{
    tfmat f2v_;

    // tfmat
    Eigen::Matrix<float, 4, 4> f2v;
    f2v << -1, 0, 0, 0,
        0, 0, 1, 0,
        0, 1, 0, 0,
        0, 0, 0, 1;
    f2v_ = tfmat(f2v);

    return f2v_;
}

tfmat get_f2tcp()
{
    tfmat f2tcp_;

    // tfmat
    Eigen::Matrix<float, 4, 4> temp;
    temp << 0, 1, 0, 0, // 25
        1, 0, 0, 140,
        0, 0, -1, 33.1,
        0, 0, 0, 1;
    f2tcp_ = tfmat(temp);
    return f2tcp_;
}

tfmat xyzrpy_to_tfmat(float x, float y, float z, float roll, float pitch, float yaw)
{
    roll = DEG2RAD(roll);
    pitch = DEG2RAD(pitch);
    yaw = DEG2RAD(yaw);

    // Calculate rotation matrix around X axis (Roll)
    Eigen::Matrix3d R_x;
    R_x << 1, 0, 0,
        0, cos(roll), -sin(roll),
        0, sin(roll), cos(roll);

    // Calculate rotation matrix around Y axis (Pitch)
    Eigen::Matrix3d R_y;
    R_y << cos(pitch), 0, sin(pitch),
        0, 1, 0,
        -sin(pitch), 0, cos(pitch);

    // Calculate rotation matrix around Z axis (Yaw)
    Eigen::Matrix3d R_z;
    R_z << cos(yaw), -sin(yaw), 0,
        sin(yaw), cos(yaw), 0,
        0, 0, 1;

    // Combine the rotation matrices
    Eigen::Matrix3d R = R_z * R_y * R_x;

    tfmat transformation;
    transformation.rot_mat(0, 0) = R(0, 0);
    transformation.rot_mat(0, 1) = R(0, 1);
    transformation.rot_mat(0, 2) = R(0, 2);
    transformation.rot_mat(1, 0) = R(1, 0);
    transformation.rot_mat(1, 1) = R(1, 1);
    transformation.rot_mat(1, 2) = R(1, 2);
    transformation.rot_mat(2, 0) = R(2, 0);
    transformation.rot_mat(2, 1) = R(2, 1);
    transformation.rot_mat(2, 2) = R(2, 2);
    transformation.trans_vec(0) = x;
    transformation.trans_vec(1) = y;
    transformation.trans_vec(2) = z;

    return transformation;
}

tfmat xyzrpy_to_tfmat(Eigen::VectorXf pose)
{
    float x = pose[0];
    float y = pose[1];
    float z = pose[2];
    float roll = pose[3];
    float pitch = pose[4];
    float yaw = pose[5];

    roll = DEG2RAD(roll);
    pitch = DEG2RAD(pitch);
    yaw = DEG2RAD(yaw);

    // Calculate rotation matrix around X axis (Roll)
    Eigen::Matrix3d R_x;
    R_x << 1, 0, 0,
        0, cos(roll), -sin(roll),
        0, sin(roll), cos(roll);

    // Calculate rotation matrix around Y axis (Pitch)
    Eigen::Matrix3d R_y;
    R_y << cos(pitch), 0, sin(pitch),
        0, 1, 0,
        -sin(pitch), 0, cos(pitch);

    // Calculate rotation matrix around Z axis (Yaw)
    Eigen::Matrix3d R_z;
    R_z << cos(yaw), -sin(yaw), 0,
        sin(yaw), cos(yaw), 0,
        0, 0, 1;

    // Combine the rotation matrices
    Eigen::Matrix3d R = R_z * R_y * R_x;

    tfmat transformation;
    transformation.rot_mat(0, 0) = R(0, 0);
    transformation.rot_mat(0, 1) = R(0, 1);
    transformation.rot_mat(0, 2) = R(0, 2);
    transformation.rot_mat(1, 0) = R(1, 0);
    transformation.rot_mat(1, 1) = R(1, 1);
    transformation.rot_mat(1, 2) = R(1, 2);
    transformation.rot_mat(2, 0) = R(2, 0);
    transformation.rot_mat(2, 1) = R(2, 1);
    transformation.rot_mat(2, 2) = R(2, 2);
    transformation.trans_vec(0) = x;
    transformation.trans_vec(1) = y;
    transformation.trans_vec(2) = z;

    return transformation;
}

Eigen::VectorXf tfmat_to_xyzrpy(const tfmat& transformation)
{
    string debugloc = "/home/chanwoo/berry_data/calibration/";
    string filename = debugloc + "tcp_calibration_result.txt";

    float roll, pitch, yaw, x, y, z;
    Eigen::VectorXf xyzrpy(6);
    // 회전 행렬 추출
    Eigen::Matrix3d R;
    R << transformation.rot_mat(0, 0), transformation.rot_mat(0, 1), transformation.rot_mat(0, 2),
        transformation.rot_mat(1, 0), transformation.rot_mat(1, 1), transformation.rot_mat(1, 2),
        transformation.rot_mat(2, 0), transformation.rot_mat(2, 1), transformation.rot_mat(2, 2);

    // RPY 추출
    pitch = std::asin(-R(2, 0)); // Pitch 계산

    // Gimbal Lock을 고려한 Roll, Yaw 계산
    if (std::abs(std::cos(pitch)) > 1e-6) { // 일반적인 경우
        roll = std::atan2(R(2, 1), R(2, 2)); // Roll 계산
        yaw = std::atan2(R(1, 0), R(0, 0)); // Yaw 계산
    } else { // Gimbal Lock 상태인 경우 (Pitch가 ±90도)
        roll = 0; // Roll은 정의되지 않으므로 0으로 설정
        yaw = std::atan2(-R(0, 1), R(1, 1)); // Yaw 계산
    }

    // 위치(X, Y, Z) 추출
    x = transformation.trans_vec(0);
    y = transformation.trans_vec(1);
    z = transformation.trans_vec(2);

    // RPY 값을 각도로 변환
    roll = RAD2DEG(roll);
    pitch = RAD2DEG(pitch);
    yaw = RAD2DEG(yaw);

    xyzrpy << x, y, z, roll, pitch, yaw;
    return xyzrpy;
}

Eigen::Vector4f calibrateTCP(ARM& rob)
{
    string debugloc = "/home/chanwoo/berry_data/calibration/";
    std::string filename = debugloc + "tcp_calibration_result.txt";

    int cnt = 0;
    int max_cnt = 8;
    int hi;
    std::vector<Eigen::Vector3f> poses;
    while (cnt < max_cnt) {
        std::cout << "Please move the robot to the calibration pose " << std::endl;
        std::cin >> hi;
        tfmat tf_base2flange;
        rob.getTransformationFlange(tf_base2flange);
        Eigen::Vector3f pose;
        pose << tf_base2flange.trans_vec(0), tf_base2flange.trans_vec(1), tf_base2flange.trans_vec(2);
        cout << "pose: " << pose << endl;
        poses.push_back(pose);
        cnt++;
    }

    // A * x = b 형식으로 정의할 것
    Eigen::MatrixXf A(max_cnt, 4);
    Eigen::VectorXf b(max_cnt);

    // 행렬 A와 벡터 b 채우기
    for (int i = 0; i < max_cnt; ++i) {
        const Eigen::Vector3f& p = poses[i];
        A(i, 0) = 2 * p(0);
        A(i, 1) = 2 * p(1);
        A(i, 2) = 2 * p(2);
        A(i, 3) = 1;
        b(i) = p.squaredNorm(); // p.x^2 + p.y^2 + p.z^2
    }

    // 최소 자승법으로 해를 구함
    Eigen::Vector4f x = A.colPivHouseholderQr().solve(b);

    // 구의 중심 좌표 추출
    float cx = x(0);
    float cy = x(1);
    float cz = x(2);

    // 구의 반지름 계산
    float radius = std::sqrt(cx * cx + cy * cy + cz * cz + x(3));

    std::ofstream file(filename);
    if (file.is_open()) {
        // 구의 중심과 반지름을 파일에 저장
        file << "TCP Position (cx, cy, cz): " << cx << ", " << cy << ", " << cz << std::endl;
        file << "Radius (distance from TCP to flange): " << radius << std::endl;
        file.close();
        std::cout << "Result saved to " << filename << std::endl;
    } else {
        std::cerr << "Unable to open file: " << filename << std::endl;
    }

    return Eigen::Vector4f(cx, cy, cz, radius); // 구의 중심 (cx, cy, cz)와 반지름 반환
}

void loadTCPCalibrationResult(tfmat& tf_f2tcp, const std::string& filename)
{
    std::ifstream file(filename);

    Eigen::Vector4d result;

    if (file.is_open()) {
        std::string line;
        // 첫 번째 줄 (TCP Position)을 읽음
        std::getline(file, line);
        std::istringstream iss1(line);
        std::string dummy;
        char comma;
        iss1 >> dummy >> dummy >> dummy >> result(0) >> comma >> result(1) >> comma >> result(2);

        // 두 번째 줄 (Radius)을 읽음
        std::getline(file, line);
        std::istringstream iss2(line);
        iss2 >> dummy >> dummy >> dummy >> dummy >> dummy >> result(3);

        file.close();
        std::cout << "Result loaded from " << filename << std::endl;
    } else {
        std::cerr << "Unable to open file: " << filename << std::endl;
    }

    // tfmat
    Eigen::Matrix<float, 4, 4> temp;
    temp << 0, 1, 0, result(0), // 25
        1, 0, 0, result(1),
        0, 0, -1, result(2),
        0, 0, 0, 1;
    tf_f2tcp = tfmat(temp);

    return;
}

void teachingMode(ARM& rob)
{
    std::string filename = "/home/chanwoo/berry_data/teaching/joints.txt";
    int a;
    std::vector<Eigen::VectorXf> joints;
    std::cout << "Robot teaching mode start" << std::endl;
    while (1) {
        std::cin >> a;
        if (a == 0)
            break;
        else {
            Eigen::VectorXf joint_temp;
            rob.getCurrentJoint(joint_temp);
            joints.push_back(joint_temp);
        }
    }

    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Unable to open file for writing: " << filename << std::endl;
        return;
    }

    // Save each joint vector to the file
    for (const auto& joint : joints) {
        for (int i = 0; i < joint.size(); ++i) {
            file << joint[i]; // Write each element of the vector
            if (i != joint.size() - 1) {
                file << " "; // Separate elements with spaces
            }
        }
        file << "\n"; // Newline after each joint vector
    }

    file.close();
    std::cout << "Joint data saved to file: " << filename << std::endl;
    joints.clear();

    return;
}

void loadTeachingJoints(std::vector<Eigen::VectorXf>& joints, std::string filename)
{
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Unable to open file for reading: " << filename << std::endl;
        return;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::vector<float> joint_values((std::istream_iterator<float>(iss)), std::istream_iterator<float>());

        // Create Eigen::VectorXf from the joint values
        Eigen::VectorXf joint_vector(joint_values.size());
        for (size_t i = 0; i < joint_values.size(); ++i) {
            joint_vector[i] = joint_values[i];
        }

        // Add the joint vector to the vector of joints
        joints.push_back(joint_vector);
    }

    file.close();
    std::cout << "Joint data loaded from file: " << filename << std::endl;
}