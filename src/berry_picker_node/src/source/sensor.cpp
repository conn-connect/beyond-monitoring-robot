#include "sensor.h"
#include <omp.h>

void Sensor::start(std::string serial_num, int image_size, bool isD405_)
{
    isD405 = isD405_;
    rs2::context ctx;
    for (auto&& dev : ctx.query_devices()) {
        std::string serial_ = std::string(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
        if (serial_num == serial_) {
            rs2::config cfg;
            if (image_size == 0) {
                cfg.enable_stream(rs2_stream::RS2_STREAM_COLOR, -1, 640, 360, rs2_format::RS2_FORMAT_RGB8, 0);
                cfg.enable_stream(rs2_stream::RS2_STREAM_DEPTH, -1, 640, 360, rs2_format::RS2_FORMAT_Z16, 0);
            } else if (image_size == 1) {
                cfg.enable_stream(rs2_stream::RS2_STREAM_COLOR, -1, 1280, 720, rs2_format::RS2_FORMAT_RGB8, 0);
                cfg.enable_stream(rs2_stream::RS2_STREAM_DEPTH, -1, 1280, 720, rs2_format::RS2_FORMAT_Z16, 0);
            }
            cfg.enable_device(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
            pipe.start(cfg);

            rs2::frameset data = pipe.wait_for_frames();
            rs2::align align_to(RS2_STREAM_COLOR);
            rs2::frameset aligned_data = align_to.process(data);

            rs2::frame color_frame = aligned_data.get_color_frame();
            rs2::frame depth_frame = aligned_data.get_depth_frame();

            intrinsic_color = color_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
            intrinsic_depth = depth_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

            // rs2::sensor sensor = dev.first<rs2::sensor>();
            // if (sensor.supports(RS2_OPTION_HDR_ENABLED)) {
            //     sensor.set_option(RS2_OPTION_HDR_ENABLED, 1);
            //     std::cout << "[Sensor] HDR is enabled" << std::endl;
            // }
        }
    }
}

void Sensor::capture(cv::Mat& color, cv::Mat& depth)
{
    rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
    rs2::align align_to(RS2_STREAM_COLOR);
    rs2::frameset aligned_data = align_to.process(data);

    rs2::frame color_frame = aligned_data.get_color_frame();
    rs2::frame depth_frame = aligned_data.get_depth_frame();

    const int color_w = color_frame.as<rs2::video_frame>().get_width();
    const int color_h = color_frame.as<rs2::video_frame>().get_height();
    cv::Mat color_mat(cv::Size(color_w, color_h), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
    cv::cvtColor(color_mat, color, cv::COLOR_RGB2BGR);

    const int depth_w = depth_frame.as<rs2::video_frame>().get_width();
    const int depth_h = depth_frame.as<rs2::video_frame>().get_height();
    cv::Mat depth_mat(cv::Size(depth_w, depth_h), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
    depth = depth_mat.clone();
}

void Sensor::capture_cloud(cv::Mat& color, cv::Mat& depth, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    cloud->clear();
    rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
    rs2::align align_to(RS2_STREAM_COLOR);
    rs2::frameset aligned_data = align_to.process(data);

    rs2::frame color_frame = aligned_data.get_color_frame();
    rs2::frame depth_frame = aligned_data.get_depth_frame();

    const int color_w = color_frame.as<rs2::video_frame>().get_width();
    const int color_h = color_frame.as<rs2::video_frame>().get_height();
    cv::Mat color_mat(cv::Size(color_w, color_h), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
    cv::cvtColor(color_mat, color, cv::COLOR_RGB2BGR);

    const int depth_w = depth_frame.as<rs2::video_frame>().get_width();
    const int depth_h = depth_frame.as<rs2::video_frame>().get_height();
    cv::Mat depth_mat(cv::Size(depth_w, depth_h), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
    depth = depth_mat.clone();

    const uint16_t* depth_data = (const uint16_t*)depth_frame.get_data();
    const rs2_intrinsics intr = depth_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

    for (int y = 0; y < depth_h; y++) {
        for (int x = 0; x < depth_w; x++) {
            uint16_t depth_value = depth_data[y * depth_w + x];

            float depth_in_meters = depth_value * 0.1f; // 깊이 값을 밀리미터 단위로 변환
            float pixel[2] = { (float)x, (float)y };
            float point[3];
            rs2_deproject_pixel_to_point(point, &intr, pixel, depth_in_meters);

            if (depth_value == 0) {
                point[0] = 0;
                point[1] = 0;
                point[2] = 0;
            }

            pcl::PointXYZRGB pcl_point;
            pcl_point.x = point[0];
            pcl_point.y = point[1];
            pcl_point.z = point[2];

            // Get RGB values from color frame
            int color_x = static_cast<int>(x * color_w / depth_w);
            int color_y = static_cast<int>(y * color_h / depth_h);
            cv::Vec3b rgb = color.at<cv::Vec3b>(color_y, color_x);
            pcl_point.r = rgb[2];
            pcl_point.g = rgb[1];
            pcl_point.b = rgb[0];

            cloud->points.push_back(pcl_point);
        }
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;
}

// void Sensor::generate_pointcloud(cv::Mat color, cv::Mat depth, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
// {
//     cloud->clear();
//     const uint16_t* depth_data = (const uint16_t*)depth.data;
//     const rs2_intrinsics intr = intrinsic_depth;

//     const int depth_w = depth.cols;
//     const int depth_h = depth.rows;
//     const int color_w = color.cols;
//     const int color_h = color.rows;

//     for (int y = 0; y < depth_h; y++) {
//         for (int x = 0; x < depth_w; x++) {
//             uint16_t depth_value = depth_data[y * depth_w + x];

//             float depth_in_meters = depth_value * 0.1f; // 깊이 값을 밀리미터 단위로 변환
//             float pixel[2] = { (float)x, (float)y };
//             float point[3];
//             rs2_deproject_pixel_to_point(point, &intr, pixel, depth_in_meters);

//             if (depth_value == 0) {
//                 point[0] = 0;
//                 point[1] = 0;
//                 point[2] = 0;
//             }

//             pcl::PointXYZRGB pcl_point;
//             pcl_point.x = point[0];
//             pcl_point.y = point[1];
//             pcl_point.z = point[2];

//             // Get RGB values from color frame
//             int color_x = static_cast<int>(x * color_w / depth_w);
//             int color_y = static_cast<int>(y * color_h / depth_h);
//             cv::Vec3b rgb = color.at<cv::Vec3b>(color_y, color_x);
//             pcl_point.r = rgb[2];
//             pcl_point.g = rgb[1];
//             pcl_point.b = rgb[0];

//             cloud->points.push_back(pcl_point);
//         }
//     }
//     cloud->width = cloud->points.size();
//     cloud->height = 1;
//     cloud->is_dense = false;
// }

void Sensor::generate_pointcloud(cv::Mat color, cv::Mat depth, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
    cloud->clear(); // 기존 포인트클라우드 초기화
    const uint16_t* depth_data = (const uint16_t*)depth.data;
    const rs2_intrinsics intr = intrinsic_depth;

    const int depth_w = depth.cols;
    const int depth_h = depth.rows;
    const int color_w = color.cols;
    const int color_h = color.rows;

    // 포인트클라우드의 크기를 미리 할당
    cloud->points.resize(depth_w * depth_h);

// OpenMP를 이용한 병렬 처리
#pragma omp parallel for collapse(2)
    for (int y = 0; y < depth_h; ++y) {
        for (int x = 0; x < depth_w; ++x) {
            int idx = y * depth_w + x; // 고유 인덱스 계산

            uint16_t depth_value = depth_data[idx];
            float depth_in_meters = depth_value * 0.1f; // 깊이 값을 밀리미터 단위로 변환
            float pixel[2] = { (float)x, (float)y };
            float point[3];
            rs2_deproject_pixel_to_point(point, &intr, pixel, depth_in_meters);

            if (depth_value == 0) {
                point[0] = 0;
                point[1] = 0;
                point[2] = 0;
            }

            pcl::PointXYZRGB pcl_point;
            pcl_point.x = point[0];
            pcl_point.y = point[1];
            pcl_point.z = point[2];

            // Get RGB values from color frame
            int color_x = static_cast<int>(x * color_w / depth_w);
            int color_y = static_cast<int>(y * color_h / depth_h);
            cv::Vec3b rgb = color.at<cv::Vec3b>(color_y, color_x);
            pcl_point.r = rgb[2];
            pcl_point.g = rgb[1];
            pcl_point.b = rgb[0];

            cloud->points[idx] = pcl_point; // 포인트클라우드에 포인트 추가
        }
    }

    // 포인트클라우드의 메타데이터 설정
    cloud->width = depth_w;
    cloud->height = depth_h;
    cloud->is_dense = false;
}

// void Sensor::generate_pointcloud(cv::Mat color, cv::Mat depth, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
// {
//     cloud->clear();
//     const uint16_t* depth_data = (const uint16_t*)depth.data;
//     const rs2_intrinsics intr = intrinsic_depth;

//     const int depth_w = depth.cols;
//     const int depth_h = depth.rows;
//     const int color_w = color.cols;
//     const int color_h = color.rows;

//     // 포인트클라우드의 크기를 미리 할당
//     cloud->points.resize(depth_w * depth_h);

// // OpenMP 병렬 처리
// #pragma omp parallel for schedule(static)
//     for (int idx = 0; idx < depth_w * depth_h; ++idx) {
//         int y = idx / depth_w;
//         int x = idx % depth_w;

//         uint16_t depth_value = depth_data[idx];
//         float depth_in_meters = depth_value * 0.001f; // 깊이 값을 미터 단위로 변환 (단위 확인 필요)
//         float pixel[2] = { (float)x, (float)y };
//         float point[3];

//         // rs2_deproject_pixel_to_point 함수가 스레드 안전한지 확인 필요
//         rs2_deproject_pixel_to_point(point, &intr, pixel, depth_in_meters);

//         if (depth_value == 0) {
//             point[0] = point[1] = point[2] = 0;
//         }

//         pcl::PointXYZRGB pcl_point;
//         pcl_point.x = point[0];
//         pcl_point.y = point[1];
//         pcl_point.z = point[2];

//         int color_x = static_cast<int>(x * color_w / depth_w);
//         int color_y = static_cast<int>(y * color_h / depth_h);

//         // 인덱스 범위 보정
//         color_x = std::min(color_x, color_w - 1);
//         color_y = std::min(color_y, color_h - 1);

//         cv::Vec3b rgb = color.at<cv::Vec3b>(color_y, color_x);
//         pcl_point.r = rgb[2];
//         pcl_point.g = rgb[1];
//         pcl_point.b = rgb[0];

//         // 각 스레드가 고유한 인덱스에 쓰기 때문에 동기화가 필요 없음
//         cloud->points[idx] = pcl_point;
//     }

//     cloud->width = depth_w;
//     cloud->height = depth_h;
//     cloud->is_dense = false;
// }

void Sensor::capture_cloud_average(cv::Mat& color, cv::Mat& depth, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    cloud->clear();
    const int num_frames = 5; // 사용할 프레임 수
    std::vector<cv::Mat> depth_frames(num_frames);

    // RealSense 프레임 수집 및 depth 데이터 누적
    for (int i = 0; i < num_frames; i++) {
        rs2::frameset data = pipe.wait_for_frames(); // 카메라로부터 다음 프레임을 대기
        rs2::align align_to(RS2_STREAM_COLOR);
        rs2::frameset aligned_data = align_to.process(data);

        rs2::frame color_frame = aligned_data.get_color_frame();
        rs2::frame depth_frame = aligned_data.get_depth_frame();

        // 첫 번째 프레임에서 color frame 저장
        if (i == 0) {
            const int color_w = color_frame.as<rs2::video_frame>().get_width();
            const int color_h = color_frame.as<rs2::video_frame>().get_height();
            cv::Mat color_mat(cv::Size(color_w, color_h), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
            cv::cvtColor(color_mat, color, cv::COLOR_RGB2BGR);
        }

        // 각 프레임의 depth 데이터 저장 (16-bit unsigned depth)
        const int depth_w = depth_frame.as<rs2::video_frame>().get_width();
        const int depth_h = depth_frame.as<rs2::video_frame>().get_height();
        depth_frames[i] = cv::Mat(cv::Size(depth_w, depth_h), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
    }

    // Depth 프레임 평균 계산
    cv::Mat avg_depth = cv::Mat::zeros(depth_frames[0].size(), CV_32FC1); // 평균을 계산할 때는 float로
    cv::Mat cnt_mat = cv::Mat::zeros(depth_frames[0].size(), CV_32SC1); // 유효한 depth 값을 세기 위해 정수형 행렬

    for (int i = 0; i < num_frames; i++) {
        uint16_t* p_depth_frames = (uint16_t*)depth_frames[i].data;
        int depth_w = depth_frames[i].cols;
        int depth_h = depth_frames[i].rows;

        for (int y = 0; y < depth_h; y++) {
            for (int x = 0; x < depth_w; x++) {
                uint16_t depth_value = p_depth_frames[y * depth_w + x];

                // 유효한 깊이 값이 0이 아닐 때 평균에 추가
                if (depth_value != 0) {
                    avg_depth.at<float>(y, x) += (float)depth_value;
                    cnt_mat.at<int>(y, x)++;
                }
            }
        }
    }

    // 평균 계산 (누적된 depth 값을 카운트로 나눔)
    for (int y = 0; y < avg_depth.rows; y++) {
        for (int x = 0; x < avg_depth.cols; x++) {
            if (cnt_mat.at<int>(y, x) > 0) {
                avg_depth.at<float>(y, x) /= cnt_mat.at<int>(y, x); // 유효한 값에 대해 평균 계산
            }
        }
    }

    // 평균을 16-bit depth로 다시 변환
    avg_depth.convertTo(depth, CV_16UC1);

    // Depth와 Color 정보를 사용하여 Point Cloud 생성
    const int depth_w = avg_depth.cols;
    const int depth_h = avg_depth.rows;

    const uint16_t* depth_data = (const uint16_t*)depth.data;
    const rs2_intrinsics intr = pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();

    cloud->clear();

    for (int y = 0; y < depth_h; y++) {
        for (int x = 0; x < depth_w; x++) {
            uint16_t depth_value = depth_data[y * depth_w + x];

            float depth_in_meters = depth_value * 0.001f; // 깊이 값을 미터 단위로 변환
            float pixel[2] = { (float)x, (float)y };
            float point[3];
            rs2_deproject_pixel_to_point(point, &intr, pixel, depth_in_meters);

            if (depth_value == 0) {
                point[0] = 0;
                point[1] = 0;
                point[2] = 0;
            }

            pcl::PointXYZRGB pcl_point;
            pcl_point.x = point[0];
            pcl_point.y = point[1];
            pcl_point.z = point[2];

            // Get RGB values from color frame
            int color_x = static_cast<int>(x * color.cols / depth_w);
            int color_y = static_cast<int>(y * color.rows / depth_h);
            cv::Vec3b rgb = color.at<cv::Vec3b>(color_y, color_x);
            pcl_point.r = rgb[2];
            pcl_point.g = rgb[1];
            pcl_point.b = rgb[0];

            cloud->points.push_back(pcl_point);
        }
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;
}
