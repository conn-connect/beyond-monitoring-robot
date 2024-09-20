#include "algorithm.h"
#include <omp.h>

int pctransform(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, Eigen::Matrix4f transform)
{

// OpenMP를 사용하여 점군을 병렬로 처리하지만 인덱스를 그대로 유지
#pragma omp parallel for
    for (std::size_t i = 0; i < cloud->points.size(); ++i) {
        const auto& pt = cloud->points[i];
        Eigen::Vector4f pt_vec(pt.x, pt.y, pt.z, 1.0f);
        Eigen::Vector4f transformed_pt = transform * pt_vec;

        // 각 포인트의 변환 결과를 동일 인덱스에 저장
        cloud->points[i].x = transformed_pt.x();
        cloud->points[i].y = transformed_pt.y();
        cloud->points[i].z = transformed_pt.z();
    }

    return 0;
}

void fitEllipseToMask(const cv::Mat& mask, double& majorAxis, double& minorAxis, cv::Point& center, cv::Point& startPoint, cv::Point& endPoint, cv::Mat& outputImage)
{
    // 입력 마스크가 비어있는지 확인
    if (mask.empty()) {
        std::cerr << "Input mask is empty." << std::endl;
        return;
    }

    // 마스크에서 경계선을 찾습니다.
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 가장 큰 경계선 찾기
    if (contours.empty()) {
        std::cerr << "No contours found in the mask." << std::endl;
        return;
    }

    std::vector<cv::Point> largestContour = contours[0];
    for (const auto& contour : contours) {
        if (cv::contourArea(contour) > cv::contourArea(largestContour)) {
            largestContour = contour;
        }
    }

    // 타원 피팅 (5개 이상의 점이 있어야 합니다)
    if (largestContour.size() < 5) {
        std::cerr << "Not enough points to fit an ellipse." << std::endl;
        return;
    }
    cv::RotatedRect fittedEllipse = cv::fitEllipse(largestContour);

    // 타원의 중심 좌표
    cv::Point2f center_temp;
    center_temp = fittedEllipse.center;
    center = cv::Point((int)center_temp.x, (int)center_temp.y);

    // 타원의 장축과 단축의 길이
    majorAxis = std::max(fittedEllipse.size.width, fittedEllipse.size.height);
    minorAxis = std::min(fittedEllipse.size.width, fittedEllipse.size.height);

    // 각도를 라디안으로 변환하고 90도를 더해서 장축의 방향을 얻습니다.
    double angleRad = (fittedEllipse.angle + 90) * CV_PI / 180.0;
    double halfMajorAxis = majorAxis / 2.0;

    // 장축의 방향 벡터를 사용하여 끝점 계산
    cv::Point2f direction(std::cos(angleRad), std::sin(angleRad));

    // 장축의 시작점과 끝점 계산
    startPoint = center_temp + cv::Point2f(-direction.x * halfMajorAxis, -direction.y * halfMajorAxis);
    endPoint = center_temp + cv::Point2f(direction.x * halfMajorAxis, direction.y * halfMajorAxis);

    // y좌표 조건을 확인하여 필요하면 교환
    if (startPoint.y > endPoint.y) {
        std::swap(startPoint, endPoint);
    }

    // 원본 이미지에 타원 그리기
    outputImage = mask.clone();
    cv::cvtColor(outputImage, outputImage, cv::COLOR_GRAY2BGR);
    cv::ellipse(outputImage, fittedEllipse, cv::Scalar(0, 255, 0), 2);

    // 장축의 시작점과 끝점을 그립니다.
    cv::circle(outputImage, startPoint, 5, cv::Scalar(255, 0, 0), -1); // 파란색
    cv::circle(outputImage, endPoint, 5, cv::Scalar(0, 0, 255), -1); // 빨간색
    cv::circle(outputImage, center, 5, cv::Scalar(0, 255, 255), -1); // 노란색 - 중심 표시
    cv::line(outputImage, startPoint, endPoint, cv::Scalar(255, 0, 0), 2); // 파란색 선으로 장축 표시
}

// 겹침 정도를 계산하는 함수
double computeOverlap(const cv::Mat& img1, const cv::Mat& img2)
{
    cv::Mat intersection;
    cv::bitwise_and(img1, img2, intersection);
    double overlap = cv::countNonZero(intersection);
    return overlap;
}

cv::Mat addPaddingToTemplate(const cv::Mat& templateMask, int paddingSize)
{
    // 패딩된 이미지 크기 계산
    int newWidth = templateMask.cols + 2 * paddingSize;
    int newHeight = templateMask.rows + 2 * paddingSize;

    // 새로운 Mat 생성 및 초기화 (검은색 배경)
    cv::Mat paddedTemplate = cv::Mat::zeros(newHeight, newWidth, templateMask.type());

    // 원본 마스크를 중앙에 복사
    cv::Rect roi(paddingSize, paddingSize, templateMask.cols, templateMask.rows);
    templateMask.copyTo(paddedTemplate(roi));

    return paddedTemplate;
}

void alignTemplateToImage(
    cv::Mat& templateMask,
    cv::Mat& targetMask,
    double& bestAngle,
    cv::Point& bestTranslation,
    cv::Mat& outputImage)
{
    double bestScale = 1.0;
    // 입력 이미지가 이진 이미지인지 확인
    CV_Assert(templateMask.type() == CV_8UC1 && targetMask.type() == CV_8UC1);

    // 변환 범위 설정
    std::vector<double> scales;
    for (double scale = 1; scale <= 1; scale += 0.05) {
        scales.push_back(scale);
    }

    std::vector<double> angles;
    for (double angle = -10; angle <= 10; angle += 1) {
        angles.push_back(angle);
    }

    int targetMask_count = 0;
    for (int x = 0; x < targetMask.cols; x++) {
        for (int y = 0; y < targetMask.rows; y++) {
            if (targetMask.at<uchar>(y, x) == 255)
                targetMask_count++;
        }
    }
    std::cout << "Target Mask Count : " << targetMask_count << std::endl;

    // 이동 범위 설정
    int maxShiftX = 30;
    int maxShiftY = 30;
    int shiftStep = 5;

    // 최적의 파라미터 초기화
    double globalMaxOverlap = -1.0;
    double globalBestAngle = 0.0;
    double globalBestScale = 1.0;
    cv::Point globalBestTranslation(0, 0);

// OpenMP를 사용하여 외부 반복문 병렬화
#pragma omp parallel for schedule(dynamic)
    for (int scaleIdx = 0; scaleIdx < scales.size(); ++scaleIdx) {
        double scale = scales[scaleIdx];

        cv::Mat templateResized;
        cv::resize(templateMask, templateResized, cv::Size(), scale, scale, cv::INTER_LINEAR);

        // int templateMask_count = 0;
        // for (int x = 0; x < templateResized.cols; x++) {
        //     for (int y = 0; y < templateResized.rows; y++) {
        //         if (templateResized.at<uchar>(y, x) == 255)
        //             templateMask_count++;
        //     }
        // }

        for (int angleIdx = 0; angleIdx < angles.size(); ++angleIdx) {
            double angle = angles[angleIdx];

            cv::Mat templateTransformed;
            cv::Mat rotationMatrix = cv::getRotationMatrix2D(
                cv::Point(templateResized.cols / 2, templateResized.rows / 2), angle, 1.0);
            cv::warpAffine(templateResized, templateTransformed, rotationMatrix, templateResized.size(), cv::INTER_LINEAR);

            double localMaxOverlap = -1.0;
            cv::Point localBestTranslation(0, 0);

            // 이동 범위 내에서 반복
            for (int dx = -maxShiftX; dx <= maxShiftX; dx += shiftStep) {
                for (int dy = -maxShiftY; dy <= maxShiftY; dy += shiftStep) {
                    // 변환된 템플릿을 타겟 이미지와 동일한 크기로 배치
                    cv::Mat transformedMask = cv::Mat::zeros(targetMask.size(), CV_8UC1);

                    int xOffset = targetMask.cols / 2 - templateTransformed.cols / 2 + dx;
                    int yOffset = targetMask.rows / 2 - templateTransformed.rows / 2 + dy;

                    int xStart = std::max(0, xOffset);
                    int yStart = std::max(0, yOffset);
                    int xEnd = std::min(targetMask.cols, xOffset + templateTransformed.cols);
                    int yEnd = std::min(targetMask.rows, yOffset + templateTransformed.rows);

                    int templateXStart = std::max(0, -xOffset);
                    int templateYStart = std::max(0, -yOffset);

                    if (xStart >= xEnd || yStart >= yEnd)
                        continue;

                    cv::Rect roiTarget(xStart, yStart, xEnd - xStart, yEnd - yStart);
                    cv::Rect roiTemplate(templateXStart, templateYStart, xEnd - xStart, yEnd - yStart);

                    templateTransformed(roiTemplate).copyTo(transformedMask(roiTarget));

                    // 겹침 정도 계산
                    double overlap = computeOverlap(transformedMask, targetMask);
                    // double diff = abs(targetMask_count - templateMask_count) / targetMask_count;
                    // if (diff > 0.01)
                    //     overlap = 0;

                    // 최대 겹침이면 업데이트
                    if (overlap > localMaxOverlap) {
                        localMaxOverlap = overlap;
                        localBestTranslation = cv::Point(xOffset, yOffset);
                    }
                }
            }

// 전역 최대값 업데이트 (동기화 필요)
#pragma omp critical
            {
                if (localMaxOverlap > globalMaxOverlap) {
                    globalMaxOverlap = localMaxOverlap;
                    globalBestAngle = angle;
                    globalBestScale = scale;
                    globalBestTranslation = localBestTranslation;
                }
            }
        }
    }

    // 최적의 파라미터 설정
    bestAngle = globalBestAngle;
    bestScale = globalBestScale;
    bestTranslation = globalBestTranslation;

    // 최적의 변환 적용하여 출력 이미지 생성
    cv::Mat templateResized, templateTransformed;
    cv::resize(templateMask, templateResized, cv::Size(), bestScale, bestScale, cv::INTER_LINEAR);

    cv::Mat rotationMatrix = cv::getRotationMatrix2D(
        cv::Point(templateResized.cols / 2, templateResized.rows / 2), bestAngle, 1.0);
    cv::warpAffine(templateResized, templateTransformed, rotationMatrix, templateResized.size(), cv::INTER_LINEAR);

    cv::Mat transformedMask = cv::Mat::zeros(targetMask.size(), CV_8UC1);

    int dx = bestTranslation.x;
    int dy = bestTranslation.y;

    int xStart = std::max(0, dx);
    int yStart = std::max(0, dy);
    int xEnd = std::min(targetMask.cols, dx + templateTransformed.cols);
    int yEnd = std::min(targetMask.rows, dy + templateTransformed.rows);

    int templateXStart = std::max(0, -dx);
    int templateYStart = std::max(0, -dy);

    if (xStart < xEnd && yStart < yEnd) {
        cv::Rect roiTarget(xStart, yStart, xEnd - xStart, yEnd - yStart);
        cv::Rect roiTemplate(templateXStart, templateYStart, xEnd - xStart, yEnd - yStart);

        templateTransformed(roiTemplate).copyTo(transformedMask(roiTarget));
    }

    // 결과 이미지 생성
    cv::Mat coloredTarget;
    cv::cvtColor(targetMask, coloredTarget, cv::COLOR_GRAY2BGR);
    cv::Mat coloredTransformed;
    cv::cvtColor(transformedMask, coloredTransformed, cv::COLOR_GRAY2BGR);

    // 겹치는 영역을 녹색으로 표시
    cv::Mat overlapImage;
    cv::addWeighted(coloredTarget, 0.5, coloredTransformed, 0.5, 0, overlapImage);

    std::cout << "[Algo] Best angle: " << bestAngle << ", Best Scale: " << bestScale
              << ", Best translation: " << bestTranslation << ", Overlap: " << globalMaxOverlap << std::endl;

    outputImage = overlapImage;
}