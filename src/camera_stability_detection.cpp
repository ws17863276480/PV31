#include "lidar_line_detection.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"

using namespace cv;
using namespace std;

// 相机自检相关功能实现
namespace CameraStabilityDetection {

    static std::shared_ptr<spdlog::logger> logger = spdlog::basic_logger_mt("camera_logger", "log/camera_stability_detection.log");

    // 标靶配置文件 读取
    DetectionResultCode loadTargetConfig(const string &configPath, LidarLineDetector::TargetConfig &config)
    {
        logger->info("开始读取标靶配置文件: {}", configPath);
        ifstream file(configPath);
        if (!file.is_open())
        {
            logger->error("无法打开配置文件: {}", configPath);
            return DetectionResultCode::CONFIG_LOAD_FAILED;
        }

        string line;
        bool centerRead = false, toleranceRead = false;
        while (getline(file, line))
        {
            if (line.find("center_x:") == 0)
            {
                if (sscanf(line.c_str(), "center_x: %f", &config.expected_center.x) == 1)
                {
                    centerRead = true;
                }
                else
                {
                    logger->error("解析 center_x 失败: {}", line);
                }
            }
            else if (line.find("center_y:") == 0)
            {
                if (sscanf(line.c_str(), "center_y: %f", &config.expected_center.y) == 1)
                {
                    centerRead = true;
                }
                else
                {
                    logger->error("解析 center_y 失败: {}", line);
                }
            }
            else if (line.find("tolerance:") == 0)
            {
                if (sscanf(line.c_str(), "tolerance: %f", &config.tolerance) == 1)
                {
                    toleranceRead = true;
                }
                else
                {
                    logger->error("解析 tolerance 失败: {}", line);
                }
            }
        }
        file.close();

        if (!centerRead || !toleranceRead)
        {
            logger->error("配置文件格式错误，未读取到完整信息。");
        }
        
        if (centerRead && toleranceRead) {
            logger->info("标靶配置读取成功: center=({:.1f}, {:.1f}), tolerance={:.1f}", 
                        config.expected_center.x, config.expected_center.y, config.tolerance);
            return DetectionResultCode::SUCCESS;
        } else {
            return DetectionResultCode::CONFIG_LOAD_FAILED;
        }
    }

    // 检测标靶四个角落的黑色方块
    bool detectTarget(const Mat& image, vector<Point2f>& corners, Mat& displayImage) {
        logger->info("开始检测标靶四个角落的黑色方块");
        Mat gray, binary;
        cvtColor(image, gray, COLOR_BGR2GRAY);
        threshold(gray, binary, 80, 255, THRESH_BINARY_INV);
        
        // 形态学操作去噪
        Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
        morphologyEx(binary, binary, MORPH_OPEN, kernel);
        morphologyEx(binary, binary, MORPH_CLOSE, kernel);
        
        vector<vector<Point>> contours;
        findContours(binary, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        
        vector<Point2f> centers;
        for (const auto& contour : contours) {
            double area = contourArea(contour);
            if (area < 2000 || area > 50000) continue;
            
            vector<Point> approx;
            approxPolyDP(contour, approx, arcLength(contour, true) * 0.02, true);
            if (approx.size() == 4 && isContourConvex(approx)) {
                Rect rect = boundingRect(approx);
                double aspect = (double)rect.width / rect.height;
                if (aspect > 0.7 && aspect < 1.3) {
                    Moments m = moments(contour);
                    if (m.m00 != 0) {
                        Point2f center(m.m10/m.m00, m.m01/m.m00);
                        centers.push_back(center);
                        // 在显示图像上绘制检测到的方块
                        circle(displayImage, center, 8, Scalar(0, 255, 0), 2);
                        rectangle(displayImage, rect, Scalar(0, 255, 0), 2);
                    }
                }
            }
        }
        
        if (centers.size() != 4) {
            logger->warn("未能检测到4个标靶方块，找到: {}", centers.size());
            cv::putText(displayImage, "Target Detection Failed: " + std::to_string(centers.size()) + " targets found", 
                       cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
            return false;
        }
        
        // 按位置排序：左上、右上、左下、右下
        sort(centers.begin(), centers.end(), [](const Point2f& a, const Point2f& b) {
            return a.y < b.y || (a.y == b.y && a.x < b.x);
        });
        if (centers[0].x > centers[1].x) swap(centers[0], centers[1]);
        if (centers[2].x > centers[3].x) swap(centers[2], centers[3]);
        
        corners = centers;
        logger->info("成功检测到4个标靶方块");
        return true;
    }

    // 计算标靶中心点
    Point2f calculateTargetCenter(const vector<Point2f>& corners) {
        if (corners.size() != 4) return Point2f(-1, -1);
        float centerX = (corners[0].x + corners[1].x + corners[2].x + corners[3].x) / 4.0f;
        float centerY = (corners[0].y + corners[1].y + corners[2].y + corners[3].y) / 4.0f;
        return Point2f(centerX, centerY);
    }

    // 标靶中心点检测
    DetectionResultCode detectTargetCenter(const Mat &image, Point2f &outCenter, Mat &displayImage)
    {
        logger->info("开始标靶中心点检测");
        displayImage = image.clone();
        
        vector<Point2f> corners;
        if (!detectTarget(image, corners, displayImage)) {
            return DetectionResultCode::CAMERA_SELF_CHECK_FAILED;
        }
        
        outCenter = calculateTargetCenter(corners);
        if (outCenter.x < 0 || outCenter.y < 0) {
            logger->error("计算标靶中心点失败");
            cv::putText(displayImage, "Center Calculation Failed", 
                       cv::Point(20, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
            return DetectionResultCode::CAMERA_SELF_CHECK_FAILED;
        }
        
        // 在显示图像上绘制中心点
        circle(displayImage, outCenter, 10, Scalar(0, 0, 255), -1);
        circle(displayImage, outCenter, 15, Scalar(0, 0, 255), 2);
        
        logger->info("标靶中心点检测成功: ({:.1f}, {:.1f})", outCenter.x, outCenter.y);
        return DetectionResultCode::SUCCESS;
    }

    // 相机自检函数
    TargetMovementResult_C checkCameraMovement(const Mat &image, const LidarLineDetector::TargetConfig &config, Mat &displayImage)
    {
        logger->info("开始相机移动检测");
        // 修复：显式转换枚举类型
        TargetMovementResult_C result{0, 0, 0, 0, static_cast<int>(DetectionResultCode::SUCCESS), ""};
        Point2f currentCenter;
        DetectionResultCode err = detectTargetCenter(image, currentCenter, displayImage);
        if (err != DetectionResultCode::SUCCESS)
        {
            result.error_code = static_cast<int>(err);
            snprintf(result.message, sizeof(result.message), "标靶检测失败: %d", result.error_code);
            logger->error("标靶检测失败，错误码: {}", result.error_code);
            // 在显示图像上标注失败原因
            cv::putText(displayImage, "Target Detection Failed", cv::Point(20, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
            return result;
        }

        float dx = currentCenter.x - config.expected_center.x;
        float dy = currentCenter.y - config.expected_center.y;
        result.dx = dx;
        result.dy = dy;
        result.distance = sqrt(dx * dx + dy * dy);
        result.is_stable = (result.distance <= config.tolerance);

        snprintf(result.message, sizeof(result.message),
                 result.is_stable ? "相机稳定，偏差: %.1fpx" : "相机移动！偏差: %.1fpx (>%.1fpx)",
                 result.distance, result.distance, config.tolerance);

        // 在显示图像上绘制检测结果
        circle(displayImage, config.expected_center, (int)config.tolerance, Scalar(255, 0, 0), 2);
        line(displayImage, config.expected_center, currentCenter, Scalar(0, 255, 255), 2);
        putText(displayImage, result.message, Point(20, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);

        logger->info("相机移动检测完成: {} (距离: {:.1f}px)", 
                    result.is_stable ? "稳定" : "移动", result.distance);
        return result;
    }

} // namespace CameraStabilityDetection

// 封装类实现 - 相机自检相关方法
DetectionResultCode CLidarLineDetector::loadTargetConfig(const char *configPath, LidarLineDetector::TargetConfig &config)
{
    return CameraStabilityDetection::loadTargetConfig(configPath, config);
}

TargetMovementResult_C CLidarLineDetector::checkCameraStability(const TCMat_C image, const TTargetConfig_C config)
{
    Mat image_cpp(image.rows, image.cols, image.type, image.data);
    LidarLineDetector::TargetConfig internalConfig{
        Point2f(config.center_x, config.center_y),
        config.tolerance};
    Mat displayImage;
    return CameraStabilityDetection::checkCameraMovement(image_cpp, internalConfig, displayImage);
}

// C 接口实现 - 相机自检相关
extern "C"
{
    Smpclass_API DetectionResultCode CLidarLineDetector_loadTargetConfig(CLidarLineDetector *instance, const char *configPath, TTargetConfig_C *config)
    {
        LidarLineDetector::TargetConfig internalConfig;
        auto err = instance->loadTargetConfig(configPath, internalConfig);
        if (err == DetectionResultCode::SUCCESS)
        {
            config->center_x = internalConfig.expected_center.x;
            config->center_y = internalConfig.expected_center.y;
            config->tolerance = internalConfig.tolerance;
        }
        return err;
    }

    Smpclass_API TargetMovementResult_C CLidarLineDetector_checkCameraStability(CLidarLineDetector *instance, const TCMat_C image, const TTargetConfig_C config)
    {
        return instance->checkCameraStability(image, config);
    }
} 