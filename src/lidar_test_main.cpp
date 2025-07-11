#include <iostream>
#include <windows.h>
#include <opencv2/opencv.hpp>
#include "lidar_line_detection.h"
#include <iomanip> // Required for std::fixed and std::setprecision
#include <ctime> // Required for time()
#ifdef _WIN32
#include <direct.h>
#else
#include <sys/stat.h>
#include <sys/types.h>
#endif

int main() {
    SetConsoleOutputCP(CP_UTF8);
    std::cout << "激光线检测库 v" << LidarLineDetector::getVersionMajor() << "."
              << LidarLineDetector::getVersionMinor() << "."
              << LidarLineDetector::getVersionPatch() << std::endl;

    // 读取ROI配置
    LidarLineDetector::ROI roi;
    std::string roiConfigPath = "D:\\OpenCV\\Code\\PV31\\config\\roi_config.txt";
    DetectionResultCode roiConfigResult = LidarLineDetector::readROIFromConfig(roiConfigPath, roi);
    if (roiConfigResult != DetectionResultCode::SUCCESS) {
        std::cout << "[错误] ROI配置读取失败，错误码: " << static_cast<int>(roiConfigResult) << std::endl;
        return 1;
    }

    // 激光线检测
    cv::Mat testImage = cv::imread("D:\\OpenCV\\Code\\PV31\\image\\111.jpg");
    if (testImage.empty()) {
        std::cout << "[错误] 无法加载激光线测试图像" << std::endl;
        return 1;
    }
    std::string outputDir = "D:\\OpenCV\\Code\\PV31\\output";
    if (!outputDir.empty()) {
#ifdef _WIN32
        _mkdir(outputDir.c_str());
#else
        mkdir(outputDir.c_str(), 0755);
#endif
    }
    LidarDetectionResult result = LidarLineDetector::detectLidarLine(testImage, roi, "123456", outputDir);
    if (result.status == DetectionResultCode::SUCCESS) {
        std::cout << "[激光线检测] 成功\n  角度(弧度): " << result.line_angle
                  << "\n  结果图像: " << result.image_path << std::endl;
    } else {
        std::cout << "[激光线检测] 失败，错误码: " << static_cast<int>(result.status);
        if (!result.image_path.empty()) {
            std::cout << "\n  失败图像: " << result.image_path;
        }
        std::cout << std::endl;
    }

    // 相机移动检测
    cv::Mat cameraImage = cv::imread("D:\\OpenCV\\Code\\PV31\\image\\111.jpg");
    if (cameraImage.empty()) {
        std::cout << "[错误] 无法加载相机测试图像" << std::endl;
        return 1;
    }
    LidarLineDetector::TargetConfig targetConfig;
    DetectionResultCode targetConfigResult = CameraStabilityDetection::loadTargetConfig(
        "D:\\OpenCV\\Code\\PV31\\config\\target_config.txt", targetConfig);
    if (targetConfigResult != DetectionResultCode::SUCCESS) {
        std::cout << "[错误] 标靶配置读取失败，错误码: " << static_cast<int>(targetConfigResult) << std::endl;
        return 1;
    }
    cv::Mat displayImage;
    TargetMovementResult_C moveResult = CameraStabilityDetection::checkCameraMovement(cameraImage, targetConfig, displayImage);
    if (moveResult.error_code == static_cast<int>(DetectionResultCode::SUCCESS)) {
        std::cout << "[相机移动检测] " << (moveResult.is_stable ? "未移动" : "已移动")
                  << "，移动距离: " << std::fixed << std::setprecision(2) << moveResult.distance << " 像素" << std::endl;
        // 保存相机移动检测结果图像
        std::string cameraResultPath = outputDir + "\\camera_result_" + std::to_string(time(0)) + ".jpg";
        if (cv::imwrite(cameraResultPath, displayImage)) {
            std::cout << "  相机检测图像: " << cameraResultPath << std::endl;
        }
    } else {
        std::cout << "[相机移动检测] 检测失败，错误码: " << moveResult.error_code << std::endl;
        // 保存失败图像
        std::string cameraResultPath = outputDir + "\\camera_failed_" + std::to_string(time(0)) + ".jpg";
        if (cv::imwrite(cameraResultPath, displayImage)) {
            std::cout << "  失败图像: " << cameraResultPath << std::endl;
        }
    }
    return 0;
}    