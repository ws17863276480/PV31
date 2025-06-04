#include <iostream>
#include <windows.h>
#include <opencv2/opencv.hpp>
#include "lidar_line_detection.h"

// 辅助函数：将 cv::Mat 转换为 TCMat_C
TCMat_C matToTCMat_C(const cv::Mat& mat) {
    TCMat_C tcmat;
    tcmat.rows = mat.rows;
    tcmat.cols = mat.cols;
    tcmat.type = mat.type();
    tcmat.data = mat.data;
    return tcmat;
}

int main() {
    // 设置控制台输出代码页为 UTF-8 (CP_UTF8 = 65001)
    SetConsoleOutputCP(CP_UTF8);
    // 创建 CLidarLineDetector 实例
    CLidarLineDetector* detector = CLidarLineDetector_new();

    // 初始化配置文件
    const char* configPath = "D:\\OpenCV\\Code\\PV31\\config\\roi_config.txt";
    int initResult = CLidarLineDetector_initialize(detector, configPath);
    if (initResult != static_cast<int>(ErrorCode::SUCCESS)) {
        std::cerr << "初始化失败，错误码: " << initResult << std::endl;
        CLidarLineDetector_delete(detector);
        return 1;
    }

    // 设置 SN 和输出目录
    const char* sn = "123456";
    const char* outputDir = "D:\\OpenCV\\Code\\PV31\\output";
    CLidarLineDetector_setSn(detector, sn);
    CLidarLineDetector_setOutputDir(detector, outputDir);

    // 读取导航线测试图像
    cv::Mat testImage = cv::imread("D:\\OpenCV\\Code\\PV31\\image\\pitch_true.jpg");
    if (testImage.empty()) {
        std::cerr << "无法加载测试图像" << std::endl;
        CLidarLineDetector_delete(detector);
        return 1;
    }

    // 将 cv::Mat 转换为 TCMat_C
    TCMat_C testImageC = matToTCMat_C(testImage);

    // 进行激光线检测
    TLidarLineResult_C lineResult = CLidarLineDetector_detect(detector, testImageC);
    if (lineResult.error_code != static_cast<int>(ErrorCode::SUCCESS)) {
        std::cerr << "激光线检测失败，错误码: " << lineResult.error_code << std::endl;
    } else {
        std::cout << "激光线检测结果：" << std::endl;
        std::cout << "是否检测到激光线: " << (lineResult.line_detected ? "是" : "否") << std::endl;
        std::cout << "激光线角度（弧度）: " << lineResult.line_angle << std::endl;
        std::cout << "结果图像路径: " << lineResult.image_path << std::endl;
    }

    // 读取相机测试图像
    cv::Mat cameraImage = cv::imread("D:\\OpenCV\\Code\\PV31\\image\\11.png");
    if (cameraImage.empty()) {
        std::cerr << "无法加载测试图像" << std::endl;
        CLidarLineDetector_delete(detector);
        return 1;
    }

    // 将 cv::Mat 转换为 TCMat_C
    TCMat_C cameraImageC = matToTCMat_C(cameraImage);

    // 加载标靶配置文件
    TTargetConfig_C targetConfig;
    const char* targetConfigPath = "D:\\OpenCV\\Code\\PV31\\config\\target_config.txt";
    int loadTargetResult = CLidarLineDetector_loadTargetConfig(detector, targetConfigPath, &targetConfig);
    if (loadTargetResult != static_cast<int>(ErrorCode::SUCCESS)) {
        std::cerr << "加载标靶配置文件失败，错误码: " << loadTargetResult << std::endl;
    } else {
        // 进行相机稳定性检查
        TargetMovementResult_C stabilityResult = CLidarLineDetector_checkCameraStability(detector, cameraImageC, targetConfig);
        if (stabilityResult.error_code != static_cast<int>(ErrorCode::SUCCESS)) {
            std::cerr << "相机稳定性检查失败，错误码: " << stabilityResult.error_code << std::endl;
        } else {
            std::cout << "相机稳定性检查结果：" << std::endl;
            std::cout << "相机是否稳定: " << (stabilityResult.is_stable ? "是" : "否") << std::endl;
            std::cout << "X 方向偏差: " << stabilityResult.dx << std::endl;
            std::cout << "Y 方向偏差: " << stabilityResult.dy << std::endl;
            std::cout << "总偏差: " << stabilityResult.distance << std::endl;
            std::cout << "消息: " << stabilityResult.message << std::endl;
        }
    }

    // 释放资源
    CLidarLineDetector_delete(detector);

    return 0;
}