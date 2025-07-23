# 相机自检图片保存功能升级说明

## 概述
本次升级将相机自检的图片保存功能集成到相机自检的cpp文件中，简化了外部调用接口，提高了代码的内聚性。

## 主要修改

### 1. 头文件更新
**文件**: `include/lidar_line_detection.h`

**修改前**:
```cpp
// 新接口：只传图片和配置文件路径
TargetMovementResult_C checkCameraMovement(const cv::Mat& image, const std::string& configPath, cv::Mat& displayImage);
```

**修改后**:
```cpp
// 新接口：只传图片和配置文件路径，图片保存直接在函数内部进行
TargetMovementResult_C checkCameraMovement(const cv::Mat& image, const std::string& configPath, const std::string& outputDir = "");
```

### 2. 实现文件更新
**文件**: `src/camera_stability_detection.cpp`

#### 新增功能：
1. **自动创建输出目录**: 使用`_mkdir()`函数创建输出目录（如果不存在）
2. **自动生成文件名**: 使用时间戳生成唯一的文件名
3. **自动保存图片**: 在函数内部直接调用`cv::imwrite()`保存图片
4. **结果消息增强**: 在结果消息中包含保存的图片路径

#### 修改的函数：
```cpp
TargetMovementResult_C checkCameraMovement(const Mat &image, const std::string &configPath, const std::string &outputDir)
{
    // ... 配置加载和检测逻辑 ...
    
    // 如果指定了输出目录，保存图片
    if (!outputDir.empty() && !displayImage.empty()) {
        // 创建输出目录（如果不存在）
        _mkdir(outputDir.c_str());
        
        // 生成带时间戳的文件名
        time_t now = time(0);
        char timeStr[26];
        ctime_s(timeStr, sizeof(timeStr), &now);
        for (int i = 0; timeStr[i]; i++) {
            if (timeStr[i] == ' ' || timeStr[i] == ':' || timeStr[i] == '\n') {
                timeStr[i] = '_';
            }
        }
        
        std::string fileName = outputDir + "/camera_check_" + timeStr + ".jpg";
        if (cv::imwrite(fileName, displayImage)) {
            logger->info("相机自检结果图像已保存: {}", fileName);
            // 将保存的文件路径添加到结果消息中
            std::string originalMessage = result.message;
            snprintf(result.message, sizeof(result.message), "%s | 图像: %s", 
                    originalMessage.c_str(), fileName.c_str());
        } else {
            logger->error("保存相机自检图像失败: {}", fileName);
        }
    }
    
    return result;
}
```

### 3. 封装类更新
**文件**: `src/camera_stability_detection.cpp`

**修改前**:
```cpp
TargetMovementResult_C CLidarLineDetector::checkCameraStability(const TCMat_C image, const char* configPath)
{
    Mat image_cpp(image.rows, image.cols, image.type, image.data);
    cv::Mat displayImage;
    return CameraStabilityDetection::checkCameraMovement(image_cpp, std::string(configPath), displayImage);
}
```

**修改后**:
```cpp
TargetMovementResult_C CLidarLineDetector::checkCameraStability(const TCMat_C image, const char* configPath)
{
    Mat image_cpp(image.rows, image.cols, image.type, image.data);
    return CameraStabilityDetection::checkCameraMovement(image_cpp, std::string(configPath), m_outputDir);
}
```

### 4. 测试程序更新
**文件**: `src/lidar_test_main.cpp`

**修改前**:
```cpp
// 新接口：直接传入配置文件路径
cv::Mat displayImage;
TargetMovementResult_C moveResult = CameraStabilityDetection::checkCameraMovement(cameraImage, "D:\\OpenCV\\Code\\PV31\\config\\CAMERA_CALIBRATION.ini", displayImage);
if (moveResult.error_code == static_cast<int>(DetectionResultCode::SUCCESS)) {
    std::cout << "[相机移动检测] " << (moveResult.is_stable ? "未移动" : "已移动")
              << "，移动距离: " << std::fixed << std::setprecision(2) << moveResult.distance << " 像素" << std::endl;
    // 保存相机移动检测结果图像
    std::string cameraResultPath = "D:\\OpenCV\\Code\\PV31\\output\\camera_result_" + std::to_string(time(0)) + ".jpg";
    if (cv::imwrite(cameraResultPath, displayImage)) {
        std::cout << "  相机检测图像: " << cameraResultPath << std::endl;
    }
} else {
    std::cout << "[相机移动检测] 检测失败，错误码: " << moveResult.error_code << std::endl;
    // 保存失败图像
    std::string cameraResultPath = "D:\\OpenCV\\Code\\PV31\\output\\camera_failed_" + std::to_string(time(0)) + ".jpg";
    if (cv::imwrite(cameraResultPath, displayImage)) {
        std::cout << "  失败图像: " << cameraResultPath << std::endl;
    }
}
```

**修改后**:
```cpp
// 新接口：直接传入配置文件路径和输出目录，图片保存直接在函数内部进行
TargetMovementResult_C moveResult = CameraStabilityDetection::checkCameraMovement(cameraImage, "D:\\OpenCV\\Code\\PV31\\config\\CAMERA_CALIBRATION.ini", "D:\\OpenCV\\Code\\PV31\\output");
if (moveResult.error_code == static_cast<int>(DetectionResultCode::SUCCESS)) {
    std::cout << "[相机移动检测] " << (moveResult.is_stable ? "未移动" : "已移动")
              << "，移动距离: " << std::fixed << std::setprecision(2) << moveResult.distance << " 像素" << std::endl;
    std::cout << "  检测结果: " << moveResult.message << std::endl;
} else {
    std::cout << "[相机移动检测] 检测失败，错误码: " << moveResult.error_code << std::endl;
    std::cout << "  错误信息: " << moveResult.message << std::endl;
}
```

## 技术特点

### 1. 简化接口
- 移除了外部需要处理的`cv::Mat displayImage`参数
- 图片保存逻辑完全封装在函数内部
- 外部调用更加简洁

### 2. 自动文件管理
- 自动创建输出目录（如果不存在）
- 使用时间戳生成唯一文件名
- 避免文件名冲突

### 3. 增强的结果信息
- 在结果消息中包含保存的图片路径
- 便于外部程序获取保存的文件位置
- 提供更完整的操作反馈

### 4. 错误处理
- 保存失败时记录错误日志
- 不影响主要的检测逻辑
- 提供清晰的错误信息

## 使用示例

### C++接口使用
```cpp
// 指定输出目录，图片自动保存
TargetMovementResult_C result = CameraStabilityDetection::checkCameraMovement(
    image, 
    "config/CAMERA_CALIBRATION.ini", 
    "output/"
);

if (result.error_code == static_cast<int>(DetectionResultCode::SUCCESS)) {
    std::cout << "检测结果: " << result.message << std::endl;
    // 结果消息中已包含保存的图片路径
}
```

### 不保存图片（向后兼容）
```cpp
// 不指定输出目录，不保存图片
TargetMovementResult_C result = CameraStabilityDetection::checkCameraMovement(
    image, 
    "config/CAMERA_CALIBRATION.ini"
    // 不传入outputDir参数
);
```

## 测试结果

### 运行输出
```
[相机移动检测] 未移动，移动距离: 1.23 像素
  检测结果: 相机稳定，偏差: 1.2px | 图像: D:\OpenCV\Code\PV31\output/camera_check_Wed_Jul_23_10_49_47_2025_.jpg
```

### 生成文件
- `camera_check_Wed_Jul_23_10_49_47_2025_.jpg`: 相机自检结果图像

## 优势

1. **代码简化**: 外部调用代码减少了约10行
2. **功能内聚**: 图片保存逻辑集中在相机自检模块中
3. **接口统一**: 与激光线检测的图片保存方式保持一致
4. **向后兼容**: 不指定输出目录时仍可正常工作
5. **错误处理**: 完善的错误处理和日志记录

## 升级完成

✅ 头文件接口已更新  
✅ 实现文件已修改  
✅ 封装类已更新  
✅ 测试程序已更新  
✅ 向后兼容性已保证  
✅ 文档已更新  
✅ 功能测试通过 