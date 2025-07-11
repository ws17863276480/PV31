# 激光线检测项目结构说明

## 项目概述
本项目实现了激光线检测和相机自检两个核心功能，现已将功能拆分为独立的文件以提高代码组织性和可维护性。

## 文件结构

### 头文件
- `include/lidar_line_detection.h` - 主要头文件，包含所有接口定义和数据结构

### 源文件
- `src/lidar_line_detection.cpp` - **激光线检测功能实现**
  - ROI配置读取
  - 激光线检测核心算法
  - 图像处理和保存
  - 版本信息管理
  - C++封装类和C接口实现

- `src/camera_stability_detection.cpp` - **相机自检功能实现**
  - 标靶配置读取
  - 标靶中心点检测
  - 相机移动检测
  - 相机自检相关C接口实现

- `src/lidar_test_main.cpp` - 测试主程序
  - 演示激光线检测功能
  - 演示相机自检功能
  - 统一的测试接口

## 功能分工

### 激光线检测模块 (`LidarLineDetector` 命名空间)
- **ROI配置管理**: 读取和验证ROI配置
- **激光线检测**: 核心检测算法，包括图像预处理、边缘检测、霍夫变换
- **结果输出**: 角度计算和结果图像保存
- **版本管理**: 库版本信息

### 相机自检模块 (`CameraStabilityDetection` 命名空间)
- **标靶配置**: 读取标靶中心点和容差配置
- **标靶检测**: 图像中检测标靶矩形并计算中心点
- **移动检测**: 比较当前中心点与期望中心点的偏差
- **稳定性判断**: 根据容差判断相机是否稳定

## 接口设计

### 统一错误码
使用 `DetectionResultCode` 枚举统一管理所有错误状态：
- `SUCCESS` - 操作成功
- `NOT_FOUND` - 未检测到目标
- `OUT_OF_ROI` - 目标超出ROI范围
- `CONFIG_LOAD_FAILED` - 配置加载失败
- `IMAGE_SAVE_FAILED` - 图像保存失败
- `UNKNOWN_ERROR` - 未知错误

### 数据结构
- `LidarDetectionResult` - 激光线检测结果
- `TargetMovementResult_C` - 相机移动检测结果
- `ROI` - ROI配置结构
- `TargetConfig` - 标靶配置结构

## 编译配置
- `CMakeLists.txt` - 构建配置，包含两个源文件
- 支持生成可执行文件和共享库
- 链接OpenCV库

## 使用示例
```cpp
// 激光线检测
LidarDetectionResult result = LidarLineDetector::detectLidarLine(image, roi, sn, outputDir);

// 相机自检
TargetMovementResult_C moveResult = CameraStabilityDetection::checkCameraMovement(image, config, displayImage);
```
 
## 优势
1. **模块化设计**: 功能清晰分离，便于维护
2. **命名空间隔离**: 避免函数名冲突
3. **统一接口**: 使用相同的错误码和数据结构
4. **独立编译**: 可以单独编译和测试各个模块
5. **扩展性好**: 便于添加新功能或修改现有功能 

cmake --build .; cd ..; .\build\TestLidarLineDetection.exe 