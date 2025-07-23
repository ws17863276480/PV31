# PV31 激光线检测与相机自检系统

## 项目简介

本项目实现了基于OpenCV的激光线检测与相机自检功能，支持**四边形ROI**灵活区域定义，具备高鲁棒性和可维护性。  
代码结构清晰，支持C++/C接口，易于集成和二次开发。

---

## 快速开始

### 1. 编译

```bash
cmake -G "MinGW Makefiles" -DCMAKE_C_COMPILER=D:/openCV/mingw64-8.1.0/bin/gcc.exe -DCMAKE_CXX_COMPILER=D:/openCV/mingw64-8.1.0/bin/g++.exe -DOpenCV_DIR=D:/openCV/OpenCV-MinGW-Build-4.5.2/x64/mingw/lib .
mingw32-make
```

### 2. 运行测试

```bash
# 激光线检测主程序
./TestLidarLineDetection.exe

# 四边形ROI测试
./QuadROITest.exe
```

---

## 主要功能

- **激光线检测**：支持矩形和四边形ROI，自动拟合直线，输出角度、端点、物理仰角等。
- **相机自检**：检测标靶中心，判断相机是否移动，自动保存检测图片。
- **配置灵活**：ROI、物理参数、阈值均可通过配置文件调整。
- **日志与可视化**：详细日志输出，自动生成检测结果和调试图片。

---

## 目录结构

- `include/lidar_line_detection.h` 头文件，所有接口和数据结构定义
- `src/lidar_line_detection.cpp` 激光线检测实现
- `src/camera_stability_detection.cpp` 相机自检实现
- `src/lidar_test_main.cpp` 主测试程序
- `src/quad_roi_test.cpp` 四边形ROI测试
- `config/ROI_CONFIG.ini` ROI配置（支持四边形）
- `output/` 检测结果图片、日志等

---

## 四边形ROI配置示例

```ini
# 四边形ROI配置 - 四个坐标点 (x, y)
# 格式：X1, X2, X3, X4 按顺时针或逆时针顺序
X1: 322, 548
X2: 1601, 587
X3: 1604, 632
X4: 322, 619
```

---

## 典型用法

```cpp
// 自动检测四边形ROI
LidarLineDetector::LidarLineResult result =
    LidarLineDetector::detect(image, configPath, sn, outputDir);
```

---

## 主要升级与文档

### 1. [四边形ROI激光线检测升级说明](QUAD_ROI_UPGRADE.md)
- 支持四边形ROI配置与检测
- 自动兼容老格式矩形ROI
- 详细配置与代码示例

### 2. [相机自检图片保存功能升级说明](CAMERA_SAVE_UPGRADE.md)
- 相机自检图片自动保存
- 输出目录和文件名自动管理
- 结果信息增强

### 3. [项目结构说明](PROJECT_STRUCTURE.md)
- 详细文件结构与模块分工
- 主要接口与数据结构说明

### 4. [头文件重新整理说明](HEADER_REORGANIZATION.md)
- 头文件分区、注释、依赖顺序优化
- 便于维护和扩展

---

## 维护建议

- 新增功能时，按现有分组结构添加代码，保持注释风格统一
- 修改现有代码时，保持结构和风格，更新相关注释
- 代码审查时，关注结构清晰、注释完整、依赖正确

---

## 附录

---

### QUAD_ROI_UPGRADE.md

