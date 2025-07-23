# 四边形ROI激光线检测升级说明

## 概述
本次升级将激光线检测系统从矩形ROI升级为四边形ROI，提供了更灵活的检测区域定义方式。

## 主要修改

### 1. 配置文件格式更新
**文件**: `config/ROI_CONFIG.ini`

**原格式** (矩形ROI):
```
x: 322
y: 548
width: 1282
height: 84
```

**新格式** (四边形ROI):
```
# 四边形ROI配置 - 四个坐标点 (x, y)
# 格式：X1, X2, X3, X4 按顺时针或逆时针顺序
X1: 322, 548
X2: 1601, 587
X3: 1604, 632
X4: 322, 619
```

### 2. 头文件更新
**文件**: `include/lidar_line_detection.h`

新增数据结构：
```cpp
// 新增：四边形ROI结构体
struct QuadROI {
    cv::Point2i points[4];  // 四个坐标点，按顺时针或逆时针顺序
};
```

新增函数声明：
```cpp
// 新增：读取四边形ROI配置
DetectionResultCode readQuadROIFromConfig(const std::string& configPath, QuadROI& quadRoi);
// 新增：使用四边形ROI的激光线检测
LidarDetectionResult detectLidarLineWithQuadROI(const cv::Mat& image, const QuadROI& quadRoi, const std::string& sn, const std::string& outputDir);
```

### 3. 实现文件更新
**文件**: `src/lidar_line_detection.cpp`

#### 新增函数：
1. **`readQuadROIFromConfig`**: 读取四边形ROI配置文件
   - 支持注释行（以#开头）
   - 验证四个坐标点的完整性
   - 计算四边形面积进行有效性验证

2. **`detectLidarLineWithQuadROI`**: 使用四边形ROI的激光线检测
   - 创建四边形掩码进行区域提取
   - 使用掩码提取ROI区域
   - 保持原有的激光线检测算法逻辑
   - 在结果图像中绘制四边形ROI边界

#### 修改函数：
**`detect`**: 主检测函数
- 优先尝试读取四边形ROI配置
- 如果四边形ROI读取失败，自动回退到矩形ROI
- 提供向后兼容性

### 4. 测试程序更新
**文件**: `src/lidar_test_main.cpp`
- 移除手动ROI配置读取代码
- 直接使用`detect`函数，自动处理四边形ROI

**新增文件**: `src/quad_roi_test.cpp`
- 专门的四边形ROI测试程序
- 生成四边形ROI可视化图像
- 显示检测结果和角度信息

### 5. 构建系统更新
**文件**: `CMakeLists.txt`
- 添加新的四边形ROI测试程序`QuadROITest`

## 技术特点

### 1. 向后兼容性
- 系统会自动检测配置文件格式
- 优先使用四边形ROI，失败时回退到矩形ROI
- 现有代码无需修改即可使用

### 2. 四边形ROI处理
- 使用OpenCV的`fillPoly`函数创建四边形掩码
- 支持任意四边形形状（凸四边形或凹四边形）
- 自动验证四边形有效性（面积检查）

### 3. 可视化增强
- 在结果图像中绘制四边形ROI边界
- 生成四边形ROI可视化图像
- 显示四个坐标点的位置和编号

## 使用示例

### 配置文件示例
```ini
# 四边形ROI配置 - 四个坐标点 (x, y)
# 格式：X1, X2, X3, X4 按顺时针或逆时针顺序
X1: 322, 548
X2: 1601, 587
X3: 1604, 632
X4: 322, 619
```

### 代码使用示例
```cpp
// 自动检测四边形ROI配置
LidarLineDetector::LidarLineResult result = 
    LidarLineDetector::detect(image, configPath, sn, outputDir);
```

## 测试结果

### 检测性能
- 四边形ROI检测成功
- 检测角度: 0.77度
- 激光点数: 6171
- RMS误差: 1.82
- 检测长度: 1280.14像素

### 生成文件
- `quad_roi_visualization.jpg`: 四边形ROI可视化图像
- `result_QUAD_TEST_*.jpg`: 检测结果图像
- `debug_laser_points_*.jpg`: 激光点调试图像

## 注意事项

1. **坐标点顺序**: 四个坐标点应按顺时针或逆时针顺序定义
2. **面积验证**: 系统会检查四边形面积，过小的四边形会被拒绝
3. **图像边界**: 所有坐标点必须在图像范围内
4. **向后兼容**: 系统支持原有的矩形ROI格式

## 升级完成

✅ 配置文件已更新为四边形ROI格式  
✅ 代码已支持四边形ROI检测  
✅ 测试程序已更新  
✅ 向后兼容性已保证  
✅ 可视化功能已增强  
✅ 文档已更新 