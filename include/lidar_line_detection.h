#pragma once

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <ctime>
#include <cstdint>
#include <cmath>

// 宏定义（在实现文件中使用导出标记）
#ifdef _WIN32
    #define Smpclass_API __declspec(dllexport)
#else
    #define Smpclass_API
#endif

// 错误码枚举
enum class ErrorCode {
    SUCCESS = 0,
    IMAGE_LOAD_FAILED = 1,
    CONFIG_LOAD_FAILED = 2,
    ROI_INVALID = 3,
    LINE_DETECTION_FAILED = 4,
    IMAGE_SAVE_FAILED = 5,
    BITMAP_CONVERSION_FAILED = 6
};

// C接口数据结构
#pragma pack(push, 1)
struct TCMat_C {
    int rows;
    int cols;
    int type;
    void* data;
};

struct TROIConfig_C {
    int x;
    int y;
    int width;
    int height;
};

struct TLidarLineResult_C {
    bool line_detected;
    float line_angle; // 直线角度（弧度）
    char image_path[256];
    int error_code; // 修正为int类型
};

struct TTargetConfig_C {
    float center_x;
    float center_y;
    float tolerance; // 允许的像素偏差
};

struct TargetMovementResult_C {
    int is_stable;
    float dx;
    float dy;
    float distance;
    int error_code;
    char message[256];
};
#pragma pack(pop)

// 封装类定义
class CLidarLineDetector {
public:
    CLidarLineDetector();
    ~CLidarLineDetector();

    ErrorCode initialize(const char* configPath);
    void setROI(int x, int y, int width, int height);
    void setSn(const char* sn);
    void setOutputDir(const char* outputDir);
    TLidarLineResult_C detect(const TCMat_C image);
    ErrorCode loadTargetConfig(const char* configPath, TTargetConfig_C* config);
    TargetMovementResult_C checkCameraStability(const TCMat_C image, const TTargetConfig_C config);
};

// C 接口定义
extern "C" {
    Smpclass_API CLidarLineDetector* CLidarLineDetector_new();
    Smpclass_API void CLidarLineDetector_delete(CLidarLineDetector* instance);
    Smpclass_API int CLidarLineDetector_initialize(CLidarLineDetector* instance, const char* configPath);
    Smpclass_API void CLidarLineDetector_setROI(CLidarLineDetector* instance, int x, int y, int width, int height);
    Smpclass_API void CLidarLineDetector_setSn(CLidarLineDetector* instance, const char* sn);
    Smpclass_API void CLidarLineDetector_setOutputDir(CLidarLineDetector* instance, const char* outputDir);
    Smpclass_API TLidarLineResult_C CLidarLineDetector_detect(CLidarLineDetector* instance, const TCMat_C image);
    Smpclass_API int CLidarLineDetector_loadTargetConfig(CLidarLineDetector* instance, const char* configPath, TTargetConfig_C* config);
    Smpclass_API TargetMovementResult_C CLidarLineDetector_checkCameraStability(CLidarLineDetector* instance, const TCMat_C image, const TTargetConfig_C config);
}