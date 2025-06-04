#ifndef LIDAR_LINE_DETECTION_H
#define LIDAR_LINE_DETECTION_H

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

// 版本信息
#define LIDAR_LINE_DETECTION_VERSION_MAJOR 1
#define LIDAR_LINE_DETECTION_VERSION_MINOR 0
#define LIDAR_LINE_DETECTION_VERSION_PATCH 0

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

// 版本信息结构 - 移至错误码定义之后，确保所有依赖都已定义
struct VersionInfo {
    int major;
    int minor;
    int patch;
    const char* versionString;
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

// 宏定义（在实现文件中使用导出标记）
#ifdef _WIN32
    #define Smpclass_API __declspec(dllexport)
#else
    #define Smpclass_API
#endif

// C++ 实现部分
namespace LidarLineDetector {

struct ROI {
    int x;
    int y;
    int width;
    int height;
};

struct LidarLineResult {
    bool line_detected;
    float line_angle;
    std::string image_path;
    ErrorCode error_code;
};

struct TargetConfig {
    cv::Point2f expected_center;
    float tolerance;
};

// 版本信息函数 - 移至命名空间内
VersionInfo getVersionInfo();
const char* getVersionString();
int getVersionMajor();
int getVersionMinor();
int getVersionPatch();

// 函数声明
ErrorCode readROIFromConfig(const std::string& configPath, ROI& roi);
std::string generateFileName(const std::string& basePath, const std::string& sn);
bool detectLidarLine(const cv::Mat& inputImage, const ROI& roi, float& lineAngle);
LidarLineResult detect(const cv::Mat& image, const ROI& roi, const std::string& sn, const std::string& outputDir);
ErrorCode loadTargetConfig(const std::string& configPath, TargetConfig& config);
ErrorCode detectTargetCenter(const cv::Mat& image, cv::Point2f& outCenter, cv::Mat& displayImage);
TargetMovementResult_C checkCameraMovement(const cv::Mat& image, const TargetConfig& config, cv::Mat& displayImage);

} // namespace LidarLineDetector

// 封装类定义
class CLidarLineDetector {
private:
    LidarLineDetector::ROI m_roi;
    std::string m_sn, m_outputDir;

public:
    CLidarLineDetector() = default;
    ~CLidarLineDetector() = default;

    ErrorCode initialize(const char* configPath);
    void setROI(int x, int y, int width, int height);
    void setSn(const char* sn);
    void setOutputDir(const char* outputDir);
    TLidarLineResult_C detect(const TCMat_C image);
    ErrorCode loadTargetConfig(const char* configPath, LidarLineDetector::TargetConfig& config);
    TargetMovementResult_C checkCameraStability(const TCMat_C image, const TTargetConfig_C config);
    
    // 版本信息接口 - 添加导出标记
    static Smpclass_API VersionInfo getVersionInfo();
    static Smpclass_API const char* getVersionString();
    static Smpclass_API int getVersionMajor();
    static Smpclass_API int getVersionMinor();
    static Smpclass_API int getVersionPatch();
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
    
    // 版本信息C接口
    Smpclass_API VersionInfo LidarLineDetector_GetVersionInfo();
    Smpclass_API const char* LidarLineDetector_GetVersionString();
    Smpclass_API int LidarLineDetector_GetVersionMajor();
    Smpclass_API int LidarLineDetector_GetVersionMinor();
    Smpclass_API int LidarLineDetector_GetVersionPatch();
}

#endif // LIDAR_LINE_DETECTION_H    