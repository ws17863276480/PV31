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

using namespace cv;
using namespace std;

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
    string image_path;
    ErrorCode error_code;
};

struct TargetConfig {
    Point2f expected_center;
    float tolerance;
};

// 读取ROI配置文件
ErrorCode readROIFromConfig(const string& configPath, ROI& roi) {
    cerr << "尝试打开的配置文件路径: " << configPath << endl;
    ifstream configFile(configPath);
    if (!configFile.is_open()) {
        cerr << "无法打开配置文件: " << configPath << endl;
        perror("错误信息"); // 打印系统错误信息
        return ErrorCode::CONFIG_LOAD_FAILED;
    }

    string line;
    bool xRead = false, yRead = false, widthRead = false, heightRead = false;

    while (getline(configFile, line)) {
        if (line.find("x:") == 0) {
            sscanf(line.c_str(), "x: %d", &roi.x);
            xRead = true;
        }
        else if (line.find("y:") == 0) {
            sscanf(line.c_str(), "y: %d", &roi.y);
            yRead = true;
        }
        else if (line.find("width:") == 0) {
            sscanf(line.c_str(), "width: %d", &roi.width);
            widthRead = true;
        }
        else if (line.find("height:") == 0) {
            sscanf(line.c_str(), "height: %d", &roi.height);
            heightRead = true;
        }
    }
    configFile.close();

    if (!xRead || !yRead || !widthRead || !heightRead) 
        return ErrorCode::CONFIG_LOAD_FAILED;
    if (roi.width <= 0 || roi.height <= 0) 
        return ErrorCode::ROI_INVALID;
    return ErrorCode::SUCCESS;
}

// 生成带时间和SN的文件名
string generateFileName(const string& basePath, const string& sn) {
    time_t now = time(0);
    char timeStr[26];
    ctime_s(timeStr, sizeof(timeStr), &now);
    for (int i = 0; timeStr[i]; i++) 
        if (timeStr[i] == ' ' || timeStr[i] == ':' || timeStr[i] == '\n') 
            timeStr[i] = '_';
    return basePath + "_" + sn + "_" + timeStr + ".jpg";
}

// 激光线检测核心函数
bool detectLidarLine(const cv::Mat& inputImage, const ROI& roi, float& lineAngle) {
    // 检查ROI是否在图像范围内
    cv::Rect roiRect(roi.x, roi.y, roi.width, roi.height);
    if (roiRect.x < 0 || roiRect.y < 0 || 
        roiRect.x + roiRect.width > inputImage.cols || 
        roiRect.y + roiRect.height > inputImage.rows) {
        std::cerr << "ROI超出图像范围" << std::endl;
        return false;
    }

    // 提取ROI区域
    cv::Mat roiMat = inputImage(roiRect).clone();
    if (roiMat.empty()) {
        std::cerr << "提取ROI区域失败" << std::endl;
        return false;
    }

    // 图像预处理
    cv::Mat gray, blurred, edges;
    cv::cvtColor(roiMat, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blurred, cv::Size(7, 7), 1.5);
    cv::Canny(blurred, edges, 30, 120);

    // 霍夫变换检测直线
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 60, 60, 15);
    if (lines.empty()) {
        std::cerr << "未检测到直线" << std::endl;
        return false;
    }

    // 收集所有直线上的点
    std::vector<cv::Point> points;
    for (const auto& line : lines) {
        points.emplace_back(line[0], line[1]);
        points.emplace_back(line[2], line[3]);
    }

    // 使用最小二乘法拟合直线
    cv::Vec4f line;
    cv::fitLine(points, line, cv::DIST_L2, 0, 0.01, 0.01);

    // 计算直线角度
    lineAngle = std::atan2(line[1], line[0]);
    return true;
}

// 激光线检测主函数
LidarLineResult detect(const cv::Mat& image, const ROI& roi, const std::string& sn, const std::string& outputDir) {
    LidarLineResult result{false, 0, "", ErrorCode::SUCCESS};
    float lineAngle;

    // 进行激光线检测
    if (!detectLidarLine(image, roi, lineAngle)) {
        result.error_code = ErrorCode::LINE_DETECTION_FAILED;
        return result;
    }

    result.line_angle = lineAngle;

    // 如果输出目录不为空，保存结果图像
    if (!outputDir.empty()) {
        cv::Mat resultImage = image.clone();
        if (resultImage.empty()) {
            std::cerr << "克隆图像失败" << std::endl;
            result.error_code = ErrorCode::IMAGE_SAVE_FAILED;
            return result;
        }

        // 计算直线端点
        float vx = std::cos(lineAngle), vy = std::sin(lineAngle);
        float x0 = roi.x + roi.width / 2.0f, y0 = roi.y + roi.height / 2.0f;
        float t_left = (roi.x - x0) / vx, t_right = (roi.x + roi.width - 1 - x0) / vx;
        cv::Point pt1(x0 + t_left * vx, y0 + t_left * vy);
        cv::Point pt2(x0 + t_right * vx, y0 + t_right * vy);

        // 绘制ROI矩形和检测到的直线
        cv::rectangle(resultImage, cv::Rect(roi.x, roi.y, roi.width, roi.height), cv::Scalar(0, 255, 0), 1);
        cv::line(resultImage, pt1, pt2, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

        // 生成文件名并保存图像
        std::string fileName = generateFileName(outputDir + "/result", sn);
        if (!cv::imwrite(fileName, resultImage)) {
            std::cerr << "保存图像失败: " << fileName << std::endl;
            result.error_code = ErrorCode::IMAGE_SAVE_FAILED;
        } else {
            result.image_path = fileName;
        }
    }

    result.line_detected = true;
    return result;
}

// 标靶配置文件读取
ErrorCode loadTargetConfig(const string& configPath, TargetConfig& config) {
    ifstream file(configPath);
    if (!file.is_open()) {
        cerr << "无法打开配置文件: " << configPath << endl;
        return ErrorCode::CONFIG_LOAD_FAILED;
    }

    string line;
    bool centerRead = false, toleranceRead = false;
    while (getline(file, line)) {
        if (line.find("center_x:") == 0) {
            if (sscanf(line.c_str(), "center_x: %f", &config.expected_center.x) == 1) {
                centerRead = true;
            } else {
                cerr << "解析 center_x 失败: " << line << endl;
            }
        } else if (line.find("center_y:") == 0) {
            if (sscanf(line.c_str(), "center_y: %f", &config.expected_center.y) == 1) {
                centerRead = true;
            } else {
                cerr << "解析 center_y 失败: " << line << endl;
            }
        } else if (line.find("tolerance:") == 0) {
            if (sscanf(line.c_str(), "tolerance: %f", &config.tolerance) == 1) {
                toleranceRead = true;
            } else {
                cerr << "解析 tolerance 失败: " << line << endl;
            }
        }
    }
    file.close();

    if (!centerRead || !toleranceRead) {
        cerr << "配置文件格式错误，未读取到完整信息。" << endl;
    }
    return (centerRead && toleranceRead) ? ErrorCode::SUCCESS : ErrorCode::CONFIG_LOAD_FAILED;
}

// 标靶中心点检测
ErrorCode detectTargetCenter(const Mat& image, Point2f& outCenter, Mat& displayImage) {
    displayImage = image.clone();
    Mat gray, binary;
    cvtColor(image, gray, COLOR_BGR2GRAY);
    threshold(gray, binary, 30, 255, THRESH_BINARY_INV);

    vector<vector<Point>> contours;
    findContours(binary, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    vector<Rect> target_rects;
    for (const auto& cnt : contours) {
        Rect bbox = boundingRect(cnt);
        float area = bbox.area(), aspectRatio = (float)bbox.width/bbox.height;
        if (area > 1000 && area < 10000 && aspectRatio > 0.8 && aspectRatio < 1.2) {
            target_rects.push_back(bbox);
            rectangle(displayImage, bbox, Scalar(0, 255, 0), 2);
        }
    }

    if (target_rects.size() != 4) 
        return ErrorCode::LINE_DETECTION_FAILED;

    Rect outerRect = target_rects[0];
    for (size_t i = 1; i < target_rects.size(); ++i) 
        outerRect |= target_rects[i];

    outCenter = Point2f(outerRect.x + outerRect.width/2, outerRect.y + outerRect.height/2);
    circle(displayImage, outCenter, 5, Scalar(0, 0, 255), -1);
    return ErrorCode::SUCCESS;
}

// 相机自检函数
TargetMovementResult_C checkCameraMovement(const Mat& image, const TargetConfig& config, Mat& displayImage) {
    // 修复：显式转换枚举类型
    TargetMovementResult_C result{0, 0, 0, 0, static_cast<int>(ErrorCode::SUCCESS), ""};
    Point2f currentCenter;
    ErrorCode err = detectTargetCenter(image, currentCenter, displayImage);
    if (err != ErrorCode::SUCCESS) {
        result.error_code = static_cast<int>(err);
        snprintf(result.message, sizeof(result.message), "标靶检测失败: %d", result.error_code);
        return result;
    }

    float dx = currentCenter.x - config.expected_center.x;
    float dy = currentCenter.y - config.expected_center.y;
    result.dx = dx;
    result.dy = dy;
    result.distance = sqrt(dx*dx + dy*dy);
    result.is_stable = (result.distance <= config.tolerance);

    snprintf(result.message, sizeof(result.message), 
        result.is_stable ? 
        "相机稳定，偏差: %.1fpx" : 
        "相机移动！偏差: %.1fpx (>%.1fpx)", 
        result.distance, result.distance, config.tolerance);

    circle(displayImage, config.expected_center, (int)config.tolerance, Scalar(255, 0, 0), 2);
    line(displayImage, config.expected_center, currentCenter, Scalar(0, 255, 255), 2);
    putText(displayImage, result.message, Point(20, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);

    return result;
}

} // namespace LidarLineDetector

// 封装类定义
class CLidarLineDetector {
private:
    LidarLineDetector::ROI m_roi;
    string m_sn, m_outputDir;

public:
    CLidarLineDetector() = default;
    ~CLidarLineDetector() = default;

    ErrorCode initialize(const char* configPath) {
        return LidarLineDetector::readROIFromConfig(configPath, m_roi);
    }

    void setROI(int x, int y, int width, int height) { m_roi = {x, y, width, height}; }
    void setSn(const char* sn) { m_sn = sn ? sn : ""; }
    void setOutputDir(const char* outputDir) { m_outputDir = outputDir ? outputDir : ""; }

    // 修复：正确初始化返回结构体
    TLidarLineResult_C detect(const TCMat_C image) {
        Mat image_cpp(image.rows, image.cols, image.type, image.data);
        auto result = LidarLineDetector::detect(image_cpp, m_roi, m_sn, m_outputDir);
        
        TLidarLineResult_C result_c;
        result_c.line_detected = result.line_detected;
        result_c.line_angle = result.line_angle;
        snprintf(result_c.image_path, sizeof(result_c.image_path), "%s", result.image_path.c_str());
        result_c.error_code = static_cast<int>(result.error_code);
        return result_c;
    }

    ErrorCode loadTargetConfig(const char* configPath, LidarLineDetector::TargetConfig& config) {
        return LidarLineDetector::loadTargetConfig(configPath, config);
    }

    TargetMovementResult_C checkCameraStability(const TCMat_C image, const TTargetConfig_C config) {
        Mat image_cpp(image.rows, image.cols, image.type, image.data);
        LidarLineDetector::TargetConfig internalConfig{
            Point2f(config.center_x, config.center_y),
            config.tolerance
        };
        Mat displayImage;
        return LidarLineDetector::checkCameraMovement(image_cpp, internalConfig, displayImage);
    }
};

// C 接口定义
extern "C" {
    Smpclass_API CLidarLineDetector* CLidarLineDetector_new() {
        return new CLidarLineDetector();
    }

    Smpclass_API void CLidarLineDetector_delete(CLidarLineDetector* instance) {
        delete instance;
    }

    Smpclass_API int CLidarLineDetector_initialize(CLidarLineDetector* instance, const char* configPath) {
        return static_cast<int>(instance->initialize(configPath));
    }

    Smpclass_API void CLidarLineDetector_setROI(CLidarLineDetector* instance, int x, int y, int width, int height) {
        instance->setROI(x, y, width, height);
    }

    Smpclass_API void CLidarLineDetector_setSn(CLidarLineDetector* instance, const char* sn) {
        instance->setSn(sn);
    }

    Smpclass_API void CLidarLineDetector_setOutputDir(CLidarLineDetector* instance, const char* outputDir) {
        instance->setOutputDir(outputDir);
    }

    Smpclass_API TLidarLineResult_C CLidarLineDetector_detect(CLidarLineDetector* instance, const TCMat_C image) {
        return instance->detect(image);
    }

    Smpclass_API int CLidarLineDetector_loadTargetConfig(CLidarLineDetector* instance, const char* configPath, TTargetConfig_C* config) {
        LidarLineDetector::TargetConfig internalConfig;
        auto err = instance->loadTargetConfig(configPath, internalConfig);
        if (err == ErrorCode::SUCCESS) {
            config->center_x = internalConfig.expected_center.x;
            config->center_y = internalConfig.expected_center.y;
            config->tolerance = internalConfig.tolerance;
        }
        return static_cast<int>(err);
    }

    Smpclass_API TargetMovementResult_C CLidarLineDetector_checkCameraStability(CLidarLineDetector* instance, const TCMat_C image, const TTargetConfig_C config) {
        return instance->checkCameraStability(image, config);
    }
}