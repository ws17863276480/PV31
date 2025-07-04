#include "lidar_line_detection.h"
#include <iostream>
#include <fstream>
#include <ctime>
#include <cstdint>
#include <cmath>

using namespace cv;
using namespace std;

namespace LidarLineDetector
{

    // 版本信息实现
    static const char *versionString = "1.0.0";

    VersionInfo getVersionInfo()
    {
        return {
            LIDAR_LINE_DETECTION_VERSION_MAJOR,
            LIDAR_LINE_DETECTION_VERSION_MINOR,
            LIDAR_LINE_DETECTION_VERSION_PATCH,
            versionString};
    }

    const char *getVersionString()
    {
        return versionString;
    }

    int getVersionMajor()
    {
        return LIDAR_LINE_DETECTION_VERSION_MAJOR;
    }

    int getVersionMinor()
    {
        return LIDAR_LINE_DETECTION_VERSION_MINOR;
    }

    int getVersionPatch()
    {
        return LIDAR_LINE_DETECTION_VERSION_PATCH;
    }

    // 读取ROI配置文件
    ErrorCode readROIFromConfig(const string &configPath, ROI &roi)
    {
        cerr << "尝试打开的配置文件路径: " << configPath << endl;
        ifstream configFile(configPath);
        if (!configFile.is_open())
        {
            cerr << "无法打开配置文件: " << configPath << endl;
            perror("错误信息"); // 打印系统错误信息
            return ErrorCode::CONFIG_LOAD_FAILED;
        }

        string line;
        bool xRead = false, yRead = false, widthRead = false, heightRead = false;

        while (getline(configFile, line))
        {
            if (line.find("x:") == 0)
            {
                sscanf(line.c_str(), "x: %d", &roi.x);
                xRead = true;
            }
            else if (line.find("y:") == 0)
            {
                sscanf(line.c_str(), "y: %d", &roi.y);
                yRead = true;
            }
            else if (line.find("width:") == 0)
            {
                sscanf(line.c_str(), "width: %d", &roi.width);
                widthRead = true;
            }
            else if (line.find("height:") == 0)
            {
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
    string generateFileName(const string &basePath, const string &sn)
    {
        time_t now = time(0);
        char timeStr[26];
        ctime_s(timeStr, sizeof(timeStr), &now);
        for (int i = 0; timeStr[i]; i++)
            if (timeStr[i] == ' ' || timeStr[i] == ':' || timeStr[i] == '\n')
                timeStr[i] = '_';
        return basePath + "_" + sn + "_" + timeStr + ".jpg";
    }

    // 激光线检测核心函数
    bool detectLidarLine(const cv::Mat &inputImage, const ROI &roi, float &lineAngle, cv::Vec4f &line)
    {
        // 检查ROI是否在图像范围内
        cv::Rect roiRect(roi.x, roi.y, roi.width, roi.height);
        if (roiRect.x < 0 || roiRect.y < 0 ||
            roiRect.x + roiRect.width > inputImage.cols ||
            roiRect.y + roiRect.height > inputImage.rows)
        {
            std::cerr << "ROI超出图像范围" << std::endl;
            return false;
        }

        // 提取ROI区域
        cv::Mat roiMat = inputImage(roiRect).clone();
        if (roiMat.empty())
        {
            std::cerr << "提取ROI区域失败" << std::endl;
            return false;
        }

        // 灰度化
        cv::Mat gray;
        cv::cvtColor(roiMat, gray, cv::COLOR_BGR2GRAY);

        // 自适应阈值或Otsu法
        cv::Mat binary;
        cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);

        // 形态学操作去噪
        cv::Mat morph;
        cv::morphologyEx(binary, morph, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

        // 边缘检测
        cv::Mat edges;
        cv::Canny(morph, edges, 50, 150);

        // 霍夫变换检测直线
        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 50, 80, 10);

        if (lines.empty())
        {
            std::cerr << "未检测到直线" << std::endl;
            return false;
        }

        // 只取每一行最亮的点
        std::vector<cv::Point> laserPoints;
        for (int y = 0; y < gray.rows; ++y)
        {
            double minVal, maxVal;
            cv::Point minLoc, maxLoc;
            cv::minMaxLoc(gray.row(y), &minVal, &maxVal, &minLoc, &maxLoc);
            // 可加阈值过滤
            if (maxVal > 200)
            { // 200可根据实际调整
                laserPoints.emplace_back(maxLoc.x, y);
            }
        }

        if (laserPoints.size() < 10)
        { // 点太少说明检测失败
            std::cerr << "激光点太少，检测失败" << std::endl;
            return false;
        }

        cv::fitLine(laserPoints, line, cv::DIST_L2, 0, 0.01, 0.01);
        lineAngle = std::atan2(line[1], line[0]);
        return true;
    }

    // 激光线检测主函数
    LidarLineResult detect(const cv::Mat &image, const ROI &roi, const std::string &sn, const std::string &outputDir)
    {
        LidarLineResult result{false, 0, "", ErrorCode::SUCCESS};
        float lineAngle;
        cv::Vec4f line; // 新增

        // 进行激光线检测
        if (!detectLidarLine(image, roi, lineAngle, line))
        {
            result.error_code = ErrorCode::LINE_DETECTION_FAILED;
            return result;
        }
        result.line_angle = lineAngle;

        // 如果输出目录不为空，保存结果图像
        if (!outputDir.empty())
        {
            cv::Mat resultImage = image.clone();
            if (resultImage.empty())
            {
                std::cerr << "克隆图像失败" << std::endl;
                result.error_code = ErrorCode::IMAGE_SAVE_FAILED;
                return result;
            }

            // line: [vx, vy, x, y]，x/y是ROI内坐标
            float vx = line[0], vy = line[1];
            float x = line[2], y = line[3];

            // 计算直线在ROI内的两个端点（以ROI内坐标为基准）
            float leftY = (-x * vy / vx) + y;                   // x=0
            float rightY = ((roi.width - 1 - x) * vy / vx) + y; // x=roi.width-1

            cv::Point pt1(roi.x, roi.y + cvRound(leftY));
            cv::Point pt2(roi.x + roi.width - 1, roi.y + cvRound(rightY));
            cv::rectangle(resultImage, cv::Rect(roi.x, roi.y, roi.width, roi.height), cv::Scalar(0, 255, 0), 2);
            cv::line(resultImage, pt1, pt2, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

            // 生成文件名并保存图像
            std::string fileName = generateFileName(outputDir + "/result", sn);
            if (!cv::imwrite(fileName, resultImage))
            {
                std::cerr << "保存图像失败: " << fileName << std::endl;
                result.error_code = ErrorCode::IMAGE_SAVE_FAILED;
            }
            else
            {
                result.image_path = fileName;
            }
        }

        result.line_detected = true;
        return result;
    }

    // 标靶配置文件读取
    ErrorCode loadTargetConfig(const string &configPath, TargetConfig &config)
    {
        ifstream file(configPath);
        if (!file.is_open())
        {
            cerr << "无法打开配置文件: " << configPath << endl;
            return ErrorCode::CONFIG_LOAD_FAILED;
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
                    cerr << "解析 center_x 失败: " << line << endl;
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
                    cerr << "解析 center_y 失败: " << line << endl;
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
                    cerr << "解析 tolerance 失败: " << line << endl;
                }
            }
        }
        file.close();

        if (!centerRead || !toleranceRead)
        {
            cerr << "配置文件格式错误，未读取到完整信息。" << endl;
        }
        return (centerRead && toleranceRead) ? ErrorCode::SUCCESS : ErrorCode::CONFIG_LOAD_FAILED;
    }

    // 标靶中心点检测
    ErrorCode detectTargetCenter(const Mat &image, Point2f &outCenter, Mat &displayImage)
    {
        displayImage = image.clone();
        Mat gray, binary;
        cvtColor(image, gray, COLOR_BGR2GRAY);
        threshold(gray, binary, 30, 255, THRESH_BINARY_INV);

        vector<vector<Point>> contours;
        findContours(binary, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        vector<Rect> target_rects;
        for (const auto &cnt : contours)
        {
            Rect bbox = boundingRect(cnt);
            float area = bbox.area(), aspectRatio = (float)bbox.width / bbox.height;
            if (area > 1000 && area < 10000 && aspectRatio > 0.8 && aspectRatio < 1.2)
            {
                target_rects.push_back(bbox);
                rectangle(displayImage, bbox, Scalar(0, 255, 0), 2);
            }
        }

        if (target_rects.size() != 4)
            return ErrorCode::LINE_DETECTION_FAILED;

        Rect outerRect = target_rects[0];
        for (size_t i = 1; i < target_rects.size(); ++i)
            outerRect |= target_rects[i];

        outCenter = Point2f(outerRect.x + outerRect.width / 2, outerRect.y + outerRect.height / 2);
        circle(displayImage, outCenter, 5, Scalar(0, 0, 255), -1);
        return ErrorCode::SUCCESS;
    }

    // 相机自检函数
    TargetMovementResult_C checkCameraMovement(const Mat &image, const TargetConfig &config, Mat &displayImage)
    {
        // 修复：显式转换枚举类型
        TargetMovementResult_C result{0, 0, 0, 0, static_cast<int>(ErrorCode::SUCCESS), ""};
        Point2f currentCenter;
        ErrorCode err = detectTargetCenter(image, currentCenter, displayImage);
        if (err != ErrorCode::SUCCESS)
        {
            result.error_code = static_cast<int>(err);
            snprintf(result.message, sizeof(result.message), "标靶检测失败: %d", result.error_code);
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

        circle(displayImage, config.expected_center, (int)config.tolerance, Scalar(255, 0, 0), 2);
        line(displayImage, config.expected_center, currentCenter, Scalar(0, 255, 255), 2);
        putText(displayImage, result.message, Point(20, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);

        return result;
    }

} // namespace LidarLineDetector

// 封装类实现
ErrorCode CLidarLineDetector::initialize(const char *configPath)
{
    return LidarLineDetector::readROIFromConfig(configPath, m_roi);
}

void CLidarLineDetector::setROI(int x, int y, int width, int height) { m_roi = {x, y, width, height}; }
void CLidarLineDetector::setSn(const char *sn) { m_sn = sn ? sn : ""; }
void CLidarLineDetector::setOutputDir(const char *outputDir) { m_outputDir = outputDir ? outputDir : ""; }

TLidarLineResult_C CLidarLineDetector::detect(const TCMat_C image)
{
    Mat image_cpp(image.rows, image.cols, image.type, image.data);
    auto result = LidarLineDetector::detect(image_cpp, m_roi, m_sn, m_outputDir);

    TLidarLineResult_C result_c;
    result_c.line_detected = result.line_detected;
    result_c.line_angle = result.line_angle;
    snprintf(result_c.image_path, sizeof(result_c.image_path), "%s", result.image_path.c_str());
    result_c.error_code = static_cast<int>(result.error_code);
    return result_c;
}

ErrorCode CLidarLineDetector::loadTargetConfig(const char *configPath, LidarLineDetector::TargetConfig &config)
{
    return LidarLineDetector::loadTargetConfig(configPath, config);
}

TargetMovementResult_C CLidarLineDetector::checkCameraStability(const TCMat_C image, const TTargetConfig_C config)
{
    Mat image_cpp(image.rows, image.cols, image.type, image.data);
    LidarLineDetector::TargetConfig internalConfig{
        Point2f(config.center_x, config.center_y),
        config.tolerance};
    Mat displayImage;
    return LidarLineDetector::checkCameraMovement(image_cpp, internalConfig, displayImage);
}
// 版本信息实现
VersionInfo CLidarLineDetector::getVersionInfo()
{
    return LidarLineDetector::getVersionInfo();
}

const char *CLidarLineDetector::getVersionString()
{
    return LidarLineDetector::getVersionString();
}

int CLidarLineDetector::getVersionMajor()
{
    return LidarLineDetector::getVersionMajor();
}

int CLidarLineDetector::getVersionMinor()
{
    return LidarLineDetector::getVersionMinor();
}

int CLidarLineDetector::getVersionPatch()
{
    return LidarLineDetector::getVersionPatch();
}

// C 接口实现
extern "C"
{
    Smpclass_API CLidarLineDetector *CLidarLineDetector_new()
    {
        return new CLidarLineDetector();
    }

    Smpclass_API void CLidarLineDetector_delete(CLidarLineDetector *instance)
    {
        delete instance;
    }

    Smpclass_API int CLidarLineDetector_initialize(CLidarLineDetector *instance, const char *configPath)
    {
        return static_cast<int>(instance->initialize(configPath));
    }

    Smpclass_API void CLidarLineDetector_setROI(CLidarLineDetector *instance, int x, int y, int width, int height)
    {
        instance->setROI(x, y, width, height);
    }

    Smpclass_API void CLidarLineDetector_setSn(CLidarLineDetector *instance, const char *sn)
    {
        instance->setSn(sn);
    }

    Smpclass_API void CLidarLineDetector_setOutputDir(CLidarLineDetector *instance, const char *outputDir)
    {
        instance->setOutputDir(outputDir);
    }

    Smpclass_API TLidarLineResult_C CLidarLineDetector_detect(CLidarLineDetector *instance, const TCMat_C image)
    {
        return instance->detect(image);
    }

    Smpclass_API int CLidarLineDetector_loadTargetConfig(CLidarLineDetector *instance, const char *configPath, TTargetConfig_C *config)
    {
        LidarLineDetector::TargetConfig internalConfig;
        auto err = instance->loadTargetConfig(configPath, internalConfig);
        if (err == ErrorCode::SUCCESS)
        {
            config->center_x = internalConfig.expected_center.x;
            config->center_y = internalConfig.expected_center.y;
            config->tolerance = internalConfig.tolerance;
        }
        return static_cast<int>(err);
    }

    Smpclass_API TargetMovementResult_C CLidarLineDetector_checkCameraStability(CLidarLineDetector *instance, const TCMat_C image, const TTargetConfig_C config)
    {
        return instance->checkCameraStability(image, config);
    }

    // 版本信息C接口实现
    Smpclass_API VersionInfo LidarLineDetector_GetVersionInfo()
    {
        return LidarLineDetector::getVersionInfo();
    }

    Smpclass_API const char *LidarLineDetector_GetVersionString()
    {
        return LidarLineDetector::getVersionString();
    }

    Smpclass_API int LidarLineDetector_GetVersionMajor()
    {
        return LidarLineDetector::getVersionMajor();
    }

    Smpclass_API int LidarLineDetector_GetVersionMinor()
    {
        return LidarLineDetector::getVersionMinor();
    }

    Smpclass_API int LidarLineDetector_GetVersionPatch()
    {
        return LidarLineDetector::getVersionPatch();
    }
}