#include "lidar_line_detection.h"
#include <iostream>
#include <fstream>
#include <ctime>
#include <cstdint>
#include <cmath>
#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"
#include <direct.h> // Windows下创建文件夹

using namespace cv;
using namespace std;

// 将所有LidarLineDetector相关函数实现放入命名空间
namespace LidarLineDetector {

    // 版本信息实现
    static const char *versionString = "1.0.0";

    static std::shared_ptr<spdlog::logger> logger = spdlog::basic_logger_mt("lidar_logger", "log/lidar_line_detection.log");

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
    DetectionResultCode readROIFromConfig(const string &configPath, ROI &roi)
    {
        logger->info("开始读取ROI配置文件: {}", configPath);
        ifstream configFile(configPath);
        if (!configFile.is_open())
        {
            logger->error("无法打开配置文件: {}", configPath);
            perror("错误信息"); // 打印系统错误信息
            return DetectionResultCode::CONFIG_LOAD_FAILED;
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
            return DetectionResultCode::CONFIG_LOAD_FAILED;
        if (roi.width <= 0 || roi.height <= 0)
            return DetectionResultCode::ROI_INVALID;
        logger->info("ROI配置读取成功: x={}, y={}, w={}, h={}", roi.x, roi.y, roi.width, roi.height);
        return DetectionResultCode::SUCCESS;
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
    LidarDetectionResult detectLidarLine(const cv::Mat& image, const ROI& roi, const std::string& sn, const std::string& outputDir)
    {
        logger->info("开始激光线检测，ROI: x={}, y={}, w={}, h={}", roi.x, roi.y, roi.width, roi.height);
        LidarDetectionResult result;
        result.status = DetectionResultCode::NOT_FOUND;
        result.line_angle = 0.0f;
        result.image_path = "";

        // 检查ROI是否在图像范围内
        cv::Rect roiRect(roi.x, roi.y, roi.width, roi.height);
        if (roiRect.x < 0 || roiRect.y < 0 ||
            roiRect.x + roiRect.width > image.cols ||
            roiRect.y + roiRect.height > image.rows)
        {
            logger->warn("ROI超出图像范围");
            result.status = DetectionResultCode::OUT_OF_ROI;
            // 保存失败图像
            if (!outputDir.empty())
            {
                cv::Mat resultImage = image.clone();
                cv::rectangle(resultImage, cv::Rect(roi.x, roi.y, roi.width, roi.height), cv::Scalar(0, 0, 255), 2);
                cv::putText(resultImage, "ROI Out of Range", cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
                std::string fileName = generateFileName(outputDir + "/result", sn);
                if (cv::imwrite(fileName, resultImage))
                {
                    logger->info("失败结果图像已保存: {}", fileName);
                    result.image_path = fileName;
                }
            }
            return result;
        }
        // 提取ROI区域
        cv::Mat roiMat = image(roiRect).clone();
        if (roiMat.empty())
        {
            logger->error("提取ROI区域失败");
            result.status = DetectionResultCode::OUT_OF_ROI;
            // 保存失败图像
            if (!outputDir.empty())
            {
                cv::Mat resultImage = image.clone();
                cv::rectangle(resultImage, cv::Rect(roi.x, roi.y, roi.width, roi.height), cv::Scalar(0, 0, 255), 2);
                cv::putText(resultImage, "ROI Extraction Failed", cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
                std::string fileName = generateFileName(outputDir + "/result", sn);
                if (cv::imwrite(fileName, resultImage))
                {
                    logger->info("失败结果图像已保存: {}", fileName);
                    result.image_path = fileName;
                }
            }
            return result;
        }
        // 灰度化
        cv::Mat gray;
        cv::cvtColor(roiMat, gray, cv::COLOR_BGR2GRAY);

        // 提取所有高亮点
        std::vector<cv::Point> laserPoints;
        for (int y = 0; y < gray.rows; ++y) {
            for (int x = 0; x < gray.cols; ++x) {
                if (gray.at<uchar>(y, x) > 220) { // 阈值可调
                    laserPoints.emplace_back(x, y);
                }
            }
        }

        // 可视化激光点
        cv::Mat debugPoints = roiMat.clone();
        for (const auto& pt : laserPoints) {
            cv::circle(debugPoints, pt, 1, cv::Scalar(0, 0, 255), -1);
        }
        if (!outputDir.empty()) {
            std::string debugFileName = generateFileName(outputDir + "/debug_laser_points", sn);
            cv::imwrite(debugFileName, debugPoints);
        }

        // 判据1：点数
        if (laserPoints.size() < 10)
        {
            logger->warn("激光点太少，检测失败，点数: {}", laserPoints.size());
            result.status = DetectionResultCode::NOT_FOUND;
            // 保存失败图像
            if (!outputDir.empty())
            {
                cv::Mat resultImage = image.clone();
                cv::rectangle(resultImage, cv::Rect(roi.x, roi.y, roi.width, roi.height), cv::Scalar(0, 0, 255), 2);
                cv::putText(resultImage, "Insufficient Laser Points: " + std::to_string(laserPoints.size()), cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
                std::string fileName = generateFileName(outputDir + "/result", sn);
                if (cv::imwrite(fileName, resultImage))
                {
                    logger->info("失败结果图像已保存: {}", fileName);
                    result.image_path = fileName;
                }
            }
            return result;
        }
        // 用fitLine拟合直线
        cv::Vec4f line;
        cv::fitLine(laserPoints, line, cv::DIST_L2, 0, 0.01, 0.01);
        float vx = line[0], vy = line[1], x0 = line[2], y0 = line[3];

        // 判据2：RMS误差
        double sumDist2 = 0;
        for (const auto& pt : laserPoints) {
            double dist = std::abs(vy * (pt.x - x0) - vx * (pt.y - y0)) / std::sqrt(vx * vx + vy * vy);
            sumDist2 += dist * dist;
        }
        double rms = std::sqrt(sumDist2 / laserPoints.size());

        // 判据3：投影长度
        std::vector<double> projections;
        for (const auto& pt : laserPoints) {
            double proj = (pt.x - x0) * vx + (pt.y - y0) * vy;
            projections.push_back(proj);
        }
        auto minmax = std::minmax_element(projections.begin(), projections.end());
        double length = *minmax.second - *minmax.first;
        logger->warn("激光点分布不线性或长度不足，RMS: {}, 长度: {}", rms, length);

        // 阈值可根据实际调整
        if (rms > 5.0 || length < roi.width * 0.5) {
            logger->warn("激光点分布不线性或长度不足，RMS: {}, 长度: {}", rms, length);
            result.status = DetectionResultCode::OUT_OF_ROI;
            // 保存失败图像
            if (!outputDir.empty()) {
                cv::Mat resultImage = image.clone();
                cv::rectangle(resultImage, cv::Rect(roi.x, roi.y, roi.width, roi.height), cv::Scalar(0, 0, 255), 2);
                std::string reason = (rms > 3.0) ? ("RMS: " + std::to_string(rms)) : ("Length: " + std::to_string(length));
                cv::putText(resultImage, "No Laser Line: " + reason, cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
                std::string fileName = generateFileName(outputDir + "/result", sn);
                if (cv::imwrite(fileName, resultImage))
                {
                    logger->info("失败结果图像已保存: {}", fileName);
                    result.image_path = fileName;
                }
            }
            return result;
        }

        float lineAngle = std::atan2(line[1], line[0]);
        result.status = DetectionResultCode::SUCCESS;
        result.line_angle = lineAngle;
        logger->info("激光线检测成功，角度: {:.2f}°，点数: {}, RMS: {:.2f}, 长度: {:.2f}", lineAngle * 180.0 / CV_PI, laserPoints.size(), rms, length);
        // 如果输出目录不为空，保存结果图像
        if (!outputDir.empty())
        {
            cv::Mat resultImage = image.clone();
            // 画ROI和直线段（只覆盖所有高亮点）
            // 计算所有点在直线方向上的投影
            std::vector<double> projections;
            for (const auto& pt : laserPoints) {
                double proj = (pt.x - x0) * vx + (pt.y - y0) * vy;
                projections.push_back(proj);
            }
            auto minmax = std::minmax_element(projections.begin(), projections.end());
            double minProj = *minmax.first;
            double maxProj = *minmax.second;
            // 计算直线段的两个端点（ROI内坐标）
            cv::Point pt1_roi(x0 + minProj * vx, y0 + minProj * vy);
            cv::Point pt2_roi(x0 + maxProj * vx, y0 + maxProj * vy);
            // 转为全图坐标
            cv::Point pt1(pt1_roi.x + roi.x, pt1_roi.y + roi.y);
            cv::Point pt2(pt2_roi.x + roi.x, pt2_roi.y + roi.y);
            cv::rectangle(resultImage, cv::Rect(roi.x, roi.y, roi.width, roi.height), cv::Scalar(0, 255, 0), 2);
            cv::line(resultImage, pt1, pt2, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
            std::string fileName = generateFileName(outputDir + "/result", sn);
            if (cv::imwrite(fileName, resultImage))
            {
                logger->info("检测结果图像已保存: {}", fileName);
                result.image_path = fileName;
            }
            else
            
            {
                logger->error("保存图像失败: {}", fileName);
            }
        }
        return result;
    }

    // 激光线检测主函数
    LidarLineResult detect(const cv::Mat &image, const ROI &roi, const std::string &sn, const std::string &outputDir)
    {
        logger->info("开始主检测流程");
        LidarLineResult result{false, 0, "", DetectionResultCode::SUCCESS};
        LidarDetectionResult detectionResult = detectLidarLine(image, roi, sn, outputDir);

        if (detectionResult.status != DetectionResultCode::SUCCESS)
        {
            logger->warn("主检测流程：激光线检测失败，状态: {}", static_cast<int>(detectionResult.status));
            switch (detectionResult.status) {
                case DetectionResultCode::NOT_FOUND:
                    result.error_code = DetectionResultCode::NOT_FOUND;
                    break;
                case DetectionResultCode::OUT_OF_ROI:
                    result.error_code = DetectionResultCode::OUT_OF_ROI;
                    break;
                default:
                    result.error_code = DetectionResultCode::UNKNOWN_ERROR;
                    break;
            }
            return result;
        }
        result.line_angle = detectionResult.line_angle;
        logger->info("主检测流程：激光线检测成功，角度: {:.2f}°", result.line_angle * 180.0 / CV_PI);

        // 如果输出目录不为空，保存结果图像
        if (!outputDir.empty())
        {
            cv::Mat resultImage = image.clone();
            if (resultImage.empty())
            {
                logger->error("克隆图像失败");
                result.error_code = DetectionResultCode::IMAGE_SAVE_FAILED;
                return result;
            }

            // line: [vx, vy, x, y]，x/y是ROI内坐标
            float vx = detectionResult.line_angle; // 使用检测到的角度作为vx
            float x = 0; // 假设直线在ROI的左端点
            float y = 0; // 假设直线在ROI的左端点

            // 计算直线在ROI内的两个端点（以ROI内坐标为基准）
            float leftY = (-x * vx / 1) + y;                   // x=0
            float rightY = ((roi.width - 1 - x) * vx / 1) + y; // x=roi.width-1

            cv::Point pt1(roi.x, roi.y + cvRound(leftY));
            cv::Point pt2(roi.x + roi.width - 1, roi.y + cvRound(rightY));
            cv::rectangle(resultImage, cv::Rect(roi.x, roi.y, roi.width, roi.height), cv::Scalar(0, 255, 0), 2);
            cv::line(resultImage, pt1, pt2, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);

            // 生成文件名并保存图像
            std::string fileName = generateFileName(outputDir + "/result", sn);
            if (!cv::imwrite(fileName, resultImage))
            {
                logger->error("保存图像失败: {}", fileName);
                result.error_code = DetectionResultCode::IMAGE_SAVE_FAILED;
            }
            else
            {
                logger->info("检测结果图像已保存: {}", fileName);
                result.image_path = fileName;
            }
        }

        result.line_detected = true;
        return result;
    }



} // namespace LidarLineDetector

// 封装类实现
DetectionResultCode CLidarLineDetector::initialize(const char *configPath)
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

    Smpclass_API DetectionResultCode CLidarLineDetector_initialize(CLidarLineDetector *instance, const char *configPath)
    {
        return instance->initialize(configPath);
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