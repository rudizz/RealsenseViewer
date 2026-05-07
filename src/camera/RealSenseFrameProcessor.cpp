#include "realsenseviewer/camera/RealSenseFrameProcessor.hpp"

#include "realsenseviewer/camera/PointCloudSettings.hpp"

#include <opencv2/imgproc.hpp>
#include <pcl/filters/voxel_grid.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <utility>

namespace rsv {
namespace {

constexpr int kDefaultPointCloudPixelStep = 2;
constexpr int kMinimumPointCloudPixelStep = 1;
constexpr int kMaximumPointCloudPixelStep = 12;
constexpr float kPointCloudMaxDistanceMeters = 6.0F;
constexpr float kPointCloudVoxelLeafMeters = 0.025F;

std::atomic<int> gPointCloudPixelStep { kDefaultPointCloudPixelStep };
std::atomic<bool> gPointCloudConversionEnabled { false };

std::string formatName(rs2_format format)
{
    return rs2_format_to_string(format);
}

void setDepthColor(pcl::PointXYZRGB& point, float zMeters)
{
    const float normalized = std::clamp(zMeters / kPointCloudMaxDistanceMeters, 0.0F, 1.0F);

    point.r = static_cast<std::uint8_t>(255.0F * normalized);
    point.g = static_cast<std::uint8_t>(255.0F * (1.0F - std::abs((normalized * 2.0F) - 1.0F)));
    point.b = static_cast<std::uint8_t>(255.0F * (1.0F - normalized));
}

} // namespace

int pointCloudPixelStep()
{
    return gPointCloudPixelStep.load();
}

int minimumPointCloudPixelStep()
{
    return kMinimumPointCloudPixelStep;
}

int maximumPointCloudPixelStep()
{
    return kMaximumPointCloudPixelStep;
}

void setPointCloudPixelStep(int pixelStep)
{
    gPointCloudPixelStep.store(std::clamp(pixelStep, kMinimumPointCloudPixelStep, kMaximumPointCloudPixelStep));
}

bool pointCloudConversionEnabled()
{
    return gPointCloudConversionEnabled.load();
}

void setPointCloudConversionEnabled(bool enabled)
{
    gPointCloudConversionEnabled.store(enabled);
}

FrameBundle RealSenseFrameProcessor::process(const rs2::frameset& frameset) const
{
    FrameBundle bundle;

    for (const rs2::frame& frame : frameset) {
        if (pointCloudConversionEnabled()) {
            if (auto pointCloud = convertPointCloudFrame(frame)) {
                bundle.pointCloudFrames.push_back(std::move(*pointCloud));
            }
        }

        if (auto video = convertVideoFrame(frame)) {
            bundle.videoFrames.push_back(std::move(*video));
            continue;
        }

        if (auto motion = convertMotionFrame(frame)) {
            bundle.motionSamples.push_back(std::move(*motion));
        }
    }

    return bundle;
}

std::optional<VideoFrame> RealSenseFrameProcessor::convertVideoFrame(const rs2::frame& frame) const
{
    const auto video = frame.as<rs2::video_frame>();
    if (!video) {
        return std::nullopt;
    }

    const int width = video.get_width();
    const int height = video.get_height();
    const auto stride = static_cast<size_t>(video.get_stride_in_bytes());
    const auto format = video.get_profile().format();
    cv::Mat bgrImage;

    switch (format) {
    case RS2_FORMAT_BGR8: {
        bgrImage = cv::Mat(height, width, CV_8UC3, const_cast<void*>(video.get_data()), stride).clone();
        break;
    }
    case RS2_FORMAT_RGB8: {
        const cv::Mat rgb(height, width, CV_8UC3, const_cast<void*>(video.get_data()), stride);
        cv::cvtColor(rgb, bgrImage, cv::COLOR_RGB2BGR);
        break;
    }
    case RS2_FORMAT_RGBA8: {
        const cv::Mat rgba(height, width, CV_8UC4, const_cast<void*>(video.get_data()), stride);
        cv::cvtColor(rgba, bgrImage, cv::COLOR_RGBA2BGR);
        break;
    }
    case RS2_FORMAT_BGRA8: {
        const cv::Mat bgra(height, width, CV_8UC4, const_cast<void*>(video.get_data()), stride);
        cv::cvtColor(bgra, bgrImage, cv::COLOR_BGRA2BGR);
        break;
    }
    case RS2_FORMAT_YUYV: {
        const cv::Mat yuyv(height, width, CV_8UC2, const_cast<void*>(video.get_data()), stride);
        cv::cvtColor(yuyv, bgrImage, cv::COLOR_YUV2BGR_YUY2);
        break;
    }
    case RS2_FORMAT_Y8:
    case RS2_FORMAT_RAW8: {
        const cv::Mat gray(height, width, CV_8UC1, const_cast<void*>(video.get_data()), stride);
        cv::cvtColor(gray, bgrImage, cv::COLOR_GRAY2BGR);
        break;
    }
    case RS2_FORMAT_Y16: {
        const cv::Mat gray16(height, width, CV_16UC1, const_cast<void*>(video.get_data()), stride);
        cv::Mat gray8;
        cv::normalize(gray16, gray8, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        cv::cvtColor(gray8, bgrImage, cv::COLOR_GRAY2BGR);
        break;
    }
    case RS2_FORMAT_Z16: {
        const cv::Mat depth16(height, width, CV_16UC1, const_cast<void*>(video.get_data()), stride);
        cv::Mat depth8;
        depth16.convertTo(depth8, CV_8UC1, 255.0 / 6000.0);
        cv::applyColorMap(depth8, bgrImage, cv::COLORMAP_TURBO);
        break;
    }
    default:
        return std::nullopt;
    }

    std::ostringstream details;
    details << width << "x" << height << " " << formatName(format) << " "
            << video.get_profile().fps() << "fps";

    return VideoFrame {
        frameName(frame),
        details.str(),
        std::move(bgrImage),
        frame.get_timestamp(),
    };
}

std::optional<PointCloudFrame> RealSenseFrameProcessor::convertPointCloudFrame(const rs2::frame& frame) const
{
    const auto depth = frame.as<rs2::depth_frame>();
    if (!depth || depth.get_profile().format() != RS2_FORMAT_Z16) {
        return std::nullopt;
    }

    const auto videoProfile = depth.get_profile().as<rs2::video_stream_profile>();
    if (!videoProfile) {
        return std::nullopt;
    }

    const int width = depth.get_width();
    const int height = depth.get_height();
    const auto stride = static_cast<size_t>(depth.get_stride_in_bytes());
    const auto intrinsics = videoProfile.get_intrinsics();
    const auto* depthRow = static_cast<const std::uint8_t*>(depth.get_data());
    const float depthUnits = depth.get_units();
    const int pixelStep = pointCloudPixelStep();

    auto rawCloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    rawCloud->points.reserve(static_cast<size_t>((width / pixelStep) * (height / pixelStep)));
    rawCloud->is_dense = false;

    for (int y = 0; y < height; y += pixelStep) {
        const auto* row = reinterpret_cast<const std::uint16_t*>(depthRow + (static_cast<size_t>(y) * stride));
        for (int x = 0; x < width; x += pixelStep) {
            const std::uint16_t rawDepth = row[x];
            if (rawDepth == 0) {
                continue;
            }

            const float zMeters = static_cast<float>(rawDepth) * depthUnits;
            if (zMeters <= 0.0F || zMeters > kPointCloudMaxDistanceMeters) {
                continue;
            }

            pcl::PointXYZRGB point;
            point.x = ((static_cast<float>(x) - intrinsics.ppx) / intrinsics.fx) * zMeters;
            point.y = ((static_cast<float>(y) - intrinsics.ppy) / intrinsics.fy) * zMeters;
            point.z = zMeters;
            setDepthColor(point, zMeters);
            rawCloud->points.push_back(point);
        }
    }

    rawCloud->width = static_cast<std::uint32_t>(rawCloud->points.size());
    rawCloud->height = 1;

    auto filteredCloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    if (!rawCloud->points.empty()) {
        pcl::VoxelGrid<pcl::PointXYZRGB> voxelGrid;
        voxelGrid.setInputCloud(rawCloud);
        voxelGrid.setLeafSize(kPointCloudVoxelLeafMeters, kPointCloudVoxelLeafMeters, kPointCloudVoxelLeafMeters);
        voxelGrid.filter(*filteredCloud);
    }

    std::ostringstream details;
    details << filteredCloud->points.size() << " points  pixel step " << pixelStep << "  voxel "
            << std::fixed << std::setprecision(1) << (kPointCloudVoxelLeafMeters * 100.0F) << "cm";

    return PointCloudFrame {
        "Point Cloud",
        details.str(),
        std::move(filteredCloud),
        frame.get_timestamp(),
    };
}

std::optional<MotionSample> RealSenseFrameProcessor::convertMotionFrame(const rs2::frame& frame) const
{
    const auto motion = frame.as<rs2::motion_frame>();
    if (!motion) {
        return std::nullopt;
    }

    const rs2_vector data = motion.get_motion_data();
    const rs2_stream stream = motion.get_profile().stream_type();

    return MotionSample {
        frameName(frame),
        stream == RS2_STREAM_ACCEL ? "m/s^2" : "rad/s",
        cv::Vec3f(data.x, data.y, data.z),
        frame.get_timestamp(),
    };
}

std::string RealSenseFrameProcessor::frameName(const rs2::frame& frame) const
{
    const rs2::stream_profile profile = frame.get_profile();
    std::ostringstream name;
    name << profile.stream_name();

    // if (profile.stream_index() > 0) {
    //     name << " " << profile.stream_index();
    // }

    return name.str();
}

} // namespace rsv
