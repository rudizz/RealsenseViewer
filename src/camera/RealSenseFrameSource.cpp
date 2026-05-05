#include "realsenseviewer/camera/RealSenseDiagnostics.hpp"
#include "realsenseviewer/camera/RealSenseFrameSource.hpp"

#include <opencv2/imgproc.hpp>
#include <pcl/filters/voxel_grid.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace rsv {
namespace {

constexpr unsigned int kFrameWaitTimeoutMs = 500;
constexpr auto kNoFrameWarningInterval = std::chrono::seconds(3);
constexpr int kDefaultPointCloudPixelStep = 2;
constexpr int kMinimumPointCloudPixelStep = 1;
constexpr int kMaximumPointCloudPixelStep = 12;
constexpr float kPointCloudMaxDistanceMeters = 6.0F;
constexpr float kPointCloudVoxelLeafMeters = 0.025F;

std::atomic<int> gPointCloudPixelStep { kDefaultPointCloudPixelStep };
std::atomic<bool> gPointCloudConversionEnabled { false };

bool hasFormat(const std::vector<rs2_format>& formats, rs2_format format)
{
    return std::find(formats.begin(), formats.end(), format) != formats.end();
}

int formatPreferenceScore(const std::vector<rs2_format>& preferredFormats, rs2_format format)
{
    const auto position = std::find(preferredFormats.begin(), preferredFormats.end(), format);
    if (position == preferredFormats.end()) {
        return 0;
    }

    return 200 - static_cast<int>(std::distance(preferredFormats.begin(), position)) * 25;
}

int videoProfileScore(
    const rs2::video_stream_profile& profile,
    int preferredWidth,
    int preferredHeight,
    const std::vector<rs2_format>& preferredFormats)
{
    int score = formatPreferenceScore(preferredFormats, profile.format());
    score += std::max(0, 120 - std::abs(profile.width() - preferredWidth) / 8);
    score += std::max(0, 120 - std::abs(profile.height() - preferredHeight) / 8);
    score += profile.fps() == 30 ? 80 : std::max(0, 60 - std::abs(profile.fps() - 30));
    return score;
}

std::string streamTypeName(rs2_stream stream)
{
    return rs2_stream_to_string(stream);
}

std::string formatName(rs2_format format)
{
    return rs2_format_to_string(format);
}

std::string serialNumberOf(const rs2::device& device)
{
    if (!device.supports(RS2_CAMERA_INFO_SERIAL_NUMBER)) {
        return {};
    }

    return device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
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

RealSenseFrameSource::RealSenseFrameSource(RealSenseSettings settings)
    : settings_(std::move(settings))
    , context_()
    , pipeline_(context_)
{
}

RealSenseFrameSource::~RealSenseFrameSource()
{
    stop();
}

void RealSenseFrameSource::start()
{
    if (running_) {
        return;
    }

    rs2::config config;
    configureStreams(config);

    try {
        pipeline_.start(config);
        running_ = true;
        nextNoFrameWarning_ = std::chrono::steady_clock::now() + kNoFrameWarningInterval;
        std::cout << "RealSense pipeline started. Press q or Esc in the OpenCV dashboard to quit.\n";
    } catch (const rs2::error& error) {
        std::ostringstream message;
        message << "could not start RealSense pipeline: " << error.what()
                << " (function " << error.get_failed_function() << ", args "
                << error.get_failed_args() << ")\n"
                << describeRealSenseDevices();
        throw std::runtime_error(message.str());
    }
}

void RealSenseFrameSource::stop() noexcept
{
    if (!running_) {
        return;
    }

    try {
        pipeline_.stop();
    } catch (const rs2::error& error) {
        std::cerr << "Could not stop RealSense pipeline cleanly: " << error.what() << "\n";
    }

    running_ = false;
}

bool RealSenseFrameSource::poll(FrameBundle& output)
{
    if (!running_) {
        return false;
    }

    rs2::frameset frameset;
    if (!pipeline_.try_wait_for_frames(&frameset, kFrameWaitTimeoutMs)) {
        std::cerr << "RealSense frame wait timed out after " << kFrameWaitTimeoutMs << " ms.\n";

        const auto now = std::chrono::steady_clock::now();
        if (now >= nextNoFrameWarning_) {
            std::cerr << "Still waiting for RealSense frames from the active pipeline...\n";
            nextNoFrameWarning_ = now + kNoFrameWarningInterval;
        }

        return false;
    }

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

    if (bundle.empty()) {
        const auto now = std::chrono::steady_clock::now();
        if (now >= nextNoFrameWarning_) {
            std::cerr << "RealSense delivered frames, but none used a displayable video or motion format.\n";
            nextNoFrameWarning_ = now + kNoFrameWarningInterval;
        }
    }

    output = std::move(bundle);
    return !output.empty();
}

void RealSenseFrameSource::configureStreams(rs2::config& config)
{
    if (settings_.useAutoProfileProbe) {
        configureAutoDetectedStreams(config);
        return;
    }

    configureD455DefaultStreams(config);
}

void RealSenseFrameSource::configureD455DefaultStreams(rs2::config& config) const
{
    if (!settings_.serialNumber.empty()) {
        config.enable_device(settings_.serialNumber);
    }

    config.enable_stream(RS2_STREAM_DEPTH, 0, 848, 480, RS2_FORMAT_Z16, 30);
    config.enable_stream(RS2_STREAM_COLOR, 0, 1280, 720, RS2_FORMAT_BGR8, 30);

    std::cout << "Using D455 default video streams: depth, color";
    if (settings_.enableInfraredStreams) {
        config.enable_stream(RS2_STREAM_INFRARED, 1, 848, 480, RS2_FORMAT_Y8, 30);
        config.enable_stream(RS2_STREAM_INFRARED, 2, 848, 480, RS2_FORMAT_Y8, 30);
        std::cout << ", infrared 1, infrared 2";
    }
    std::cout << "\n" << std::flush;

    if (settings_.enableMotionStreams) {
        config.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, 63);
        config.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, 200);
        std::cout << "Motion streams requested: accel @63fps, gyro @200fps\n";
    }
}

void RealSenseFrameSource::configureAutoDetectedStreams(rs2::config& config)
{
    const rs2::device device = selectDevice();
    const std::string serialNumber = serialNumberOf(device);
    if (!serialNumber.empty()) {
        config.enable_device(serialNumber);
    }

    const struct VideoRequest {
        rs2_stream stream;
        int index;
        int width;
        int height;
        std::vector<rs2_format> formats;
    } videoRequests[] = {
        {RS2_STREAM_DEPTH, 0, 640, 480, {RS2_FORMAT_Z16}},
        {RS2_STREAM_COLOR, 0, 640, 480, {RS2_FORMAT_BGR8, RS2_FORMAT_RGB8, RS2_FORMAT_YUYV}},
    };

    int enabledStreams = 0;
    for (const auto& request : videoRequests) {
        auto selected = selectVideoProfile(
            device,
            request.stream,
            request.index,
            request.width,
            request.height,
            request.formats);

        if (!selected) {
            std::cout << "Skipping unavailable stream " << streamTypeName(request.stream);
            if (request.index > 0) {
                std::cout << " " << request.index;
            }
            std::cout << "\n";
            continue;
        }

        config.enable_stream(
            selected->stream,
            selected->index,
            selected->width,
            selected->height,
            selected->format,
            selected->fps);

        std::cout << "Enabled " << streamTypeName(selected->stream);
        if (selected->index > 0) {
            std::cout << " " << selected->index;
        }
        std::cout << " " << selected->width << "x" << selected->height << " "
                  << formatName(selected->format) << " @" << selected->fps << "fps\n";
        ++enabledStreams;
    }

    if (settings_.enableInfraredStreams) {
        const VideoRequest infraredRequests[] = {
            {RS2_STREAM_INFRARED, 1, 640, 480, {RS2_FORMAT_Y8, RS2_FORMAT_Y16}},
            {RS2_STREAM_INFRARED, 2, 640, 480, {RS2_FORMAT_Y8, RS2_FORMAT_Y16}},
            {RS2_STREAM_CONFIDENCE, 0, 640, 480, {RS2_FORMAT_RAW8, RS2_FORMAT_Y8}},
        };

        for (const auto& request : infraredRequests) {
            auto selected = selectVideoProfile(
                device,
                request.stream,
                request.index,
                request.width,
                request.height,
                request.formats);

            if (!selected) {
                std::cout << "Skipping unavailable stream " << streamTypeName(request.stream);
                if (request.index > 0) {
                    std::cout << " " << request.index;
                }
                std::cout << "\n";
                continue;
            }

            config.enable_stream(
                selected->stream,
                selected->index,
                selected->width,
                selected->height,
                selected->format,
                selected->fps);

            std::cout << "Enabled " << streamTypeName(selected->stream);
            if (selected->index > 0) {
                std::cout << " " << selected->index;
            }
            std::cout << " " << selected->width << "x" << selected->height << " "
                      << formatName(selected->format) << " @" << selected->fps << "fps\n";
            ++enabledStreams;
        }
    } else {
        std::cout << "Infrared streams disabled\n";
    }

    if (settings_.enableMotionStreams) {
        for (rs2_stream stream : {RS2_STREAM_ACCEL, RS2_STREAM_GYRO}) {
            auto selected = selectMotionProfile(device, stream);
            if (!selected) {
                std::cout << "Skipping unavailable stream " << streamTypeName(stream) << "\n";
                continue;
            }

            config.enable_stream(selected->stream, selected->format, selected->fps);
            std::cout << "Enabled " << streamTypeName(selected->stream) << " "
                      << formatName(selected->format) << " @" << selected->fps << "fps\n";
            ++enabledStreams;
        }
    }

    if (enabledStreams == 0) {
        throw std::runtime_error("no supported RealSense streams were found on the selected device");
    }
}

rs2::device RealSenseFrameSource::selectDevice() const
{
    const rs2::device_list devices = context_.query_devices();
    if (devices.size() == 0) {
        throw std::runtime_error("no Intel RealSense device was found");
    }

    for (const rs2::device& device : devices) {
        if (settings_.serialNumber.empty() || serialNumberOf(device) == settings_.serialNumber) {
            return device;
        }
    }

    std::ostringstream message;
    message << "no RealSense device with serial '" << settings_.serialNumber << "' was found. Available serials:";
    for (const rs2::device& device : devices) {
        message << " " << serialNumberOf(device);
    }

    throw std::runtime_error(message.str());
}

std::optional<RealSenseFrameSource::VideoProfileChoice> RealSenseFrameSource::selectVideoProfile(
    const rs2::device& device,
    rs2_stream stream,
    int index,
    int preferredWidth,
    int preferredHeight,
    const std::vector<rs2_format>& preferredFormats) const
{
    std::optional<VideoProfileChoice> best;

    for (const rs2::sensor& sensor : device.query_sensors()) {
        for (const rs2::stream_profile& profile : sensor.get_stream_profiles()) {
            const auto videoProfile = profile.as<rs2::video_stream_profile>();
            if (!videoProfile) {
                continue;
            }

            if (videoProfile.stream_type() != stream || videoProfile.stream_index() != index) {
                continue;
            }

            if (!hasFormat(preferredFormats, videoProfile.format())) {
                continue;
            }

            const int score = videoProfileScore(videoProfile, preferredWidth, preferredHeight, preferredFormats);
            if (!best || score > best->score) {
                best = VideoProfileChoice {
                    videoProfile.stream_type(),
                    videoProfile.stream_index(),
                    videoProfile.width(),
                    videoProfile.height(),
                    videoProfile.format(),
                    videoProfile.fps(),
                    score,
                };
            }
        }
    }

    return best;
}

std::optional<RealSenseFrameSource::MotionProfileChoice> RealSenseFrameSource::selectMotionProfile(
    const rs2::device& device,
    rs2_stream stream) const
{
    std::optional<MotionProfileChoice> best;

    for (const rs2::sensor& sensor : device.query_sensors()) {
        for (const rs2::stream_profile& profile : sensor.get_stream_profiles()) {
            if (profile.stream_type() != stream || profile.format() != RS2_FORMAT_MOTION_XYZ32F) {
                continue;
            }

            const int score = profile.fps() == 200 ? 200 : std::max(0, 120 - std::abs(profile.fps() - 200));
            if (!best || score > best->score) {
                best = MotionProfileChoice {
                    profile.stream_type(),
                    profile.format(),
                    profile.fps(),
                    score,
                };
            }
        }
    }

    return best;
}

std::optional<VideoFrame> RealSenseFrameSource::convertVideoFrame(const rs2::frame& frame) const
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

std::optional<PointCloudFrame> RealSenseFrameSource::convertPointCloudFrame(const rs2::frame& frame) const
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

std::optional<MotionSample> RealSenseFrameSource::convertMotionFrame(const rs2::frame& frame) const
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

std::string RealSenseFrameSource::frameName(const rs2::frame& frame) const
{
    const rs2::stream_profile profile = frame.get_profile();
    std::ostringstream name;
    name << profile.stream_name();

    if (profile.stream_index() > 0) {
        name << " " << profile.stream_index();
    }

    return name.str();
}

} // namespace rsv
