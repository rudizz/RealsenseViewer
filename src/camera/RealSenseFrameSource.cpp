#include "realsenseviewer/camera/RealSenseDiagnostics.hpp"
#include "realsenseviewer/camera/RealSenseFrameSource.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace rsv {
namespace {

constexpr unsigned int kFrameWaitTimeoutMs = 500;
constexpr auto kNoFrameWarningInterval = std::chrono::seconds(3);
constexpr auto kQueueFullWarningInterval = std::chrono::seconds(3);
constexpr std::size_t kCapturedFrameQueueCapacity = 3;
constexpr std::size_t kProcessedFrameQueueCapacity = 2;

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

} // namespace

RealSenseFrameSource::RealSenseFrameSource(RealSenseSettings settings)
    : settings_(std::move(settings))
    , context_()
    , pipeline_(context_)
    , capturedFrames_(kCapturedFrameQueueCapacity)
    , processedFrames_(kProcessedFrameQueueCapacity)
{
}

RealSenseFrameSource::~RealSenseFrameSource()
{
    stop();
}

void RealSenseFrameSource::start()
{
    if (running_.load()) {
        return;
    }

    rs2::config config;
    configureStreams(config);

    try {
        pipeline_.start(config);
        capturedFrames_.reset();
        processedFrames_.reset();
        {
            std::lock_guard<std::mutex> lock(workerExceptionMutex_);
            workerException_ = nullptr;
        }

        running_.store(true);
        nextNoFrameWarning_ = std::chrono::steady_clock::now() + kNoFrameWarningInterval;
        processingThread_ = std::thread(&RealSenseFrameSource::processingLoop, this);
        captureThread_ = std::thread(&RealSenseFrameSource::captureLoop, this);
        std::cout << "RealSense pipeline started. Press q or Esc in the OpenCV dashboard to quit.\n";
    } catch (const rs2::error& error) {
        std::ostringstream message;
        message << "could not start RealSense pipeline: " << error.what()
                << " (function " << error.get_failed_function() << ", args "
                << error.get_failed_args() << ")\n"
                << describeRealSenseDevices();
        throw std::runtime_error(message.str());
    } catch (...) {
        stop();
        throw;
    }
}

void RealSenseFrameSource::stop() noexcept
{
    if (!running_.exchange(false)) {
        return;
    }

    capturedFrames_.close();

    try {
        pipeline_.stop();
    } catch (const rs2::error& error) {
        std::cerr << "Could not stop RealSense pipeline cleanly: " << error.what() << "\n";
    }

    if (captureThread_.joinable()) {
        captureThread_.join();
    }

    capturedFrames_.close();

    if (processingThread_.joinable()) {
        processingThread_.join();
    }

    processedFrames_.close();
}

bool RealSenseFrameSource::poll(FrameBundle& output)
{
    rethrowWorkerExceptionIfAny();

    if (!running_.load()) {
        return false;
    }

    return processedFrames_.tryPop(output) && !output.empty();
}

void RealSenseFrameSource::captureLoop() noexcept
{
    auto nextQueueFullWarning = std::chrono::steady_clock::now();

    try {
        while (running_.load()) {
            rs2::frameset frameset;
            if (!pipeline_.try_wait_for_frames(&frameset, kFrameWaitTimeoutMs)) {
                const auto now = std::chrono::steady_clock::now();
                if (now >= nextNoFrameWarning_) {
                    std::cerr << "Still waiting for RealSense frames from the active pipeline...\n";
                    nextNoFrameWarning_ = now + kNoFrameWarningInterval;
                }

                continue;
            }

            if (capturedFrames_.tryPush(std::move(frameset))) {
                continue;
            }

            const auto now = std::chrono::steady_clock::now();
            if (now >= nextQueueFullWarning) {
                std::cerr << "Dropping RealSense frames because the capture FIFO is full.\n";
                nextQueueFullWarning = now + kQueueFullWarningInterval;
            }
        }
    } catch (...) {
        if (running_.load()) {
            recordWorkerException(std::current_exception());
        }
    }

    capturedFrames_.close();
}

void RealSenseFrameSource::processingLoop() noexcept
{
    auto nextNoDisplayableWarning = std::chrono::steady_clock::now() + kNoFrameWarningInterval;
    auto nextQueueFullWarning = std::chrono::steady_clock::now();

    try {
        rs2::frameset frameset;
        while (capturedFrames_.waitPop(frameset)) {
            FrameBundle bundle = processor_.process(frameset);
            if (bundle.empty()) {
                const auto now = std::chrono::steady_clock::now();
                if (now >= nextNoDisplayableWarning) {
                    std::cerr << "RealSense delivered frames, but none used a displayable video or motion format.\n";
                    nextNoDisplayableWarning = now + kNoFrameWarningInterval;
                }
                continue;
            }

            if (processedFrames_.tryPush(std::move(bundle))) {
                continue;
            }

            const auto now = std::chrono::steady_clock::now();
            if (now >= nextQueueFullWarning) {
                std::cerr << "Dropping processed RealSense frames because the presentation FIFO is full.\n";
                nextQueueFullWarning = now + kQueueFullWarningInterval;
            }
        }
    } catch (...) {
        if (running_.load()) {
            recordWorkerException(std::current_exception());
        }
    }

    processedFrames_.close();
}

void RealSenseFrameSource::recordWorkerException(std::exception_ptr exception) noexcept
{
    std::lock_guard<std::mutex> lock(workerExceptionMutex_);
    if (!workerException_) {
        workerException_ = exception;
    }
}

void RealSenseFrameSource::rethrowWorkerExceptionIfAny()
{
    std::exception_ptr exception;
    {
        std::lock_guard<std::mutex> lock(workerExceptionMutex_);
        exception = workerException_;
        workerException_ = nullptr;
    }

    if (exception) {
        std::rethrow_exception(exception);
    }
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

} // namespace rsv
