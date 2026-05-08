#pragma once

#include "realsenseviewer/camera/IFrameSource.hpp"
#include "realsenseviewer/camera/RealSenseFrameProcessor.hpp"
#include "realsenseviewer/concurrency/ConcurrentQueue.hpp"

#include <librealsense2/rs.hpp>

#include <atomic>
#include <chrono>
#include <exception>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace rsv {

struct RealSenseSettings {
    std::string serialNumber;
    bool enableMotionStreams = false;
    bool useAutoProfileProbe = false;
    bool enableInfraredStreams = true;
};

class RealSenseFrameSource final : public IFrameSource {
public:
    explicit RealSenseFrameSource(RealSenseSettings settings = {});
    ~RealSenseFrameSource() override;

    void start() override;
    void stop() noexcept override;
    bool poll(FrameBundle& output) override;

private:
    struct VideoProfileChoice {
        rs2_stream stream = RS2_STREAM_ANY;
        int index = 0;
        int width = 0;
        int height = 0;
        rs2_format format = RS2_FORMAT_ANY;
        int fps = 0;
        int score = 0;
    };

    struct MotionProfileChoice {
        rs2_stream stream = RS2_STREAM_ANY;
        rs2_format format = RS2_FORMAT_ANY;
        int fps = 0;
        int score = 0;
    };

    void configureStreams(rs2::config& config);
    void configureD455DefaultStreams(rs2::config& config) const;
    void configureAutoDetectedStreams(rs2::config& config);
    [[nodiscard]] rs2::device selectDevice() const;
    [[nodiscard]] std::optional<VideoProfileChoice> selectVideoProfile(
        const rs2::device& device,
        rs2_stream stream,
        int index,
        int preferredWidth,
        int preferredHeight,
        const std::vector<rs2_format>& preferredFormats) const;
    [[nodiscard]] std::optional<MotionProfileChoice> selectMotionProfile(
        const rs2::device& device,
        rs2_stream stream) const;
    void captureLoop() noexcept;
    void processingLoop() noexcept;
    void recordWorkerException(std::exception_ptr exception) noexcept;
    void rethrowWorkerExceptionIfAny();

    RealSenseSettings settings_;
    rs2::context context_;
    rs2::pipeline pipeline_;
    RealSenseFrameProcessor processor_;
    ConcurrentQueue<rs2::frameset> capturedFrames_;
    ConcurrentQueue<FrameBundle> processedFrames_;
    std::thread captureThread_;
    std::thread processingThread_;
    std::mutex workerExceptionMutex_;
    std::exception_ptr workerException_;
    std::chrono::steady_clock::time_point nextNoFrameWarning_;
    std::atomic<bool> running_ { false };
};

} // namespace rsv
