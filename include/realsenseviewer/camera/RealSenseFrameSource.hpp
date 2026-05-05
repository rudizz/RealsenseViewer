#pragma once

#include "realsenseviewer/camera/IFrameSource.hpp"

#include <librealsense2/rs.hpp>

#include <chrono>
#include <optional>
#include <string>
#include <vector>

namespace rsv {

[[nodiscard]] int pointCloudPixelStep();
[[nodiscard]] int minimumPointCloudPixelStep();
[[nodiscard]] int maximumPointCloudPixelStep();
void setPointCloudPixelStep(int pixelStep);
[[nodiscard]] bool pointCloudConversionEnabled();
void setPointCloudConversionEnabled(bool enabled);

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
    [[nodiscard]] std::optional<VideoFrame> convertVideoFrame(const rs2::frame& frame) const;
    [[nodiscard]] std::optional<PointCloudFrame> convertPointCloudFrame(const rs2::frame& frame) const;
    [[nodiscard]] std::optional<MotionSample> convertMotionFrame(const rs2::frame& frame) const;
    [[nodiscard]] std::string frameName(const rs2::frame& frame) const;

    RealSenseSettings settings_;
    rs2::context context_;
    rs2::pipeline pipeline_;
    std::chrono::steady_clock::time_point nextNoFrameWarning_;
    bool running_ = false;
};

} // namespace rsv
