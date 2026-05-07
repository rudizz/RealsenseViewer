#pragma once

#include "realsenseviewer/camera/FrameTypes.hpp"

#include <librealsense2/rs.hpp>

#include <optional>
#include <string>

namespace rsv {

class RealSenseFrameProcessor final {
public:
    [[nodiscard]] FrameBundle process(const rs2::frameset& frameset) const;

private:
    [[nodiscard]] std::optional<VideoFrame> convertVideoFrame(const rs2::frame& frame) const;
    [[nodiscard]] std::optional<PointCloudFrame> convertPointCloudFrame(const rs2::frame& frame) const;
    [[nodiscard]] std::optional<MotionSample> convertMotionFrame(const rs2::frame& frame) const;
    [[nodiscard]] std::string frameName(const rs2::frame& frame) const;
};

} // namespace rsv
