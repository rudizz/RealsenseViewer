#pragma once

#include "realsenseviewer/camera/FrameTypes.hpp"

namespace rsv {

class IFrameSource {
public:
    virtual ~IFrameSource() = default;

    virtual void start() = 0;
    virtual void stop() noexcept = 0;
    virtual bool poll(FrameBundle& output) = 0;
};

} // namespace rsv
