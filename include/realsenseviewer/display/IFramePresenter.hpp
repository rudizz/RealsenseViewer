#pragma once

#include "realsenseviewer/camera/FrameTypes.hpp"

namespace rsv {

class IFramePresenter {
public:
    virtual ~IFramePresenter() = default;

    virtual bool present(const FrameBundle& bundle) = 0;
    virtual bool idle() = 0;
};

} // namespace rsv
