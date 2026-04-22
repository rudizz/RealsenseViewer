#pragma once

#include <memory>

namespace rsv {

class IFramePresenter;
class IFrameSource;

class Application {
public:
    Application(std::unique_ptr<IFrameSource> source, std::unique_ptr<IFramePresenter> presenter);

    int run();

private:
    std::unique_ptr<IFrameSource> source_;
    std::unique_ptr<IFramePresenter> presenter_;
};

} // namespace rsv
