#include "realsenseviewer/Application.hpp"

#include "realsenseviewer/camera/IFrameSource.hpp"
#include "realsenseviewer/display/IFramePresenter.hpp"

#include <exception>
#include <iostream>
#include <utility>

namespace rsv {

Application::Application(std::unique_ptr<IFrameSource> source, std::unique_ptr<IFramePresenter> presenter)
    : source_(std::move(source))
    , presenter_(std::move(presenter))
{
}

int Application::run()
{
    try {
        source_->start();

        bool running = true;
        while (running) {
            FrameBundle bundle;

            if (source_->poll(bundle)) {
                running = presenter_->present(bundle);
            } else {
                running = presenter_->idle();
            }
        }

        source_->stop();
        return 0;
    } catch (const std::exception& exception) {
        source_->stop();
        std::cerr << "RealsenseViewer failed: " << exception.what() << "\n";
        return 1;
    }
}

} // namespace rsv
