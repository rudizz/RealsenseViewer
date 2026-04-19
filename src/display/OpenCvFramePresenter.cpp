#include "realsenseviewer/display/OpenCvFramePresenter.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iomanip>
#include <sstream>

namespace rsv {
namespace {

constexpr int kExitKeyQ = 'q';
constexpr int kExitKeyEsc = 27;

std::string motionLine(const MotionSample& sample)
{
    std::ostringstream line;
    line << std::fixed << std::setprecision(3)
         << sample.name << ": "
         << "x=" << sample.value[0] << ", "
         << "y=" << sample.value[1] << ", "
         << "z=" << sample.value[2] << " " << sample.units;
    return line.str();
}

void putPanelText(cv::Mat& image, const std::string& text, int y, double scale = 0.55)
{
    cv::putText(
        image,
        text,
        cv::Point(18, y),
        cv::FONT_HERSHEY_SIMPLEX,
        scale,
        cv::Scalar(230, 230, 230),
        1,
        cv::LINE_AA);
}

} // namespace

bool OpenCvFramePresenter::present(const FrameBundle& bundle)
{
    for (const VideoFrame& frame : bundle.videoFrames) {
        showVideoFrame(frame);
    }

    for (const MotionSample& sample : bundle.motionSamples) {
        latestMotionSamples_[sample.name] = sample;
    }

    if (!latestMotionSamples_.empty()) {
        showMotionPanel();
    }

    return keepRunning(1);
}

bool OpenCvFramePresenter::idle()
{
    return keepRunning(1);
}

void OpenCvFramePresenter::showVideoFrame(const VideoFrame& frame) const
{
    if (frame.image.empty()) {
        return;
    }

    cv::Mat display = frame.image.clone();
    const std::string header = frame.name + "  " + frame.details;

    cv::rectangle(display, cv::Rect(0, 0, display.cols, 34), cv::Scalar(0, 0, 0), cv::FILLED);
    cv::putText(
        display,
        header,
        cv::Point(12, 23),
        cv::FONT_HERSHEY_SIMPLEX,
        0.62,
        cv::Scalar(255, 255, 255),
        1,
        cv::LINE_AA);

    cv::imshow(frame.name, display);
}

void OpenCvFramePresenter::showMotionPanel()
{
    const int panelHeight = 92 + static_cast<int>(latestMotionSamples_.size()) * 36;
    cv::Mat panel(panelHeight, 720, CV_8UC3, cv::Scalar(24, 24, 24));

    putPanelText(panel, "RealSense motion streams", 30, 0.7);
    putPanelText(panel, "Press q or Esc in any OpenCV window to quit", 58, 0.5);

    int y = 98;
    for (const auto& [name, sample] : latestMotionSamples_) {
        (void)name;
        putPanelText(panel, motionLine(sample), y);
        y += 36;
    }

    cv::imshow("RealSense Motion", panel);
}

bool OpenCvFramePresenter::keepRunning(int delayMs) const
{
    const int key = cv::waitKey(delayMs);
    return key != kExitKeyQ && key != kExitKeyEsc;
}

} // namespace rsv
