#pragma once

#include "realsenseviewer/display/IFramePresenter.hpp"
#include "realsenseviewer/features/ObjectFeatureMatcher.hpp"

#include <opencv2/core.hpp>

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace pcl::visualization {
class PCLVisualizer;
} // namespace pcl::visualization

namespace rsv {

struct PclVisualizerDeleter {
    void operator()(pcl::visualization::PCLVisualizer* viewer) const;
};

class OpenCvFramePresenter final : public IFramePresenter {
public:
    ~OpenCvFramePresenter() override;

    bool present(const FrameBundle& bundle) override;
    bool idle() override;

private:
    static void handleMouse(int event, int x, int y, int flags, void* userdata);

    void setupWindow();
    void onMouse(int event, int x, int y);
    void openCalibrationImage();
    void setFeatureDetectorType(features::FeatureDetectorType detectorType);
    void setFeatureMatcherType(features::FeatureMatcherType matcherType);
    void updateFrames(const FrameBundle& bundle);
    void ensureStreamControl(const std::string& name);
    void updateFeatureMatch(const VideoFrame& frame);
    void renderDashboard();
    void drawSidebar(cv::Mat& canvas);
    void drawCalibrationButton(cv::Mat& canvas, int y);
    void drawStreamControl(cv::Mat& canvas, const std::string& streamName, int number, int y);
    void drawCalibrationPreview(cv::Mat& canvas, int y) const;
    void drawTile(cv::Mat& canvas, const cv::Rect& tileBounds, const VideoFrame& frame) const;
    void drawObjectMatchOverlay(cv::Mat& image) const;
    void drawMotionTile(cv::Mat& canvas, const cv::Rect& tileBounds) const;
    void updatePointCloudViewer();
    void shutdownPointCloudViewer();
    void spinPointCloudViewer();
    [[nodiscard]] std::vector<std::string> visibleVideoStreams() const;
    [[nodiscard]] std::vector<std::string> visiblePointCloudStreams() const;
    [[nodiscard]] std::vector<std::string> visibleMotionStreams() const;
    [[nodiscard]] bool isStreamVisible(const std::string& name) const;
    [[nodiscard]] bool keepRunning(int delayMs);

    std::map<std::string, VideoFrame> latestVideoFrames_;
    std::map<std::string, PointCloudFrame> latestPointCloudFrames_;
    std::map<std::string, MotionSample> latestMotionSamples_;
    std::map<std::string, bool> streamVisibility_;
    std::map<std::string, cv::Rect> streamHitBoxes_;
    std::vector<std::string> streamOrder_;
    cv::Rect calibrationButtonBounds_;
    cv::Rect detectorComboBounds_;
    std::vector<std::pair<features::FeatureDetectorType, cv::Rect>> detectorOptionBounds_;
    cv::Rect matcherComboBounds_;
    std::vector<std::pair<features::FeatureMatcherType, cv::Rect>> matcherOptionBounds_;
    cv::Rect objectDetectionBounds_;
    features::ObjectFeatureMatcher featureMatcher_;
    std::optional<features::MatchResult> latestObjectMatch_;
    std::string calibrationImageLabel_;
    std::string featureMatchStatus_;
    std::optional<double> latestFeatureMatchMs_;
    std::unique_ptr<pcl::visualization::PCLVisualizer, PclVisualizerDeleter> pointCloudViewer_;
    bool pointCloudViewerHasCloud_ = false;
    bool objectDetectionEnabled_ = false;
    bool detectorDropdownOpen_ = false;
    bool matcherDropdownOpen_ = false;
    bool windowInitialized_ = false;
};

} // namespace rsv
