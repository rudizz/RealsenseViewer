#include "realsenseviewer/display/OpenCvFramePresenter.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/visualization/pcl_visualizer.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <optional>
#include <sstream>

namespace rsv {

void PclVisualizerDeleter::operator()(pcl::visualization::PCLVisualizer* viewer) const
{
    delete viewer;
}

namespace {

constexpr const char* kWindowName = "RealSense Viewer";
constexpr int kExitKeyQ = 'q';
constexpr int kExitKeyEsc = 27;
constexpr int kDashboardWidth = 1280;
constexpr int kDashboardHeight = 820;
constexpr int kSidebarWidth = 300;
constexpr int kGap = 12;
constexpr int kTileHeaderHeight = 38;
constexpr int kStreamListStartY = 156;
constexpr int kDetectionSectionGap = 20;
constexpr int kCalibrationPreviewHeight = 150;
constexpr int kDetectorComboHeight = 32;
constexpr int kDetectorOptionHeight = 28;
constexpr int kMatcherComboHeight = 32;
constexpr int kMatcherOptionHeight = 28;
constexpr const char* kPointCloudWindowName = "RealSense Point Cloud";
constexpr const char* kPointCloudId = "realsense-point-cloud";
constexpr const char* kPointCloudHelpTextId = "point-cloud-help-text";

const cv::Scalar kBackgroundColor(20, 22, 24);
const cv::Scalar kSidebarColor(30, 34, 38);
const cv::Scalar kPanelColor(36, 39, 43);
const cv::Scalar kPanelBorderColor(80, 86, 92);
const cv::Scalar kAccentColor(64, 173, 126);
const cv::Scalar kTextColor(235, 238, 240);
const cv::Scalar kMutedTextColor(156, 164, 170);
const cv::Scalar kWarningColor(92, 186, 255);

bool isColorStreamName(const std::string& name)
{
    return name == "Color" || name == "color";
}

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

void putTextLine(
    cv::Mat& image,
    const std::string& text,
    const cv::Point& origin,
    double scale = 0.55,
    const cv::Scalar& color = kTextColor,
    int thickness = 1)
{
    cv::putText(
        image,
        text,
        origin,
        cv::FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        thickness,
        cv::LINE_AA);
}

std::string fitText(const std::string& text, int maxWidth, double scale, int thickness = 1)
{
    int baseline = 0;
    if (cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, scale, thickness, &baseline).width <= maxWidth) {
        return text;
    }

    constexpr const char* suffix = "...";
    std::string clipped = text;
    while (!clipped.empty()) {
        clipped.pop_back();
        const std::string candidate = clipped + suffix;
        if (cv::getTextSize(candidate, cv::FONT_HERSHEY_SIMPLEX, scale, thickness, &baseline).width <= maxWidth) {
            return candidate;
        }
    }

    return suffix;
}

void drawEmptyMessage(cv::Mat& canvas, const cv::Rect& area, const std::string& message)
{
    const std::string text = fitText(message, area.width - 40, 0.7);
    int baseline = 0;
    const cv::Size size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.7, 1, &baseline);
    const cv::Point origin(
        area.x + (area.width - size.width) / 2,
        area.y + (area.height + size.height) / 2);
    putTextLine(canvas, text, origin, 0.7, kMutedTextColor);
}

std::string basenameOf(const std::string& path)
{
    if (path.empty()) {
        return {};
    }

    return std::filesystem::path(path).filename().string();
}

std::optional<std::string> chooseImageFile()
{
#if defined(__APPLE__)
    constexpr const char* command =
        "osascript "
        "-e 'set pickedFile to choose file with prompt \"Choose calibration image\"' "
        "-e 'POSIX path of pickedFile'";

    std::array<char, 512> buffer {};
    std::string output;
    FILE* pipe = popen(command, "r");
    if (pipe == nullptr) {
        return std::nullopt;
    }

    while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr) {
        output += buffer.data();
    }

    const int result = pclose(pipe);
    if (result != 0) {
        return std::nullopt;
    }

    while (!output.empty() && (output.back() == '\n' || output.back() == '\r')) {
        output.pop_back();
    }

    if (output.empty()) {
        return std::nullopt;
    }

    return output;
#else
    return std::nullopt;
#endif
}

const char* detectorTypeLabel(features::FeatureDetectorType detectorType)
{
    switch (detectorType) {
    case features::FeatureDetectorType::Sift:
        return "SIFT";
    case features::FeatureDetectorType::Orb:
        return "ORB";
    case features::FeatureDetectorType::Surf:
        return "SURF";
    }

    return "Unknown";
}

const char* matcherTypeLabel(features::FeatureMatcherType matcherType)
{
    switch (matcherType) {
    case features::FeatureMatcherType::BruteForce:
        return "Brute Force";
    case features::FeatureMatcherType::Flann:
        return "FLANN";
    }

    return "Unknown";
}

} // namespace

OpenCvFramePresenter::~OpenCvFramePresenter()
{
    shutdownPointCloudViewer();

    if (!windowInitialized_) {
        return;
    }

    try {
        cv::destroyWindow(kWindowName);
    } catch (const cv::Exception&) {
    }
}

bool OpenCvFramePresenter::present(const FrameBundle& bundle)
{
    setupWindow();
    updateFrames(bundle);
    updatePointCloudViewer();
    renderDashboard();
    return keepRunning(1);
}

bool OpenCvFramePresenter::idle()
{
    setupWindow();
    updatePointCloudViewer();
    renderDashboard();
    return keepRunning(15);
}

void OpenCvFramePresenter::handleMouse(int event, int x, int y, int flags, void* userdata)
{
    (void)flags;

    auto* presenter = static_cast<OpenCvFramePresenter*>(userdata);
    if (presenter == nullptr) {
        return;
    }

    presenter->onMouse(event, x, y);
}

void OpenCvFramePresenter::setupWindow()
{
    if (windowInitialized_) {
        return;
    }

    cv::namedWindow(kWindowName, cv::WINDOW_NORMAL);
    cv::resizeWindow(kWindowName, kDashboardWidth, kDashboardHeight);
    cv::setMouseCallback(kWindowName, &OpenCvFramePresenter::handleMouse, this);
    windowInitialized_ = true;
}

void OpenCvFramePresenter::onMouse(int event, int x, int y)
{
    if (event != cv::EVENT_LBUTTONUP) {
        return;
    }

    const cv::Point click(x, y);
    if (detectorDropdownOpen_) {
        for (const auto& [detectorType, bounds] : detectorOptionBounds_) {
            if (!bounds.contains(click)) {
                continue;
            }

            setFeatureDetectorType(detectorType);
            detectorDropdownOpen_ = false;
            renderDashboard();
            return;
        }

        if (!detectorComboBounds_.contains(click)) {
            detectorDropdownOpen_ = false;
            renderDashboard();
            return;
        }
    }

    if (matcherDropdownOpen_) {
        for (const auto& [matcherType, bounds] : matcherOptionBounds_) {
            if (!bounds.contains(click)) {
                continue;
            }

            setFeatureMatcherType(matcherType);
            matcherDropdownOpen_ = false;
            renderDashboard();
            return;
        }

        if (!matcherComboBounds_.contains(click)) {
            matcherDropdownOpen_ = false;
            renderDashboard();
            return;
        }
    }

    if (calibrationButtonBounds_.contains(click)) {
        detectorDropdownOpen_ = false;
        matcherDropdownOpen_ = false;
        openCalibrationImage();
        renderDashboard();
        return;
    }

    if (detectorComboBounds_.contains(click)) {
        detectorDropdownOpen_ = !detectorDropdownOpen_;
        matcherDropdownOpen_ = false;
        renderDashboard();
        return;
    }

    if (matcherComboBounds_.contains(click)) {
        matcherDropdownOpen_ = !matcherDropdownOpen_;
        detectorDropdownOpen_ = false;
        renderDashboard();
        return;
    }

    if (objectDetectionBounds_.contains(click)) {
        detectorDropdownOpen_ = false;
        matcherDropdownOpen_ = false;
        objectDetectionEnabled_ = !objectDetectionEnabled_;
        if (!objectDetectionEnabled_) {
            latestObjectMatch_.reset();
            featureMatchStatus_ = featureMatcher_.hasCalibration()
                ? "Object detection paused"
                : "Load calibration image first";
        } else if (!featureMatcher_.hasCalibration()) {
            latestObjectMatch_.reset();
            featureMatchStatus_ = "Load calibration image first";
        } else {
            const auto colorFrame = latestVideoFrames_.find("Color");
            if (colorFrame != latestVideoFrames_.end()) {
                updateFeatureMatch(colorFrame->second);
            } else {
                featureMatchStatus_ = "Waiting for Color stream";
            }
        }
        renderDashboard();
        return;
    }

    for (const auto& [streamName, bounds] : streamHitBoxes_) {
        if (!bounds.contains(click)) {
            continue;
        }

        detectorDropdownOpen_ = false;
        matcherDropdownOpen_ = false;
        streamVisibility_[streamName] = !isStreamVisible(streamName);
        if (latestPointCloudFrames_.find(streamName) != latestPointCloudFrames_.end()) {
            updatePointCloudViewer();
        }
        renderDashboard();
        return;
    }
}

void OpenCvFramePresenter::setFeatureDetectorType(features::FeatureDetectorType detectorType)
{
    if (featureMatcher_.detectorType() == detectorType) {
        return;
    }

    latestObjectMatch_.reset();
    const bool calibrationReloaded = featureMatcher_.setDetectorType(detectorType);
    if (!calibrationReloaded) {
        calibrationImageLabel_.clear();
        latestFeatureMatchMs_.reset();
        featureMatchStatus_ = std::string(featureMatcher_.detectorName())
            + " selected; could not extract enough features";
        return;
    }

    if (!featureMatcher_.hasCalibration()) {
        latestFeatureMatchMs_.reset();
        featureMatchStatus_ = std::string(featureMatcher_.detectorName()) + " selected";
        return;
    }

    std::ostringstream status;
    status << featureMatcher_.detectorName() << " selected: "
           << featureMatcher_.calibrationKeypointCount() << " features";
    featureMatchStatus_ = status.str();

    const auto colorFrame = latestVideoFrames_.find("Color");
    if (objectDetectionEnabled_ && colorFrame != latestVideoFrames_.end()) {
        updateFeatureMatch(colorFrame->second);
    }
}

void OpenCvFramePresenter::setFeatureMatcherType(features::FeatureMatcherType matcherType)
{
    if (featureMatcher_.matcherType() == matcherType) {
        return;
    }

    featureMatcher_.setMatcherType(matcherType);
    latestObjectMatch_.reset();

    if (!featureMatcher_.hasCalibration()) {
        latestFeatureMatchMs_.reset();
        featureMatchStatus_ = std::string(featureMatcher_.matcherName()) + " matcher selected";
        return;
    }

    const auto colorFrame = latestVideoFrames_.find("Color");
    if (objectDetectionEnabled_ && colorFrame != latestVideoFrames_.end()) {
        updateFeatureMatch(colorFrame->second);
        return;
    }

    featureMatchStatus_ = std::string(featureMatcher_.matcherName()) + " matcher selected";
}

void OpenCvFramePresenter::openCalibrationImage()
{
    const std::optional<std::string> selectedImage = chooseImageFile();
    if (!selectedImage) {
        featureMatchStatus_ = "No calibration image selected";
        return;
    }

    if (!featureMatcher_.loadCalibrationImage(*selectedImage)) {
        latestObjectMatch_.reset();
        latestFeatureMatchMs_.reset();
        calibrationImageLabel_.clear();
        featureMatchStatus_ = "Could not extract enough features";
        return;
    }

    calibrationImageLabel_ = basenameOf(*selectedImage);
    latestObjectMatch_.reset();
    latestFeatureMatchMs_.reset();

    std::ostringstream status;
    status << featureMatcher_.detectorName() << " calibration loaded: "
           << featureMatcher_.calibrationKeypointCount() << " features";
    featureMatchStatus_ = status.str();

    const auto colorFrame = latestVideoFrames_.find("Color");
    if (objectDetectionEnabled_ && colorFrame != latestVideoFrames_.end()) {
        updateFeatureMatch(colorFrame->second);
    }
}

void OpenCvFramePresenter::updateFrames(const FrameBundle& bundle)
{
    for (const VideoFrame& frame : bundle.videoFrames) {
        ensureStreamControl(frame.name);
        latestVideoFrames_[frame.name] = frame;
        if (objectDetectionEnabled_ && isColorStreamName(frame.name)) {
            updateFeatureMatch(frame);
        }
    }

    for (const PointCloudFrame& frame : bundle.pointCloudFrames) {
        ensureStreamControl(frame.name);
        latestPointCloudFrames_[frame.name] = frame;
    }

    for (const MotionSample& sample : bundle.motionSamples) {
        ensureStreamControl(sample.name);
        latestMotionSamples_[sample.name] = sample;
    }
}

void OpenCvFramePresenter::ensureStreamControl(const std::string& name)
{
    if (streamVisibility_.find(name) != streamVisibility_.end()) {
        return;
    }

    streamVisibility_[name] = true;
    streamOrder_.push_back(name);
}

void OpenCvFramePresenter::updateFeatureMatch(const VideoFrame& frame)
{
    if (!objectDetectionEnabled_) {
        latestObjectMatch_.reset();
        latestFeatureMatchMs_.reset();
        return;
    }

    if (!featureMatcher_.hasCalibration()) {
        latestObjectMatch_.reset();
        latestFeatureMatchMs_.reset();
        featureMatchStatus_ = "Load calibration image first";
        return;
    }

    try {
        const auto matchStart = std::chrono::steady_clock::now();
        latestObjectMatch_ = featureMatcher_.match(frame.image);
        const auto matchEnd = std::chrono::steady_clock::now();
        latestFeatureMatchMs_ = std::chrono::duration<double, std::milli>(matchEnd - matchStart).count();
    } catch (const cv::Exception&) {
        latestObjectMatch_.reset();
        latestFeatureMatchMs_.reset();
        featureMatchStatus_ = std::string(featureMatcher_.detectorName()) + " + "
            + featureMatcher_.matcherName() + " failed";
        return;
    }

    if (!latestObjectMatch_) {
        featureMatchStatus_ = "Object not found with strong confidence";
        return;
    }

    std::ostringstream status;
    status << "Object found: " << latestObjectMatch_->inliers << "/" << latestObjectMatch_->goodMatches
           << " inliers, confidence " << std::fixed << std::setprecision(2)
           << latestObjectMatch_->confidence;
    featureMatchStatus_ = status.str();
}

void OpenCvFramePresenter::renderDashboard()
{
    cv::Mat canvas(kDashboardHeight, kDashboardWidth, CV_8UC3, kBackgroundColor);
    streamHitBoxes_.clear();
    detectorOptionBounds_.clear();
    matcherOptionBounds_.clear();

    drawSidebar(canvas);

    const cv::Rect contentArea(
        kSidebarWidth + kGap,
        kGap,
        canvas.cols - kSidebarWidth - (2 * kGap),
        canvas.rows - (2 * kGap));

    const std::vector<std::string> videoStreams = visibleVideoStreams();
    const std::vector<std::string> pointCloudStreams = visiblePointCloudStreams();
    const std::vector<std::string> motionStreams = visibleMotionStreams();
    const int tileCount = static_cast<int>(videoStreams.size()) + (motionStreams.empty() ? 0 : 1);

    if (tileCount == 0) {
        drawEmptyMessage(
            canvas,
            contentArea,
            pointCloudStreams.empty() ? "Select one or more streams from the left panel"
                                      : "Point Cloud is open in its 3D window");
        cv::imshow(kWindowName, canvas);
        return;
    }

    const int columns = std::max(1, std::min(3, static_cast<int>(std::ceil(std::sqrt(tileCount)))));
    const int rows = static_cast<int>(std::ceil(static_cast<double>(tileCount) / columns));
    const int tileWidth = (contentArea.width - ((columns - 1) * kGap)) / columns;
    const int tileHeight = (contentArea.height - ((rows - 1) * kGap)) / rows;

    int tileIndex = 0;
    const auto drawAtNextPosition = [&](const auto& draw) {
        const int row = tileIndex / columns;
        const int column = tileIndex % columns;
        const cv::Rect tileBounds(
            contentArea.x + (column * (tileWidth + kGap)),
            contentArea.y + (row * (tileHeight + kGap)),
            tileWidth,
            tileHeight);
        draw(tileBounds);
        ++tileIndex;
    };

    for (const std::string& streamName : videoStreams) {
        const auto frame = latestVideoFrames_.find(streamName);
        if (frame == latestVideoFrames_.end()) {
            continue;
        }

        drawAtNextPosition([&](const cv::Rect& tileBounds) {
            drawTile(canvas, tileBounds, frame->second);
        });
    }

    if (!motionStreams.empty()) {
        drawAtNextPosition([&](const cv::Rect& tileBounds) {
            drawMotionTile(canvas, tileBounds);
        });
    }

    cv::imshow(kWindowName, canvas);
}

void OpenCvFramePresenter::drawSidebar(cv::Mat& canvas)
{
    cv::rectangle(canvas, cv::Rect(0, 0, kSidebarWidth, canvas.rows), kSidebarColor, cv::FILLED);
    cv::line(
        canvas,
        cv::Point(kSidebarWidth, 0),
        cv::Point(kSidebarWidth, canvas.rows),
        cv::Scalar(62, 68, 74),
        1,
        cv::LINE_AA);

    putTextLine(canvas, "RealSense Viewer", cv::Point(18, 38), 0.8, kTextColor, 2);
    putTextLine(canvas, "Click streams to show or hide them", cv::Point(18, 72), 0.48, kMutedTextColor);
    putTextLine(canvas, "Point Cloud opens a 3D window", cv::Point(18, 96), 0.48, kMutedTextColor);
    putTextLine(canvas, "q or Esc quits", cv::Point(18, 120), 0.48, kMutedTextColor);

    int y = kStreamListStartY;
    if (streamOrder_.empty()) {
        putTextLine(canvas, "Waiting for streams...", cv::Point(18, y + 4), 0.55, kMutedTextColor);
        y += 34;
    } else {
        int number = 1;
        for (const std::string& streamName : streamOrder_) {
            drawStreamControl(canvas, streamName, number, y);
            y += 34;
            ++number;
        }
    }

    drawCalibrationButton(canvas, y + kDetectionSectionGap);
}

void OpenCvFramePresenter::drawCalibrationButton(cv::Mat& canvas, int y)
{
    putTextLine(canvas, "Object detection", cv::Point(18, y), 0.62, kTextColor, 1);

    calibrationButtonBounds_ = cv::Rect(12, y + 14, kSidebarWidth - 24, 34);
    cv::rectangle(canvas, calibrationButtonBounds_, cv::Scalar(46, 58, 61), cv::FILLED);
    cv::rectangle(canvas, calibrationButtonBounds_, kAccentColor, 1, cv::LINE_AA);
    putTextLine(
        canvas,
        "Open calibration image",
        cv::Point(calibrationButtonBounds_.x + 14, calibrationButtonBounds_.y + 23),
        0.55,
        kTextColor);

    const int detectorLabelY = y + 72;
    putTextLine(canvas, "Detector", cv::Point(18, detectorLabelY), 0.47, kMutedTextColor);

    detectorComboBounds_ = cv::Rect(12, detectorLabelY + 10, kSidebarWidth - 24, kDetectorComboHeight);
    cv::rectangle(canvas, detectorComboBounds_, cv::Scalar(41, 48, 52), cv::FILLED);
    cv::rectangle(
        canvas,
        detectorComboBounds_,
        detectorDropdownOpen_ ? kAccentColor : kPanelBorderColor,
        1,
        cv::LINE_AA);
    putTextLine(
        canvas,
        detectorTypeLabel(featureMatcher_.detectorType()),
        cv::Point(detectorComboBounds_.x + 12, detectorComboBounds_.y + 22),
        0.55,
        kTextColor);

    const int arrowX = detectorComboBounds_.x + detectorComboBounds_.width - 22;
    const int arrowY = detectorComboBounds_.y + 13;
    if (detectorDropdownOpen_) {
        cv::line(canvas, cv::Point(arrowX - 5, arrowY + 5), cv::Point(arrowX, arrowY), kMutedTextColor, 1, cv::LINE_AA);
        cv::line(canvas, cv::Point(arrowX, arrowY), cv::Point(arrowX + 5, arrowY + 5), kMutedTextColor, 1, cv::LINE_AA);
    } else {
        cv::line(canvas, cv::Point(arrowX - 5, arrowY), cv::Point(arrowX, arrowY + 5), kMutedTextColor, 1, cv::LINE_AA);
        cv::line(canvas, cv::Point(arrowX, arrowY + 5), cv::Point(arrowX + 5, arrowY), kMutedTextColor, 1, cv::LINE_AA);
    }

    const int matcherLabelY = y + 132;
    putTextLine(canvas, "Matcher", cv::Point(18, matcherLabelY), 0.47, kMutedTextColor);

    matcherComboBounds_ = cv::Rect(12, matcherLabelY + 10, kSidebarWidth - 24, kMatcherComboHeight);
    cv::rectangle(canvas, matcherComboBounds_, cv::Scalar(41, 48, 52), cv::FILLED);
    cv::rectangle(
        canvas,
        matcherComboBounds_,
        matcherDropdownOpen_ ? kAccentColor : kPanelBorderColor,
        1,
        cv::LINE_AA);
    putTextLine(
        canvas,
        matcherTypeLabel(featureMatcher_.matcherType()),
        cv::Point(matcherComboBounds_.x + 12, matcherComboBounds_.y + 22),
        0.55,
        kTextColor);

    const int matcherArrowX = matcherComboBounds_.x + matcherComboBounds_.width - 22;
    const int matcherArrowY = matcherComboBounds_.y + 13;
    if (matcherDropdownOpen_) {
        cv::line(
            canvas,
            cv::Point(matcherArrowX - 5, matcherArrowY + 5),
            cv::Point(matcherArrowX, matcherArrowY),
            kMutedTextColor,
            1,
            cv::LINE_AA);
        cv::line(
            canvas,
            cv::Point(matcherArrowX, matcherArrowY),
            cv::Point(matcherArrowX + 5, matcherArrowY + 5),
            kMutedTextColor,
            1,
            cv::LINE_AA);
    } else {
        cv::line(
            canvas,
            cv::Point(matcherArrowX - 5, matcherArrowY),
            cv::Point(matcherArrowX, matcherArrowY + 5),
            kMutedTextColor,
            1,
            cv::LINE_AA);
        cv::line(
            canvas,
            cv::Point(matcherArrowX, matcherArrowY + 5),
            cv::Point(matcherArrowX + 5, matcherArrowY),
            kMutedTextColor,
            1,
            cv::LINE_AA);
    }

    const int detectionY = y + 204;
    objectDetectionBounds_ = cv::Rect(12, detectionY - 23, kSidebarWidth - 24, 30);
    const cv::Rect boxBounds(18, detectionY - 16, 18, 18);
    cv::rectangle(
        canvas,
        objectDetectionBounds_,
        objectDetectionEnabled_ ? cv::Scalar(41, 48, 52) : kSidebarColor,
        cv::FILLED);
    cv::rectangle(canvas, boxBounds, objectDetectionEnabled_ ? kAccentColor : kMutedTextColor, 1, cv::LINE_AA);
    if (objectDetectionEnabled_) {
        cv::rectangle(canvas, boxBounds + cv::Size(-1, -1), kAccentColor, cv::FILLED);
        cv::line(
            canvas,
            cv::Point(boxBounds.x + 4, boxBounds.y + 9),
            cv::Point(boxBounds.x + 8, boxBounds.y + 13),
            cv::Scalar(20, 22, 24),
            2,
            cv::LINE_AA);
        cv::line(
            canvas,
            cv::Point(boxBounds.x + 8, boxBounds.y + 13),
            cv::Point(boxBounds.x + 15, boxBounds.y + 5),
            cv::Scalar(20, 22, 24),
            2,
            cv::LINE_AA);
    }

    putTextLine(
        canvas,
        "Run object detection",
        cv::Point(46, detectionY),
        0.55,
        objectDetectionEnabled_ ? kTextColor : kMutedTextColor);

    const std::string imageLabel = calibrationImageLabel_.empty()
        ? "Calibration: none"
        : "Calibration: " + calibrationImageLabel_;
    putTextLine(
        canvas,
        fitText(imageLabel, kSidebarWidth - 36, 0.45),
        cv::Point(18, detectionY + 34),
        0.45,
        calibrationImageLabel_.empty() ? kMutedTextColor : kTextColor);

    if (!featureMatchStatus_.empty()) {
        putTextLine(
            canvas,
            fitText(featureMatchStatus_, kSidebarWidth - 36, 0.43),
            cv::Point(18, detectionY + 58),
            0.43,
            latestObjectMatch_ ? kAccentColor : kWarningColor);
    }

    if (latestFeatureMatchMs_) {
        std::ostringstream timing;
        timing << "Match time: " << std::fixed << std::setprecision(1) << *latestFeatureMatchMs_ << " ms";
        putTextLine(
            canvas,
            fitText(timing.str(), kSidebarWidth - 36, 0.43),
            cv::Point(18, detectionY + 82),
            0.43,
            kMutedTextColor);
    }

    drawCalibrationPreview(canvas, detectionY + 100);

    if (detectorDropdownOpen_) {
        const std::array<features::FeatureDetectorType, 3> detectorTypes = {
            features::FeatureDetectorType::Sift,
            features::FeatureDetectorType::Orb,
            features::FeatureDetectorType::Surf,
        };
        int optionY = detectorComboBounds_.y + detectorComboBounds_.height;
        for (features::FeatureDetectorType detectorType : detectorTypes) {
            const cv::Rect optionBounds(
                detectorComboBounds_.x,
                optionY,
                detectorComboBounds_.width,
                kDetectorOptionHeight);
            detectorOptionBounds_.push_back({ detectorType, optionBounds });

            const bool selected = featureMatcher_.detectorType() == detectorType;
            cv::rectangle(
                canvas,
                optionBounds,
                selected ? cv::Scalar(49, 70, 64) : cv::Scalar(34, 38, 42),
                cv::FILLED);
            cv::rectangle(canvas, optionBounds, kPanelBorderColor, 1, cv::LINE_AA);
            putTextLine(
                canvas,
                detectorTypeLabel(detectorType),
                cv::Point(optionBounds.x + 12, optionBounds.y + 20),
                0.52,
                selected ? kTextColor : kMutedTextColor);
            optionY += kDetectorOptionHeight;
        }
    }

    if (matcherDropdownOpen_) {
        const std::array<features::FeatureMatcherType, 2> matcherTypes = {
            features::FeatureMatcherType::BruteForce,
            features::FeatureMatcherType::Flann,
        };
        int optionY = matcherComboBounds_.y + matcherComboBounds_.height;
        for (features::FeatureMatcherType matcherType : matcherTypes) {
            const cv::Rect optionBounds(
                matcherComboBounds_.x,
                optionY,
                matcherComboBounds_.width,
                kMatcherOptionHeight);
            matcherOptionBounds_.push_back({ matcherType, optionBounds });

            const bool selected = featureMatcher_.matcherType() == matcherType;
            cv::rectangle(
                canvas,
                optionBounds,
                selected ? cv::Scalar(49, 70, 64) : cv::Scalar(34, 38, 42),
                cv::FILLED);
            cv::rectangle(canvas, optionBounds, kPanelBorderColor, 1, cv::LINE_AA);
            putTextLine(
                canvas,
                matcherTypeLabel(matcherType),
                cv::Point(optionBounds.x + 12, optionBounds.y + 20),
                0.52,
                selected ? kTextColor : kMutedTextColor);
            optionY += kMatcherOptionHeight;
        }
    }
}

void OpenCvFramePresenter::drawCalibrationPreview(cv::Mat& canvas, int y) const
{
    const cv::Mat& preview = featureMatcher_.calibrationFeatureImage();
    if (preview.empty() || y >= canvas.rows - 12) {
        return;
    }

    const cv::Rect previewBounds(
        12,
        y,
        kSidebarWidth - 24,
        std::min(kCalibrationPreviewHeight, canvas.rows - y - 12));
    cv::rectangle(canvas, previewBounds, kPanelColor, cv::FILLED);
    cv::rectangle(canvas, previewBounds, kPanelBorderColor, 1, cv::LINE_AA);

    const double scale = std::min(
        static_cast<double>(previewBounds.width - 12) / preview.cols,
        static_cast<double>(previewBounds.height - 12) / preview.rows);
    const int imageWidth = std::max(1, static_cast<int>(std::round(preview.cols * scale)));
    const int imageHeight = std::max(1, static_cast<int>(std::round(preview.rows * scale)));

    cv::Mat resized;
    cv::resize(preview, resized, cv::Size(imageWidth, imageHeight), 0.0, 0.0, cv::INTER_AREA);

    const cv::Rect target(
        previewBounds.x + ((previewBounds.width - imageWidth) / 2),
        previewBounds.y + ((previewBounds.height - imageHeight) / 2),
        imageWidth,
        imageHeight);
    resized.copyTo(canvas(target));
}

void OpenCvFramePresenter::drawStreamControl(cv::Mat& canvas, const std::string& streamName, int number, int y)
{
    const bool visible = isStreamVisible(streamName);
    const cv::Rect rowBounds(12, y - 23, kSidebarWidth - 24, 30);
    const cv::Rect boxBounds(18, y - 16, 18, 18);
    streamHitBoxes_[streamName] = rowBounds;

    cv::rectangle(canvas, rowBounds, visible ? cv::Scalar(41, 48, 52) : kSidebarColor, cv::FILLED);
    cv::rectangle(canvas, boxBounds, visible ? kAccentColor : kMutedTextColor, 1, cv::LINE_AA);

    if (visible) {
        cv::rectangle(canvas, boxBounds + cv::Size(-1, -1), kAccentColor, cv::FILLED);
        cv::line(
            canvas,
            cv::Point(boxBounds.x + 4, boxBounds.y + 9),
            cv::Point(boxBounds.x + 8, boxBounds.y + 13),
            cv::Scalar(20, 22, 24),
            2,
            cv::LINE_AA);
        cv::line(
            canvas,
            cv::Point(boxBounds.x + 8, boxBounds.y + 13),
            cv::Point(boxBounds.x + 15, boxBounds.y + 5),
            cv::Scalar(20, 22, 24),
            2,
            cv::LINE_AA);
    }

    std::ostringstream label;
    if (number <= 9) {
        label << number << ". ";
    }
    label << streamName;

    putTextLine(
        canvas,
        fitText(label.str(), kSidebarWidth - 66, 0.55),
        cv::Point(46, y),
        0.55,
        visible ? kTextColor : kMutedTextColor);
}

void OpenCvFramePresenter::drawTile(cv::Mat& canvas, const cv::Rect& tileBounds, const VideoFrame& frame) const
{
    cv::rectangle(canvas, tileBounds, kPanelColor, cv::FILLED);
    cv::rectangle(canvas, tileBounds, kPanelBorderColor, 1, cv::LINE_AA);

    const cv::Rect header(tileBounds.x, tileBounds.y, tileBounds.width, kTileHeaderHeight);
    cv::rectangle(canvas, header, cv::Scalar(45, 50, 55), cv::FILLED);
    putTextLine(
        canvas,
        fitText(frame.name + "  " + frame.details, tileBounds.width - 24, 0.57),
        cv::Point(tileBounds.x + 12, tileBounds.y + 25),
        0.57);

    const cv::Rect body(
        tileBounds.x + kGap,
        tileBounds.y + kTileHeaderHeight + kGap,
        tileBounds.width - (2 * kGap),
        tileBounds.height - kTileHeaderHeight - (2 * kGap));

    if (frame.image.empty() || body.width <= 0 || body.height <= 0) {
        drawEmptyMessage(canvas, body, "No image data");
        return;
    }

    cv::Mat annotatedImage;
    const cv::Mat* sourceImage = &frame.image;
    if (objectDetectionEnabled_ && isColorStreamName(frame.name) && latestObjectMatch_) {
        annotatedImage = frame.image.clone();
        drawObjectMatchOverlay(annotatedImage);
        sourceImage = &annotatedImage;
    }

    const double scale = std::min(
        static_cast<double>(body.width) / sourceImage->cols,
        static_cast<double>(body.height) / sourceImage->rows);
    const int imageWidth = std::max(1, static_cast<int>(std::round(sourceImage->cols * scale)));
    const int imageHeight = std::max(1, static_cast<int>(std::round(sourceImage->rows * scale)));

    cv::Mat resized;
    cv::resize(*sourceImage, resized, cv::Size(imageWidth, imageHeight), 0.0, 0.0, cv::INTER_AREA);

    const cv::Rect target(
        body.x + ((body.width - imageWidth) / 2),
        body.y + ((body.height - imageHeight) / 2),
        imageWidth,
        imageHeight);
    resized.copyTo(canvas(target));
}

void OpenCvFramePresenter::drawObjectMatchOverlay(cv::Mat& image) const
{
    if (!latestObjectMatch_ || latestObjectMatch_->objectCorners.size() != 4) {
        return;
    }

    const std::vector<cv::Point> corners = {
        latestObjectMatch_->objectCorners[0],
        latestObjectMatch_->objectCorners[1],
        latestObjectMatch_->objectCorners[2],
        latestObjectMatch_->objectCorners[3],
    };

    cv::polylines(image, corners, true, cv::Scalar(0, 255, 120), 4, cv::LINE_AA);

    std::ostringstream label;
    label << "Object found  confidence " << std::fixed << std::setprecision(2)
          << latestObjectMatch_->confidence;

    int baseline = 0;
    const cv::Size textSize = cv::getTextSize(label.str(), cv::FONT_HERSHEY_SIMPLEX, 0.8, 2, &baseline);
    cv::Point origin = corners.front() + cv::Point(0, -12);
    origin.x = std::clamp(origin.x, 8, std::max(8, image.cols - textSize.width - 8));
    origin.y = std::clamp(origin.y, textSize.height + 8, std::max(textSize.height + 8, image.rows - 8));

    const cv::Rect labelBackground(
        origin.x - 6,
        origin.y - textSize.height - 6,
        textSize.width + 12,
        textSize.height + baseline + 12);
    cv::rectangle(image, labelBackground, cv::Scalar(0, 0, 0), cv::FILLED);
    cv::putText(
        image,
        label.str(),
        origin,
        cv::FONT_HERSHEY_SIMPLEX,
        0.8,
        cv::Scalar(0, 255, 120),
        2,
        cv::LINE_AA);
}

void OpenCvFramePresenter::drawMotionTile(cv::Mat& canvas, const cv::Rect& tileBounds) const
{
    cv::rectangle(canvas, tileBounds, kPanelColor, cv::FILLED);
    cv::rectangle(canvas, tileBounds, kPanelBorderColor, 1, cv::LINE_AA);

    const cv::Rect header(tileBounds.x, tileBounds.y, tileBounds.width, kTileHeaderHeight);
    cv::rectangle(canvas, header, cv::Scalar(45, 50, 55), cv::FILLED);
    putTextLine(canvas, "Motion streams", cv::Point(tileBounds.x + 12, tileBounds.y + 25), 0.57);

    int y = tileBounds.y + kTileHeaderHeight + 34;
    for (const std::string& streamName : visibleMotionStreams()) {
        const auto sample = latestMotionSamples_.find(streamName);
        if (sample == latestMotionSamples_.end()) {
            continue;
        }

        putTextLine(
            canvas,
            fitText(motionLine(sample->second), tileBounds.width - 24, 0.58),
            cv::Point(tileBounds.x + 12, y),
            0.58);
        y += 34;
    }
}

void OpenCvFramePresenter::updatePointCloudViewer()
{
    const std::vector<std::string> pointCloudStreams = visiblePointCloudStreams();
    if (pointCloudStreams.empty()) {
        shutdownPointCloudViewer();
        return;
    }

    const auto frame = latestPointCloudFrames_.find(pointCloudStreams.front());
    if (frame == latestPointCloudFrames_.end() || !frame->second.cloud || frame->second.cloud->points.empty()) {
        spinPointCloudViewer();
        return;
    }

    if (pointCloudViewer_ && pointCloudViewer_->wasStopped()) {
        pointCloudViewer_.reset();
        pointCloudViewerHasCloud_ = false;
        streamVisibility_[frame->first] = false;
        return;
    }

    if (!pointCloudViewer_) {
        pointCloudViewer_.reset(new pcl::visualization::PCLVisualizer(kPointCloudWindowName));
        pointCloudViewer_->setBackgroundColor(0.02, 0.025, 0.03);
        pointCloudViewer_->addCoordinateSystem(0.25);
        pointCloudViewer_->initCameraParameters();
        pointCloudViewer_->setCameraPosition(0.0, -1.2, -2.2, 0.0, 0.0, 1.2, 0.0, -1.0, 0.0);
        pointCloudViewer_->setShowFPS(false);
        pointCloudViewer_->addText(
            "Navigate: left-drag rotate | right-drag zoom | middle-drag pan | r reset camera",
            12,
            16,
            24,
            0.86,
            0.90,
            0.92,
            kPointCloudHelpTextId);
    }

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> colorHandler(frame->second.cloud);
    if (pointCloudViewerHasCloud_) {
        pointCloudViewer_->updatePointCloud(frame->second.cloud, colorHandler, kPointCloudId);
    } else {
        pointCloudViewer_->addPointCloud(frame->second.cloud, colorHandler, kPointCloudId);
        pointCloudViewer_->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
            2.0,
            kPointCloudId);
        pointCloudViewer_->resetCamera();
        pointCloudViewerHasCloud_ = true;
    }

    spinPointCloudViewer();
}

void OpenCvFramePresenter::shutdownPointCloudViewer()
{
    if (!pointCloudViewer_) {
        pointCloudViewerHasCloud_ = false;
        return;
    }

    if (!pointCloudViewer_->wasStopped()) {
        pointCloudViewer_->close();
    }

    pointCloudViewer_.reset();
    pointCloudViewerHasCloud_ = false;
}

void OpenCvFramePresenter::spinPointCloudViewer()
{
    if (!pointCloudViewer_) {
        return;
    }

    if (pointCloudViewer_->wasStopped()) {
        pointCloudViewer_.reset();
        pointCloudViewerHasCloud_ = false;
        for (const auto& [name, frame] : latestPointCloudFrames_) {
            (void)frame;
            streamVisibility_[name] = false;
        }
        return;
    }

    pointCloudViewer_->spinOnce(1, false);
}

std::vector<std::string> OpenCvFramePresenter::visibleVideoStreams() const
{
    std::vector<std::string> streams;
    for (const std::string& streamName : streamOrder_) {
        if (isStreamVisible(streamName) && latestVideoFrames_.find(streamName) != latestVideoFrames_.end()) {
            streams.push_back(streamName);
        }
    }

    return streams;
}

std::vector<std::string> OpenCvFramePresenter::visiblePointCloudStreams() const
{
    std::vector<std::string> streams;
    for (const std::string& streamName : streamOrder_) {
        if (isStreamVisible(streamName) && latestPointCloudFrames_.find(streamName) != latestPointCloudFrames_.end()) {
            streams.push_back(streamName);
        }
    }

    return streams;
}

std::vector<std::string> OpenCvFramePresenter::visibleMotionStreams() const
{
    std::vector<std::string> streams;
    for (const std::string& streamName : streamOrder_) {
        if (isStreamVisible(streamName) && latestMotionSamples_.find(streamName) != latestMotionSamples_.end()) {
            streams.push_back(streamName);
        }
    }

    return streams;
}

bool OpenCvFramePresenter::isStreamVisible(const std::string& name) const
{
    const auto visible = streamVisibility_.find(name);
    return visible == streamVisibility_.end() || visible->second;
}

bool OpenCvFramePresenter::keepRunning(int delayMs)
{
    const int key = cv::waitKey(delayMs);
    if (key >= 0) {
        const int normalizedKey = key & 0xff;
        if (normalizedKey == kExitKeyQ || normalizedKey == kExitKeyEsc) {
            return false;
        }

        if (normalizedKey >= '1' && normalizedKey <= '9') {
            const auto index = static_cast<size_t>(normalizedKey - '1');
            if (index < streamOrder_.size()) {
                streamVisibility_[streamOrder_[index]] = !isStreamVisible(streamOrder_[index]);
                if (latestPointCloudFrames_.find(streamOrder_[index]) != latestPointCloudFrames_.end()) {
                    updatePointCloudViewer();
                }
            }
        }
    }

    spinPointCloudViewer();

    if (!windowInitialized_) {
        return true;
    }

    try {
        return cv::getWindowProperty(kWindowName, cv::WND_PROP_VISIBLE) >= 1.0;
    } catch (const cv::Exception&) {
        return false;
    }
}

} // namespace rsv
