#pragma once

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include <optional>
#include <string>
#include <vector>

namespace rsv::features {

enum class FeatureDetectorType {
    Sift,
    Orb,
    Surf,
};

enum class FeatureMatcherType {
    BruteForce,
    Flann,
};

struct MatchResult {
    std::vector<cv::Point2f> objectCorners;
    int goodMatches = 0;
    int inliers = 0;
    double confidence = 0.0;
};

class ObjectFeatureMatcher {
public:
    [[nodiscard]] bool setDetectorType(FeatureDetectorType detectorType);
    [[nodiscard]] bool setCalibrationResampleScale(double scale);
    void setMatcherType(FeatureMatcherType matcherType);
    [[nodiscard]] bool loadCalibrationImage(const std::string& imagePath);
    [[nodiscard]] std::optional<MatchResult> match(const cv::Mat& colorImage) const;

    [[nodiscard]] FeatureDetectorType detectorType() const;
    [[nodiscard]] FeatureMatcherType matcherType() const;
    [[nodiscard]] double calibrationResampleScale() const;
    [[nodiscard]] const char* detectorName() const;
    [[nodiscard]] const char* matcherName() const;
    [[nodiscard]] bool hasCalibration() const;
    [[nodiscard]] const std::string& calibrationImagePath() const;
    [[nodiscard]] const cv::Mat& calibrationFeatureImage() const;
    [[nodiscard]] int calibrationKeypointCount() const;

private:
    bool configureDetector(FeatureDetectorType detectorType);
    void clearCalibration();
    void matchDescriptors(const cv::Mat& sceneDescriptors, std::vector<std::vector<cv::DMatch>>& knnMatches) const;
    [[nodiscard]] int descriptorMatcherNorm() const;

    std::string calibrationImagePath_;
    cv::Size calibrationImageSize_;
    std::vector<cv::KeyPoint> calibrationKeypoints_;
    cv::Mat calibrationDescriptors_;
    cv::Mat calibrationFeatureImage_;
    cv::Ptr<cv::Feature2D> detector_ = cv::SIFT::create(1600);
    FeatureDetectorType detectorType_ = FeatureDetectorType::Sift;
    FeatureMatcherType matcherType_ = FeatureMatcherType::Flann;
    double calibrationResampleScale_ = 1.0;
};

} // namespace rsv::features
