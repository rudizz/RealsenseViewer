#include "realsenseviewer/features/ObjectFeatureMatcher.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/flann.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>

#include <algorithm>
#include <cmath>

namespace rsv::features {
namespace {

constexpr int kMinimumCalibrationKeypoints = 30;
constexpr int kMinimumSceneKeypoints = 30;
constexpr int kMinimumGoodMatches = 10;
constexpr int kMinimumInliers = 10;
constexpr double kMinimumConfidence = 0.15;
constexpr double kLoweRatio = 0.74;
constexpr double kRansacReprojectionThreshold = 2.0;
constexpr double kMinimumProjectedArea = 100.0;

cv::Mat toGray(const cv::Mat& image)
{
    if (image.empty()) {
        return {};
    }

    if (image.channels() == 1) {
        return image;
    }

    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    return gray;
}

bool isReasonableProjection(const std::vector<cv::Point2f>& corners)
{
    if (corners.size() != 4 || !cv::isContourConvex(corners)) {
        return false;
    }

    return std::abs(cv::contourArea(corners)) >= kMinimumProjectedArea;
}

} // namespace

bool ObjectFeatureMatcher::setDetectorType(FeatureDetectorType detectorType)
{
    if (detectorType_ == detectorType) {
        return true;
    }

    const std::string previousCalibrationImagePath = calibrationImagePath_;
    configureDetector(detectorType);
    if (previousCalibrationImagePath.empty()) {
        clearCalibration();
        return true;
    }

    return loadCalibrationImage(previousCalibrationImagePath);
}

void ObjectFeatureMatcher::setMatcherType(FeatureMatcherType matcherType)
{
    matcherType_ = matcherType;
}

bool ObjectFeatureMatcher::setCalibrationResampleScale(double scale)
{
    if (calibrationResampleScale_ == scale) {
        return true;
    }

    const std::string previousCalibrationImagePath = calibrationImagePath_;
    calibrationResampleScale_ = scale;
    if (previousCalibrationImagePath.empty()) {
        clearCalibration();
        return true;
    }

    return loadCalibrationImage(previousCalibrationImagePath);
}

bool ObjectFeatureMatcher::loadCalibrationImage(const std::string& imagePath)
{
    clearCalibration();

    cv::Mat image = cv::imread(imagePath, cv::IMREAD_COLOR);
    if (image.empty()) {
        return false;
    }

    if (calibrationResampleScale_ != 1.0) {
        const cv::Size targetSize(
            std::max(1, static_cast<int>(std::round(image.cols * calibrationResampleScale_))),
            std::max(1, static_cast<int>(std::round(image.rows * calibrationResampleScale_))));
        cv::resize(image, image, targetSize, 0.0, 0.0, cv::INTER_LINEAR);
    }

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    detector_->detectAndCompute(toGray(image), cv::noArray(), keypoints, descriptors);

    if (descriptors.empty() || static_cast<int>(keypoints.size()) < kMinimumCalibrationKeypoints) {
        return false;
    }

    calibrationImagePath_ = imagePath;
    calibrationImageSize_ = image.size();
    calibrationKeypoints_ = std::move(keypoints);
    calibrationDescriptors_ = std::move(descriptors);
    cv::drawKeypoints(
        image,
        calibrationKeypoints_,
        calibrationFeatureImage_,
        cv::Scalar(0, 255, 120),
        cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    return true;
}

std::optional<MatchResult> ObjectFeatureMatcher::match(const cv::Mat& colorImage) const
{
    if (!hasCalibration() || colorImage.empty()) {
        return std::nullopt;
    }

    std::vector<cv::KeyPoint> sceneKeypoints;
    cv::Mat sceneDescriptors;
    detector_->detectAndCompute(toGray(colorImage), cv::noArray(), sceneKeypoints, sceneDescriptors);

    if (sceneDescriptors.empty() || static_cast<int>(sceneKeypoints.size()) < kMinimumSceneKeypoints) {
        return std::nullopt;
    }

    std::vector<std::vector<cv::DMatch>> knnMatches;
    matchDescriptors(sceneDescriptors, knnMatches);

    std::vector<cv::DMatch> goodMatches;
    goodMatches.reserve(knnMatches.size());
    for (const auto& pair : knnMatches) {
        if (pair.size() < 2) {
            continue;
        }

        if (pair[0].distance < kLoweRatio * pair[1].distance) {
            goodMatches.push_back(pair[0]);
        }
    }

    if (static_cast<int>(goodMatches.size()) < kMinimumGoodMatches) {
        return std::nullopt;
    }

    std::vector<cv::Point2f> calibrationPoints;
    std::vector<cv::Point2f> scenePoints;
    calibrationPoints.reserve(goodMatches.size());
    scenePoints.reserve(goodMatches.size());

    for (const cv::DMatch& match : goodMatches) {
        calibrationPoints.push_back(calibrationKeypoints_[match.queryIdx].pt);
        scenePoints.push_back(sceneKeypoints[match.trainIdx].pt);
    }

    cv::Mat inlierMask;
    const cv::Mat homography = cv::findHomography(
        calibrationPoints,
        scenePoints,
        cv::RANSAC,
        kRansacReprojectionThreshold,
        inlierMask);
    if (homography.empty()) {
        return std::nullopt;
    }

    const int inliers = cv::countNonZero(inlierMask);
    const double confidence = static_cast<double>(inliers) / static_cast<double>(goodMatches.size());
    if (inliers < kMinimumInliers || confidence < kMinimumConfidence) {
        return std::nullopt;
    }

    std::vector<cv::Point2f> calibrationCorners = {
        cv::Point2f(0.0F, 0.0F),
        cv::Point2f(static_cast<float>(calibrationImageSize_.width), 0.0F),
        cv::Point2f(static_cast<float>(calibrationImageSize_.width), static_cast<float>(calibrationImageSize_.height)),
        cv::Point2f(0.0F, static_cast<float>(calibrationImageSize_.height)),
    };
    std::vector<cv::Point2f> projectedCorners;
    cv::perspectiveTransform(calibrationCorners, projectedCorners, homography);

    if (!isReasonableProjection(projectedCorners)) {
        return std::nullopt;
    }

    return MatchResult {
        std::move(projectedCorners),
        static_cast<int>(goodMatches.size()),
        inliers,
        confidence,
    };
}

FeatureDetectorType ObjectFeatureMatcher::detectorType() const
{
    return detectorType_;
}

FeatureMatcherType ObjectFeatureMatcher::matcherType() const
{
    return matcherType_;
}

double ObjectFeatureMatcher::calibrationResampleScale() const
{
    return calibrationResampleScale_;
}

const char* ObjectFeatureMatcher::detectorName() const
{
    switch (detectorType_) {
    case FeatureDetectorType::Sift:
        return "SIFT";
    case FeatureDetectorType::Orb:
        return "ORB";
    case FeatureDetectorType::Surf:
        return "SURF";
    }

    return "Unknown";
}

const char* ObjectFeatureMatcher::matcherName() const
{
    switch (matcherType_) {
    case FeatureMatcherType::BruteForce:
        return "Brute Force";
    case FeatureMatcherType::Flann:
        return "FLANN";
    }

    return "Unknown";
}

bool ObjectFeatureMatcher::hasCalibration() const
{
    return !calibrationDescriptors_.empty();
}

const std::string& ObjectFeatureMatcher::calibrationImagePath() const
{
    return calibrationImagePath_;
}

const cv::Mat& ObjectFeatureMatcher::calibrationFeatureImage() const
{
    return calibrationFeatureImage_;
}

int ObjectFeatureMatcher::calibrationKeypointCount() const
{
    return static_cast<int>(calibrationKeypoints_.size());
}

void ObjectFeatureMatcher::configureDetector(FeatureDetectorType detectorType)
{
    detectorType_ = detectorType;
    switch (detectorType_) {
    case FeatureDetectorType::Sift:
        detector_ = cv::SIFT::create(1600);
        break;
    case FeatureDetectorType::Orb:
        detector_ = cv::ORB::create(1600);
        break;
    case FeatureDetectorType::Surf:
        detector_ = cv::xfeatures2d::SURF::create(400.0);
        break;
    }
}

void ObjectFeatureMatcher::clearCalibration()
{
    calibrationImagePath_.clear();
    calibrationImageSize_ = {};
    calibrationKeypoints_.clear();
    calibrationDescriptors_.release();
    calibrationFeatureImage_.release();
}

void ObjectFeatureMatcher::matchDescriptors(
    const cv::Mat& sceneDescriptors,
    std::vector<std::vector<cv::DMatch>>& knnMatches) const
{
    switch (matcherType_) {
    case FeatureMatcherType::BruteForce: {
        cv::BFMatcher matcher(descriptorMatcherNorm());
        matcher.knnMatch(calibrationDescriptors_, sceneDescriptors, knnMatches, 2);
        break;
    }
    case FeatureMatcherType::Flann: {
        if (calibrationDescriptors_.type() == CV_8U && sceneDescriptors.type() == CV_8U) {
            cv::FlannBasedMatcher matcher(
                cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2),
                cv::makePtr<cv::flann::SearchParams>());
            matcher.knnMatch(calibrationDescriptors_, sceneDescriptors, knnMatches, 2);
            break;
        }

        cv::FlannBasedMatcher matcher;
        matcher.knnMatch(calibrationDescriptors_, sceneDescriptors, knnMatches, 2);
        break;
    }
    }
}

int ObjectFeatureMatcher::descriptorMatcherNorm() const
{
    switch (detectorType_) {
    case FeatureDetectorType::Sift:
    case FeatureDetectorType::Surf:
        return cv::NORM_L2;
    case FeatureDetectorType::Orb:
        return cv::NORM_HAMMING;
    }

    return cv::NORM_L2;
}

} // namespace rsv::features
