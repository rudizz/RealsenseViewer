#include "unity.h"

#include "realsenseviewer/features/ObjectFeatureMatcher.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <filesystem>
#include <optional>
#include <string>
#include <system_error>

namespace {

std::filesystem::path testImagePath(const std::string& name)
{
    return std::filesystem::temp_directory_path() / name;
}

void removeFile(const std::filesystem::path& path)
{
    std::error_code error;
    std::filesystem::remove(path, error);
}

cv::Mat makeFeatureRichImage()
{
    cv::Mat image(320, 320, CV_8UC3);
    cv::RNG rng(1337);
    rng.fill(image, cv::RNG::UNIFORM, 0, 255);

    for (int i = 0; i < 40; ++i) {
        const cv::Point center(rng.uniform(20, 300), rng.uniform(20, 300));
        const int radius = rng.uniform(5, 24);
        const cv::Scalar color(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        cv::circle(image, center, radius, color, 2, cv::LINE_AA);
    }

    for (int i = 0; i < 20; ++i) {
        const cv::Point start(rng.uniform(0, 320), rng.uniform(0, 320));
        const cv::Point end(rng.uniform(0, 320), rng.uniform(0, 320));
        const cv::Scalar color(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        cv::line(image, start, end, color, 2, cv::LINE_AA);
    }

    cv::putText(image, "RSV", cv::Point(78, 178), cv::FONT_HERSHEY_SIMPLEX, 2.4, cv::Scalar(255, 255, 255), 5);
    cv::putText(image, "TEST", cv::Point(90, 235), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 0), 3);
    return image;
}

std::string writeFeatureRichImage()
{
    const std::filesystem::path path = testImagePath("realsenseviewer_feature_calibration.png");
    removeFile(path);
    TEST_ASSERT_TRUE(cv::imwrite(path.string(), makeFeatureRichImage()));
    return path.string();
}

std::string writeBlankImage()
{
    const std::filesystem::path path = testImagePath("realsenseviewer_blank_calibration.png");
    removeFile(path);
    TEST_ASSERT_TRUE(cv::imwrite(path.string(), cv::Mat::zeros(200, 200, CV_8UC3)));
    return path.string();
}

} // namespace

void object_feature_matcher_defaults_are_exposed()
{
    const rsv::features::ObjectFeatureMatcher matcher;

    TEST_ASSERT_TRUE(matcher.detectorType() == rsv::features::FeatureDetectorType::Sift);
    TEST_ASSERT_TRUE(matcher.matcherType() == rsv::features::FeatureMatcherType::Flann);
    TEST_ASSERT_TRUE(matcher.calibrationResampleScale() == 1.0);
    TEST_ASSERT_TRUE(std::string(matcher.detectorName()) == "SIFT");
    TEST_ASSERT_TRUE(std::string(matcher.matcherName()) == "FLANN");
    TEST_ASSERT_FALSE(matcher.hasCalibration());
    TEST_ASSERT_TRUE(matcher.calibrationImagePath().empty());
    TEST_ASSERT_TRUE(matcher.calibrationFeatureImage().empty());
    TEST_ASSERT_EQUAL_INT(0, matcher.calibrationKeypointCount());
}

void object_feature_matcher_rejects_missing_or_blank_calibration()
{
    rsv::features::ObjectFeatureMatcher matcher;
    const std::string blankPath = writeBlankImage();

    TEST_ASSERT_FALSE(matcher.loadCalibrationImage("/definitely/not/a/real/image.png"));
    TEST_ASSERT_FALSE(matcher.hasCalibration());

    TEST_ASSERT_FALSE(matcher.loadCalibrationImage(blankPath));
    TEST_ASSERT_FALSE(matcher.hasCalibration());
    TEST_ASSERT_TRUE(matcher.calibrationImagePath().empty());

    removeFile(blankPath);
}

void object_feature_matcher_loads_feature_rich_calibration()
{
    rsv::features::ObjectFeatureMatcher matcher;
    const std::string path = writeFeatureRichImage();

    TEST_ASSERT_TRUE(matcher.loadCalibrationImage(path));
    TEST_ASSERT_TRUE(matcher.hasCalibration());
    TEST_ASSERT_TRUE(matcher.calibrationImagePath() == path);
    TEST_ASSERT_TRUE(matcher.calibrationKeypointCount() >= 30);
    TEST_ASSERT_FALSE(matcher.calibrationFeatureImage().empty());
    TEST_ASSERT_EQUAL_INT(320, matcher.calibrationFeatureImage().cols);
    TEST_ASSERT_EQUAL_INT(320, matcher.calibrationFeatureImage().rows);

    removeFile(path);
}

void object_feature_matcher_resample_scale_reloads_calibration_image()
{
    rsv::features::ObjectFeatureMatcher matcher;
    const std::string path = writeFeatureRichImage();

    TEST_ASSERT_TRUE(matcher.loadCalibrationImage(path));
    TEST_ASSERT_TRUE(matcher.setCalibrationResampleScale(0.5));

    TEST_ASSERT_TRUE(matcher.calibrationResampleScale() == 0.5);
    TEST_ASSERT_TRUE(matcher.hasCalibration());
    TEST_ASSERT_TRUE(matcher.calibrationImagePath() == path);
    TEST_ASSERT_TRUE(matcher.calibrationKeypointCount() >= 30);
    TEST_ASSERT_EQUAL_INT(160, matcher.calibrationFeatureImage().cols);
    TEST_ASSERT_EQUAL_INT(160, matcher.calibrationFeatureImage().rows);

    removeFile(path);
}

void object_feature_matcher_can_switch_detector_and_matcher_types()
{
    rsv::features::ObjectFeatureMatcher matcher;

    matcher.setMatcherType(rsv::features::FeatureMatcherType::BruteForce);
    TEST_ASSERT_TRUE(matcher.matcherType() == rsv::features::FeatureMatcherType::BruteForce);
    TEST_ASSERT_TRUE(std::string(matcher.matcherName()) == "Brute Force");

    TEST_ASSERT_TRUE(matcher.setDetectorType(rsv::features::FeatureDetectorType::Orb));
    TEST_ASSERT_TRUE(matcher.detectorType() == rsv::features::FeatureDetectorType::Orb);
    TEST_ASSERT_TRUE(std::string(matcher.detectorName()) == "ORB");
    TEST_ASSERT_FALSE(matcher.hasCalibration());
}

void object_feature_matcher_matches_identical_scene()
{
    rsv::features::ObjectFeatureMatcher matcher;
    const std::string path = writeFeatureRichImage();
    const cv::Mat scene = cv::imread(path, cv::IMREAD_COLOR);

    TEST_ASSERT_FALSE(scene.empty());
    TEST_ASSERT_TRUE(matcher.loadCalibrationImage(path));

    const std::optional<rsv::features::MatchResult> result = matcher.match(scene);

    TEST_ASSERT_TRUE(result.has_value());
    TEST_ASSERT_EQUAL_size_t(4, result->objectCorners.size());
    TEST_ASSERT_TRUE(result->goodMatches >= 10);
    TEST_ASSERT_TRUE(result->inliers >= 10);
    TEST_ASSERT_TRUE(result->confidence >= 0.15);

    removeFile(path);
}

void object_feature_matcher_returns_no_match_without_calibration_or_on_blank_scene()
{
    rsv::features::ObjectFeatureMatcher matcher;
    const std::string path = writeFeatureRichImage();

    TEST_ASSERT_FALSE(matcher.match(makeFeatureRichImage()).has_value());

    TEST_ASSERT_TRUE(matcher.loadCalibrationImage(path));
    TEST_ASSERT_FALSE(matcher.match(cv::Mat::zeros(320, 320, CV_8UC3)).has_value());

    removeFile(path);
}
