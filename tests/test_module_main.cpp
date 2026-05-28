#include "unity.h"

#include "realsenseviewer/camera/PointCloudSettings.hpp"

#include <opencv2/core/utils/logger.hpp>

void application_presents_polled_frames_until_presenter_stops();
void application_idles_when_no_frame_is_available();
void application_continues_after_idle_until_a_frame_is_presented();
void application_stops_source_and_returns_failure_when_start_throws();
void application_stops_source_and_returns_failure_when_poll_throws();
void application_stops_source_and_returns_failure_when_presenter_throws();

void command_line_defaults_match_viewer_defaults();
void command_line_help_sets_help_flag();
void command_line_parses_all_viewer_options();
void command_line_last_motion_flag_wins();
void command_line_rejects_missing_serial_value();
void command_line_rejects_unknown_argument();
void command_line_usage_mentions_supported_options();

void frame_bundle_reports_empty_when_it_has_no_frames();
void frame_bundle_reports_non_empty_for_each_frame_family();

void object_feature_matcher_defaults_are_exposed();
void object_feature_matcher_rejects_missing_or_blank_calibration();
void object_feature_matcher_loads_feature_rich_calibration();
void object_feature_matcher_resample_scale_reloads_calibration_image();
void object_feature_matcher_can_switch_detector_and_matcher_types();
void object_feature_matcher_matches_identical_scene();
void object_feature_matcher_returns_no_match_without_calibration_or_on_blank_scene();

void point_cloud_pixel_step_bounds_are_reported();
void point_cloud_pixel_step_is_clamped_to_supported_range();
void point_cloud_conversion_flag_can_be_toggled();

void setUp()
{
    rsv::setPointCloudPixelStep(2);
    rsv::setPointCloudConversionEnabled(false);
}

void tearDown()
{
}

int main()
{
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_ERROR);
    UNITY_BEGIN();

    RUN_TEST(application_presents_polled_frames_until_presenter_stops);
    RUN_TEST(application_idles_when_no_frame_is_available);
    RUN_TEST(application_continues_after_idle_until_a_frame_is_presented);
    RUN_TEST(application_stops_source_and_returns_failure_when_start_throws);
    RUN_TEST(application_stops_source_and_returns_failure_when_poll_throws);
    RUN_TEST(application_stops_source_and_returns_failure_when_presenter_throws);

    RUN_TEST(command_line_defaults_match_viewer_defaults);
    RUN_TEST(command_line_help_sets_help_flag);
    RUN_TEST(command_line_parses_all_viewer_options);
    RUN_TEST(command_line_last_motion_flag_wins);
    RUN_TEST(command_line_rejects_missing_serial_value);
    RUN_TEST(command_line_rejects_unknown_argument);
    RUN_TEST(command_line_usage_mentions_supported_options);

    RUN_TEST(frame_bundle_reports_empty_when_it_has_no_frames);
    RUN_TEST(frame_bundle_reports_non_empty_for_each_frame_family);

    RUN_TEST(object_feature_matcher_defaults_are_exposed);
    RUN_TEST(object_feature_matcher_rejects_missing_or_blank_calibration);
    RUN_TEST(object_feature_matcher_loads_feature_rich_calibration);
    RUN_TEST(object_feature_matcher_resample_scale_reloads_calibration_image);
    RUN_TEST(object_feature_matcher_can_switch_detector_and_matcher_types);
    RUN_TEST(object_feature_matcher_matches_identical_scene);
    RUN_TEST(object_feature_matcher_returns_no_match_without_calibration_or_on_blank_scene);

    RUN_TEST(point_cloud_pixel_step_bounds_are_reported);
    RUN_TEST(point_cloud_pixel_step_is_clamped_to_supported_range);
    RUN_TEST(point_cloud_conversion_flag_can_be_toggled);

    return UNITY_END();
}
