#include "unity.h"

#include "realsenseviewer/camera/PointCloudSettings.hpp"

void point_cloud_pixel_step_bounds_are_reported()
{
    TEST_ASSERT_EQUAL_INT(1, rsv::minimumPointCloudPixelStep());
    TEST_ASSERT_EQUAL_INT(12, rsv::maximumPointCloudPixelStep());
}

void point_cloud_pixel_step_is_clamped_to_supported_range()
{
    rsv::setPointCloudPixelStep(5);
    TEST_ASSERT_EQUAL_INT(5, rsv::pointCloudPixelStep());

    rsv::setPointCloudPixelStep(-10);
    TEST_ASSERT_EQUAL_INT(rsv::minimumPointCloudPixelStep(), rsv::pointCloudPixelStep());

    rsv::setPointCloudPixelStep(100);
    TEST_ASSERT_EQUAL_INT(rsv::maximumPointCloudPixelStep(), rsv::pointCloudPixelStep());
}

void point_cloud_conversion_flag_can_be_toggled()
{
    TEST_ASSERT_FALSE(rsv::pointCloudConversionEnabled());

    rsv::setPointCloudConversionEnabled(true);
    TEST_ASSERT_TRUE(rsv::pointCloudConversionEnabled());

    rsv::setPointCloudConversionEnabled(false);
    TEST_ASSERT_FALSE(rsv::pointCloudConversionEnabled());
}
