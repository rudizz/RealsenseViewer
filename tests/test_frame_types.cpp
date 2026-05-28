#include "unity.h"

#include "realsenseviewer/camera/FrameTypes.hpp"

void frame_bundle_reports_empty_when_it_has_no_frames()
{
    const rsv::FrameBundle bundle;

    TEST_ASSERT_TRUE(bundle.empty());
}

void frame_bundle_reports_non_empty_for_each_frame_family()
{
    rsv::FrameBundle videoBundle;
    videoBundle.videoFrames.push_back(rsv::VideoFrame {});
    TEST_ASSERT_FALSE(videoBundle.empty());

    rsv::FrameBundle pointCloudBundle;
    pointCloudBundle.pointCloudFrames.push_back(rsv::PointCloudFrame {
        "Point Cloud",
        "0 points",
        pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(),
        0.0,
    });
    TEST_ASSERT_FALSE(pointCloudBundle.empty());

    rsv::FrameBundle motionBundle;
    motionBundle.motionSamples.push_back(rsv::MotionSample {});
    TEST_ASSERT_FALSE(motionBundle.empty());
}
