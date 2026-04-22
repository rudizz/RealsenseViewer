#pragma once

#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>
#include <vector>

namespace rsv {

struct VideoFrame {
    std::string name;
    std::string details;
    cv::Mat image;
    double timestampMs = 0.0;
};

struct MotionSample {
    std::string name;
    std::string units;
    cv::Vec3f value = {0.0F, 0.0F, 0.0F};
    double timestampMs = 0.0;
};

struct PointCloudFrame {
    std::string name;
    std::string details;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    double timestampMs = 0.0;
};

struct FrameBundle {
    std::vector<VideoFrame> videoFrames;
    std::vector<PointCloudFrame> pointCloudFrames;
    std::vector<MotionSample> motionSamples;

    [[nodiscard]] bool empty() const
    {
        return videoFrames.empty() && pointCloudFrames.empty() && motionSamples.empty();
    }
};

} // namespace rsv
