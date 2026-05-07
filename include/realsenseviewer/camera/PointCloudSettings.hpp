#pragma once

namespace rsv {

[[nodiscard]] int pointCloudPixelStep();
[[nodiscard]] int minimumPointCloudPixelStep();
[[nodiscard]] int maximumPointCloudPixelStep();
void setPointCloudPixelStep(int pixelStep);
[[nodiscard]] bool pointCloudConversionEnabled();
void setPointCloudConversionEnabled(bool enabled);

} // namespace rsv
