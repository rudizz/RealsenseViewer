#pragma once

#include <iosfwd>
#include <string>
#include <vector>

namespace rsv {

struct RealSenseDeviceInfo {
    std::string name;
    std::string serialNumber;
    std::string firmwareVersion;
    std::string physicalPort;
};

[[nodiscard]] std::vector<RealSenseDeviceInfo> queryRealSenseDevices();
int printRealSenseDevices(std::ostream& output);
[[nodiscard]] std::string describeRealSenseDevices();

} // namespace rsv
