#include "realsenseviewer/camera/RealSenseDiagnostics.hpp"

#include <librealsense2/rs.hpp>

#include <iostream>
#include <sstream>

namespace rsv {
namespace {

std::string deviceInfo(const rs2::device& device, rs2_camera_info field)
{
    if (!device.supports(field)) {
        return {};
    }

    return device.get_info(field);
}

} // namespace

std::vector<RealSenseDeviceInfo> queryRealSenseDevices()
{
    rs2::context context;
    const rs2::device_list devices = context.query_devices();

    std::vector<RealSenseDeviceInfo> result;
    result.reserve(devices.size());

    for (const rs2::device& device : devices) {
        result.push_back(RealSenseDeviceInfo {
            deviceInfo(device, RS2_CAMERA_INFO_NAME),
            deviceInfo(device, RS2_CAMERA_INFO_SERIAL_NUMBER),
            deviceInfo(device, RS2_CAMERA_INFO_FIRMWARE_VERSION),
            deviceInfo(device, RS2_CAMERA_INFO_PHYSICAL_PORT),
        });
    }

    return result;
}

int printRealSenseDevices(std::ostream& output)
{
    const std::vector<RealSenseDeviceInfo> devices = queryRealSenseDevices();

    if (devices.empty()) {
        output << "librealsense reports zero connected RealSense devices.\n"
               << "If macOS System Information still lists the camera, the SDK cannot claim/access it.\n";
        return 1;
    }

    output << "librealsense devices:\n";
    for (const RealSenseDeviceInfo& device : devices) {
        output << "  Name: " << (device.name.empty() ? "(unknown)" : device.name) << "\n"
               << "  Serial: " << (device.serialNumber.empty() ? "(unknown)" : device.serialNumber) << "\n"
               << "  Firmware: " << (device.firmwareVersion.empty() ? "(unknown)" : device.firmwareVersion) << "\n"
               << "  Port: " << (device.physicalPort.empty() ? "(unknown)" : device.physicalPort) << "\n\n";
    }

    return 0;
}

std::string describeRealSenseDevices()
{
    std::ostringstream description;
    const std::vector<RealSenseDeviceInfo> devices = queryRealSenseDevices();

    if (devices.empty()) {
        return "librealsense currently reports zero connected RealSense devices";
    }

    description << "librealsense sees " << devices.size() << " RealSense device(s):";
    for (const RealSenseDeviceInfo& device : devices) {
        description << " [" << (device.name.empty() ? "unknown" : device.name)
                    << ", serial " << (device.serialNumber.empty() ? "unknown" : device.serialNumber)
                    << "]";
    }

    return description.str();
}

} // namespace rsv
