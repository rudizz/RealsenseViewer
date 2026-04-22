#include "realsenseviewer/Application.hpp"
#include "realsenseviewer/camera/RealSenseDiagnostics.hpp"
#include "realsenseviewer/camera/RealSenseFrameSource.hpp"
#include "realsenseviewer/display/OpenCvFramePresenter.hpp"

#include <iostream>
#include <memory>
#include <string>
#include <utility>

namespace {

void printUsage(const char* executableName)
{
    std::cout << "Usage: " << executableName
              << " [--serial CAMERA_SERIAL] [--motion] [--auto-profiles]\n\n"
              << "Options:\n"
              << "  --serial CAMERA_SERIAL  Open a specific camera\n"
              << "  --motion                Also enable accel/gyro streams\n"
              << "  --auto-profiles         Probe the SDK for every exposed stream profile\n"
              << "  --list-devices          Print devices visible to librealsense and exit\n"
              << "  --no-motion             Keep IMU streams disabled\n\n"
              << "Keys while running:\n"
              << "  q or Esc    Quit the viewer\n"
              << "  1-9         Toggle stream visibility\n"
              << "  mouse       Click dashboard controls\n";
}

} // namespace

int main(int argc, char** argv)
{
    rsv::RealSenseSettings settings;
    bool listDevices = false;

    for (int i = 1; i < argc; ++i) {
         std::string argument = argv[i];

        if (argument == "--help" || argument == "-h") {
            printUsage(argv[0]);
            return 0;
        }

        if (argument == "--serial") {
            if (i + 1 >= argc) {
                std::cerr << "--serial requires a camera serial number\n";
                return 2;
            }

            settings.serialNumber = argv[++i];
            continue;
        }

        if (argument == "--motion") {
            settings.enableMotionStreams = true;
            continue;
        }

        if (argument == "--no-motion") {
            settings.enableMotionStreams = false;
            continue;
        }

        if (argument == "--auto-profiles") {
            settings.useAutoProfileProbe = true;
            continue;
        }

        if (argument == "--list-devices") {
            listDevices = true;
            continue;
        }

        std::cerr << "Unknown argument: " << argument << "\n";
        printUsage(argv[0]);
        return 2;
    }

    if (listDevices) {
        return rsv::printRealSenseDevices(std::cout);
    }

    auto source = std::make_unique<rsv::RealSenseFrameSource>(settings);
    auto presenter = std::make_unique<rsv::OpenCvFramePresenter>();

    rsv::Application app(std::move(source), std::move(presenter));
    return app.run();
}
