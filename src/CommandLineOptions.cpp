#include "realsenseviewer/CommandLineOptions.hpp"

#include <ostream>

namespace rsv {

void printUsage(std::ostream& output, const char* executableName)
{
    output << "Usage: " << executableName
           << " [--serial CAMERA_SERIAL] [--motion] [--auto-profiles] [--no-infrared]\n\n"
           << "Options:\n"
           << "  --serial CAMERA_SERIAL  Open a specific camera\n"
           << "  --motion                Also enable accel/gyro streams\n"
           << "  --auto-profiles         Probe the SDK for every exposed stream profile\n"
           << "  --no-infrared           Skip infrared streams\n"
           << "  --list-devices          Print devices visible to librealsense and exit\n"
           << "  --no-motion             Keep IMU streams disabled\n\n"
           << "Keys while running:\n"
           << "  q or Esc    Quit the viewer\n"
           << "  1-9         Toggle stream visibility\n"
           << "  mouse       Click dashboard controls\n";
}

CommandLineParseResult parseCommandLine(int argc, char** argv)
{
    CommandLineParseResult result;

    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];

        if (argument == "--help" || argument == "-h") {
            result.options.showHelp = true;
            return result;
        }

        if (argument == "--serial") {
            if (i + 1 >= argc) {
                result.error = "--serial requires a camera serial number";
                return result;
            }

            result.options.settings.serialNumber = argv[++i];
            continue;
        }

        if (argument == "--motion") {
            result.options.settings.enableMotionStreams = true;
            continue;
        }

        if (argument == "--no-motion") {
            result.options.settings.enableMotionStreams = false;
            continue;
        }

        if (argument == "--auto-profiles") {
            result.options.settings.useAutoProfileProbe = true;
            continue;
        }

        if (argument == "--no-infrared") {
            result.options.settings.enableInfraredStreams = false;
            continue;
        }

        if (argument == "--list-devices") {
            result.options.listDevices = true;
            continue;
        }

        result.error = "Unknown argument: " + argument;
        return result;
    }

    return result;
}

} // namespace rsv
