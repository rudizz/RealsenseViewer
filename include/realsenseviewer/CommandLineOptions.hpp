#pragma once

#include "realsenseviewer/camera/RealSenseFrameSource.hpp"

#include <iosfwd>
#include <string>

namespace rsv {

struct CommandLineOptions {
    RealSenseSettings settings;
    bool listDevices = false;
    bool showHelp = false;
};

struct CommandLineParseResult {
    CommandLineOptions options;
    std::string error;
};

[[nodiscard]] CommandLineParseResult parseCommandLine(int argc, char** argv);
void printUsage(std::ostream& output, const char* executableName);

} // namespace rsv
