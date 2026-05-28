#include "unity.h"

#include "realsenseviewer/CommandLineOptions.hpp"

#include <initializer_list>
#include <sstream>
#include <string>
#include <vector>

namespace {

rsv::CommandLineParseResult parseArgs(std::initializer_list<const char*> arguments)
{
    std::vector<char*> argv;
    argv.reserve(arguments.size());
    for (const char* argument : arguments) {
        argv.push_back(const_cast<char*>(argument));
    }

    return rsv::parseCommandLine(static_cast<int>(argv.size()), argv.data());
}

} // namespace

void command_line_defaults_match_viewer_defaults()
{
    const rsv::CommandLineParseResult result = parseArgs({"viewer"});

    TEST_ASSERT_TRUE(result.error.empty());
    TEST_ASSERT_FALSE(result.options.showHelp);
    TEST_ASSERT_FALSE(result.options.listDevices);
    TEST_ASSERT_TRUE(result.options.settings.serialNumber.empty());
    TEST_ASSERT_FALSE(result.options.settings.enableMotionStreams);
    TEST_ASSERT_FALSE(result.options.settings.useAutoProfileProbe);
    TEST_ASSERT_TRUE(result.options.settings.enableInfraredStreams);
}

void command_line_help_sets_help_flag()
{
    const rsv::CommandLineParseResult longHelp = parseArgs({"viewer", "--help"});
    const rsv::CommandLineParseResult shortHelp = parseArgs({"viewer", "-h"});

    TEST_ASSERT_TRUE(longHelp.error.empty());
    TEST_ASSERT_TRUE(longHelp.options.showHelp);
    TEST_ASSERT_TRUE(shortHelp.error.empty());
    TEST_ASSERT_TRUE(shortHelp.options.showHelp);
}

void command_line_parses_all_viewer_options()
{
    const rsv::CommandLineParseResult result = parseArgs({
        "viewer",
        "--serial",
        "053122251294",
        "--motion",
        "--auto-profiles",
        "--no-infrared",
        "--list-devices",
    });

    TEST_ASSERT_TRUE(result.error.empty());
    TEST_ASSERT_TRUE(result.options.settings.serialNumber == "053122251294");
    TEST_ASSERT_TRUE(result.options.settings.enableMotionStreams);
    TEST_ASSERT_TRUE(result.options.settings.useAutoProfileProbe);
    TEST_ASSERT_FALSE(result.options.settings.enableInfraredStreams);
    TEST_ASSERT_TRUE(result.options.listDevices);
}

void command_line_last_motion_flag_wins()
{
    const rsv::CommandLineParseResult enabledThenDisabled = parseArgs({"viewer", "--motion", "--no-motion"});
    const rsv::CommandLineParseResult disabledThenEnabled = parseArgs({"viewer", "--no-motion", "--motion"});

    TEST_ASSERT_TRUE(enabledThenDisabled.error.empty());
    TEST_ASSERT_FALSE(enabledThenDisabled.options.settings.enableMotionStreams);
    TEST_ASSERT_TRUE(disabledThenEnabled.error.empty());
    TEST_ASSERT_TRUE(disabledThenEnabled.options.settings.enableMotionStreams);
}

void command_line_rejects_missing_serial_value()
{
    const rsv::CommandLineParseResult result = parseArgs({"viewer", "--serial"});

    TEST_ASSERT_TRUE(result.error == "--serial requires a camera serial number");
}

void command_line_rejects_unknown_argument()
{
    const rsv::CommandLineParseResult result = parseArgs({"viewer", "--banana"});

    TEST_ASSERT_TRUE(result.error == "Unknown argument: --banana");
}

void command_line_usage_mentions_supported_options()
{
    std::ostringstream usage;
    rsv::printUsage(usage, "viewer");
    const std::string text = usage.str();

    TEST_ASSERT_TRUE(text.find("Usage: viewer") != std::string::npos);
    TEST_ASSERT_TRUE(text.find("--serial CAMERA_SERIAL") != std::string::npos);
    TEST_ASSERT_TRUE(text.find("--motion") != std::string::npos);
    TEST_ASSERT_TRUE(text.find("--auto-profiles") != std::string::npos);
    TEST_ASSERT_TRUE(text.find("--no-infrared") != std::string::npos);
    TEST_ASSERT_TRUE(text.find("--list-devices") != std::string::npos);
    TEST_ASSERT_TRUE(text.find("--no-motion") != std::string::npos);
}
