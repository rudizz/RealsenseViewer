#include "realsenseviewer/Application.hpp"
#include "realsenseviewer/CommandLineOptions.hpp"
#include "realsenseviewer/camera/RealSenseDiagnostics.hpp"
#include "realsenseviewer/camera/RealSenseFrameSource.hpp"
#include "realsenseviewer/display/OpenCvFramePresenter.hpp"

#include <iostream>
#include <memory>
#include <utility>

int main(int argc, char** argv)
{
    const rsv::CommandLineParseResult commandLine = rsv::parseCommandLine(argc, argv);
    if (commandLine.options.showHelp) {
        rsv::printUsage(std::cout, argv[0]);
        return 0;
    }

    if (!commandLine.error.empty()) {
        std::cerr << commandLine.error << "\n";
        rsv::printUsage(std::cerr, argv[0]);
        return 2;
    }

    if (commandLine.options.listDevices) {
        return rsv::printRealSenseDevices(std::cout);
    }

    auto source = std::make_unique<rsv::RealSenseFrameSource>(commandLine.options.settings);
    auto presenter = std::make_unique<rsv::OpenCvFramePresenter>();

    rsv::Application app(std::move(source), std::move(presenter));
    return app.run();
}
