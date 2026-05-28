#include "unity.h"

#include "realsenseviewer/Application.hpp"
#include "realsenseviewer/camera/IFrameSource.hpp"
#include "realsenseviewer/display/IFramePresenter.hpp"

#include <memory>
#include <iostream>
#include <stdexcept>
#include <string>
#include <sstream>
#include <utility>
#include <vector>

namespace {

rsv::FrameBundle makeMotionBundle(const std::string& name)
{
    rsv::FrameBundle bundle;
    bundle.motionSamples.push_back(rsv::MotionSample {
        name,
        "unit",
        cv::Vec3f(1.0F, 2.0F, 3.0F),
        10.0,
    });
    return bundle;
}

class FakeFrameSource final : public rsv::IFrameSource {
public:
    void start() override
    {
        ++startCalls;
        if (throwOnStart) {
            throw std::runtime_error("start failed");
        }
    }

    void stop() noexcept override
    {
        ++stopCalls;
    }

    bool poll(rsv::FrameBundle& output) override
    {
        ++pollCalls;
        if (throwOnPoll) {
            throw std::runtime_error("poll failed");
        }

        const std::size_t index = static_cast<std::size_t>(pollCalls - 1);
        if (index >= pollResults.size()) {
            return false;
        }

        if (!pollResults[index]) {
            return false;
        }

        output = index < bundles.size() ? bundles[index] : makeMotionBundle("Default");
        return true;
    }

    int startCalls = 0;
    int stopCalls = 0;
    int pollCalls = 0;
    bool throwOnStart = false;
    bool throwOnPoll = false;
    std::vector<bool> pollResults;
    std::vector<rsv::FrameBundle> bundles;
};

class FakePresenter final : public rsv::IFramePresenter {
public:
    bool present(const rsv::FrameBundle& bundle) override
    {
        ++presentCalls;
        if (throwOnPresent) {
            throw std::runtime_error("present failed");
        }

        presentedBundles.push_back(bundle);
        const std::size_t index = static_cast<std::size_t>(presentCalls - 1);
        return index < presentResults.size() ? presentResults[index] : false;
    }

    bool idle() override
    {
        ++idleCalls;
        if (throwOnIdle) {
            throw std::runtime_error("idle failed");
        }

        const std::size_t index = static_cast<std::size_t>(idleCalls - 1);
        return index < idleResults.size() ? idleResults[index] : false;
    }

    int presentCalls = 0;
    int idleCalls = 0;
    bool throwOnPresent = false;
    bool throwOnIdle = false;
    std::vector<bool> presentResults;
    std::vector<bool> idleResults;
    std::vector<rsv::FrameBundle> presentedBundles;
};

class ScopedCerrRedirect final {
public:
    ScopedCerrRedirect()
        : previousBuffer_(std::cerr.rdbuf(sink_.rdbuf()))
    {
    }

    ~ScopedCerrRedirect()
    {
        std::cerr.rdbuf(previousBuffer_);
    }

    ScopedCerrRedirect(const ScopedCerrRedirect&) = delete;
    ScopedCerrRedirect& operator=(const ScopedCerrRedirect&) = delete;

private:
    std::ostringstream sink_;
    std::streambuf* previousBuffer_ = nullptr;
};

} // namespace

void application_presents_polled_frames_until_presenter_stops()
{
    auto source = std::make_unique<FakeFrameSource>();
    auto presenter = std::make_unique<FakePresenter>();
    FakeFrameSource* sourceView = source.get();
    FakePresenter* presenterView = presenter.get();

    sourceView->pollResults = {true};
    sourceView->bundles = {makeMotionBundle("Gyro")};
    presenterView->presentResults = {false};

    rsv::Application app(std::move(source), std::move(presenter));
    const int result = app.run();

    TEST_ASSERT_EQUAL_INT(0, result);
    TEST_ASSERT_EQUAL_INT(1, sourceView->startCalls);
    TEST_ASSERT_EQUAL_INT(1, sourceView->pollCalls);
    TEST_ASSERT_EQUAL_INT(1, sourceView->stopCalls);
    TEST_ASSERT_EQUAL_INT(1, presenterView->presentCalls);
    TEST_ASSERT_EQUAL_INT(0, presenterView->idleCalls);
    TEST_ASSERT_EQUAL_size_t(1, presenterView->presentedBundles.size());
    TEST_ASSERT_EQUAL_size_t(1, presenterView->presentedBundles[0].motionSamples.size());
    TEST_ASSERT_TRUE(presenterView->presentedBundles[0].motionSamples[0].name == "Gyro");
}

void application_idles_when_no_frame_is_available()
{
    auto source = std::make_unique<FakeFrameSource>();
    auto presenter = std::make_unique<FakePresenter>();
    FakeFrameSource* sourceView = source.get();
    FakePresenter* presenterView = presenter.get();

    sourceView->pollResults = {false};
    presenterView->idleResults = {false};

    rsv::Application app(std::move(source), std::move(presenter));
    const int result = app.run();

    TEST_ASSERT_EQUAL_INT(0, result);
    TEST_ASSERT_EQUAL_INT(1, sourceView->startCalls);
    TEST_ASSERT_EQUAL_INT(1, sourceView->pollCalls);
    TEST_ASSERT_EQUAL_INT(1, sourceView->stopCalls);
    TEST_ASSERT_EQUAL_INT(0, presenterView->presentCalls);
    TEST_ASSERT_EQUAL_INT(1, presenterView->idleCalls);
}

void application_continues_after_idle_until_a_frame_is_presented()
{
    auto source = std::make_unique<FakeFrameSource>();
    auto presenter = std::make_unique<FakePresenter>();
    FakeFrameSource* sourceView = source.get();
    FakePresenter* presenterView = presenter.get();

    sourceView->pollResults = {false, true};
    sourceView->bundles = {makeMotionBundle("unused"), makeMotionBundle("Accel")};
    presenterView->idleResults = {true};
    presenterView->presentResults = {false};

    rsv::Application app(std::move(source), std::move(presenter));
    const int result = app.run();

    TEST_ASSERT_EQUAL_INT(0, result);
    TEST_ASSERT_EQUAL_INT(2, sourceView->pollCalls);
    TEST_ASSERT_EQUAL_INT(1, presenterView->idleCalls);
    TEST_ASSERT_EQUAL_INT(1, presenterView->presentCalls);
    TEST_ASSERT_EQUAL_INT(1, sourceView->stopCalls);
    TEST_ASSERT_TRUE(presenterView->presentedBundles[0].motionSamples[0].name == "Accel");
}

void application_stops_source_and_returns_failure_when_start_throws()
{
    auto source = std::make_unique<FakeFrameSource>();
    auto presenter = std::make_unique<FakePresenter>();
    FakeFrameSource* sourceView = source.get();
    FakePresenter* presenterView = presenter.get();

    sourceView->throwOnStart = true;

    rsv::Application app(std::move(source), std::move(presenter));
    const ScopedCerrRedirect redirect;
    const int result = app.run();

    TEST_ASSERT_EQUAL_INT(1, result);
    TEST_ASSERT_EQUAL_INT(1, sourceView->startCalls);
    TEST_ASSERT_EQUAL_INT(1, sourceView->stopCalls);
    TEST_ASSERT_EQUAL_INT(0, sourceView->pollCalls);
    TEST_ASSERT_EQUAL_INT(0, presenterView->presentCalls);
    TEST_ASSERT_EQUAL_INT(0, presenterView->idleCalls);
}

void application_stops_source_and_returns_failure_when_poll_throws()
{
    auto source = std::make_unique<FakeFrameSource>();
    auto presenter = std::make_unique<FakePresenter>();
    FakeFrameSource* sourceView = source.get();

    sourceView->throwOnPoll = true;

    rsv::Application app(std::move(source), std::move(presenter));
    const ScopedCerrRedirect redirect;
    const int result = app.run();

    TEST_ASSERT_EQUAL_INT(1, result);
    TEST_ASSERT_EQUAL_INT(1, sourceView->startCalls);
    TEST_ASSERT_EQUAL_INT(1, sourceView->pollCalls);
    TEST_ASSERT_EQUAL_INT(1, sourceView->stopCalls);
}

void application_stops_source_and_returns_failure_when_presenter_throws()
{
    auto source = std::make_unique<FakeFrameSource>();
    auto presenter = std::make_unique<FakePresenter>();
    FakeFrameSource* sourceView = source.get();
    FakePresenter* presenterView = presenter.get();

    sourceView->pollResults = {true};
    presenterView->throwOnPresent = true;

    rsv::Application app(std::move(source), std::move(presenter));
    const ScopedCerrRedirect redirect;
    const int result = app.run();

    TEST_ASSERT_EQUAL_INT(1, result);
    TEST_ASSERT_EQUAL_INT(1, sourceView->startCalls);
    TEST_ASSERT_EQUAL_INT(1, sourceView->pollCalls);
    TEST_ASSERT_EQUAL_INT(1, sourceView->stopCalls);
    TEST_ASSERT_EQUAL_INT(1, presenterView->presentCalls);
}
