#include "unity.h"

#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

extern void setUp();
extern void tearDown();

namespace unity {
namespace {

struct TestFailure final : std::runtime_error {
    TestFailure(std::string failureMessage, const char* failureFile, int failureLine)
        : std::runtime_error(std::move(failureMessage))
        , file(failureFile)
        , line(failureLine)
    {
    }

    std::string file;
    int line = 0;
};

struct UnityState {
    std::string file;
    int testsRun = 0;
    int testsFailed = 0;
};

UnityState& state()
{
    static UnityState currentState;
    return currentState;
}

} // namespace

void begin(const char* file)
{
    UnityState& currentState = state();
    currentState.file = file;
    currentState.testsRun = 0;
    currentState.testsFailed = 0;

    std::cout << "Unity test run: " << currentState.file << '\n';
}

int end()
{
    const UnityState& currentState = state();
    const int passed = currentState.testsRun - currentState.testsFailed;

    std::cout << "Tests run: " << currentState.testsRun << ", passed: " << passed
              << ", failed: " << currentState.testsFailed << '\n';

    return currentState.testsFailed == 0 ? 0 : 1;
}

void runTest(void (*testFunction)(), const char* testName, int line)
{
    UnityState& currentState = state();
    ++currentState.testsRun;

    try {
        setUp();
        testFunction();
        tearDown();
        std::cout << currentState.file << ':' << line << ": " << testName << ": PASS\n";
    } catch (const TestFailure& failure) {
        ++currentState.testsFailed;
        try {
            tearDown();
        } catch (...) {
        }
        std::cout << failure.file << ':' << failure.line << ": " << testName << ": FAIL: "
                  << failure.what() << '\n';
    } catch (const std::exception& exception) {
        ++currentState.testsFailed;
        try {
            tearDown();
        } catch (...) {
        }
        std::cout << currentState.file << ':' << line << ": " << testName
                  << ": FAIL: unexpected exception: " << exception.what() << '\n';
    } catch (...) {
        ++currentState.testsFailed;
        try {
            tearDown();
        } catch (...) {
        }
        std::cout << currentState.file << ':' << line << ": " << testName
                  << ": FAIL: unknown exception\n";
    }
}

void fail(const std::string& message, const char* file, int line)
{
    throw TestFailure(message, file, line);
}

void assertTrue(bool condition, const char* conditionText, const char* file, int line)
{
    if (!condition) {
        fail(std::string("Expected true: ") + conditionText, file, line);
    }
}

void assertTrueMessage(bool condition, const char* message, const char* file, int line)
{
    if (!condition) {
        fail(message, file, line);
    }
}

void assertFalse(bool condition, const char* conditionText, const char* file, int line)
{
    if (condition) {
        fail(std::string("Expected false: ") + conditionText, file, line);
    }
}

void assertNotNull(const void* pointer, const char* pointerText, const char* file, int line)
{
    if (pointer == nullptr) {
        fail(std::string("Expected non-null pointer: ") + pointerText, file, line);
    }
}

} // namespace unity
