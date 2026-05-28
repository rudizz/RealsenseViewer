#pragma once

#include <cstddef>
#include <sstream>
#include <string>

namespace unity {

void begin(const char* file);
int end();
void runTest(void (*testFunction)(), const char* testName, int line);

[[noreturn]] void fail(const std::string& message, const char* file, int line);

template <typename T>
std::string toString(const T& value)
{
    std::ostringstream stream;
    stream << value;
    return stream.str();
}

template <typename Expected, typename Actual>
void assertEqual(
    const Expected& expected,
    const Actual& actual,
    const char* expectedText,
    const char* actualText,
    const char* file,
    int line)
{
    if (!(expected == actual)) {
        fail(std::string("Expected ") + expectedText + " == " + actualText + " (" + toString(expected)
                 + " vs " + toString(actual) + ")",
            file,
            line);
    }
}

void assertTrue(bool condition, const char* conditionText, const char* file, int line);
void assertTrueMessage(bool condition, const char* message, const char* file, int line);
void assertFalse(bool condition, const char* conditionText, const char* file, int line);
void assertNotNull(const void* pointer, const char* pointerText, const char* file, int line);

} // namespace unity

#define UNITY_BEGIN() ::unity::begin(__FILE__)
#define UNITY_END() ::unity::end()
#define RUN_TEST(testFunction) ::unity::runTest(testFunction, #testFunction, __LINE__)

#define TEST_FAIL_MESSAGE(message) ::unity::fail((message), __FILE__, __LINE__)

#define TEST_ASSERT_TRUE(condition) ::unity::assertTrue(static_cast<bool>(condition), #condition, __FILE__, __LINE__)
#define TEST_ASSERT_TRUE_MESSAGE(condition, message) \
    ::unity::assertTrueMessage(static_cast<bool>(condition), (message), __FILE__, __LINE__)
#define TEST_ASSERT_FALSE(condition) ::unity::assertFalse(static_cast<bool>(condition), #condition, __FILE__, __LINE__)
#define TEST_ASSERT_NOT_NULL(pointer) \
    ::unity::assertNotNull(static_cast<const void*>(pointer), #pointer, __FILE__, __LINE__)

#define TEST_ASSERT_EQUAL(expected, actual) \
    ::unity::assertEqual((expected), (actual), #expected, #actual, __FILE__, __LINE__)
#define TEST_ASSERT_EQUAL_INT(expected, actual) TEST_ASSERT_EQUAL((expected), (actual))
#define TEST_ASSERT_EQUAL_size_t(expected, actual) \
    ::unity::assertEqual(static_cast<std::size_t>(expected), static_cast<std::size_t>(actual), #expected, #actual, __FILE__, __LINE__)
