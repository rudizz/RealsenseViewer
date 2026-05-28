#include "unity.h"

#include "realsenseviewer/concurrency/ConcurrentQueue.hpp"

#include <atomic>
#include <chrono>
#include <stdexcept>
#include <thread>

using namespace std::chrono_literals;

void setUp()
{
}

void tearDown()
{
}

void constructor_rejects_zero_capacity()
{
    bool threw = false;

    try {
        rsv::ConcurrentQueue<int> queue(0);
    } catch (const std::invalid_argument&) {
        threw = true;
    }

    TEST_ASSERT_TRUE_MESSAGE(threw, "ConcurrentQueue should reject zero capacity");
}

void try_push_and_try_pop_preserve_fifo_order()
{
    rsv::ConcurrentQueue<int> queue(2);
    int output = 0;

    TEST_ASSERT_TRUE(queue.tryPush(10));
    TEST_ASSERT_TRUE(queue.tryPush(20));

    TEST_ASSERT_TRUE(queue.tryPop(output));
    TEST_ASSERT_EQUAL_INT(10, output);

    TEST_ASSERT_TRUE(queue.tryPop(output));
    TEST_ASSERT_EQUAL_INT(20, output);

    TEST_ASSERT_FALSE(queue.tryPop(output));
}

void try_push_rejects_items_when_queue_is_full()
{
    rsv::ConcurrentQueue<int> queue(1);
    int output = 0;

    TEST_ASSERT_TRUE(queue.tryPush(10));
    TEST_ASSERT_FALSE(queue.tryPush(20));
    TEST_ASSERT_EQUAL_size_t(1, queue.droppedCount());

    TEST_ASSERT_TRUE(queue.tryPop(output));
    TEST_ASSERT_EQUAL_INT(10, output);
    TEST_ASSERT_FALSE(queue.tryPop(output));
}

void close_rejects_new_pushes_and_wakes_waiters()
{
    rsv::ConcurrentQueue<int> queue(1);
    std::atomic<bool> waitPopReturned { false };
    std::atomic<bool> waitPopResult { true };

    std::thread consumer([&queue, &waitPopReturned, &waitPopResult] {
        int output = 0;
        waitPopResult = queue.waitPop(output);
        waitPopReturned = true;
    });

    std::this_thread::sleep_for(20ms);
    queue.close();
    consumer.join();

    TEST_ASSERT_TRUE(waitPopReturned.load());
    TEST_ASSERT_FALSE(waitPopResult.load());
    TEST_ASSERT_FALSE(queue.tryPush(10));
}

void reset_clears_items_drop_count_and_closed_state()
{
    rsv::ConcurrentQueue<int> queue(1);
    int output = 0;

    TEST_ASSERT_TRUE(queue.tryPush(10));
    TEST_ASSERT_FALSE(queue.tryPush(20));
    queue.close();

    queue.reset();

    TEST_ASSERT_EQUAL_size_t(0, queue.droppedCount());
    TEST_ASSERT_TRUE(queue.tryPush(30));
    TEST_ASSERT_TRUE(queue.tryPop(output));
    TEST_ASSERT_EQUAL_INT(30, output);
}

void wait_pop_receives_item_from_producer_thread()
{
    rsv::ConcurrentQueue<int> queue(1);
    std::atomic<bool> pushed { false };
    int output = 0;

    std::thread producer([&queue, &pushed] {
        std::this_thread::sleep_for(20ms);
        pushed = queue.tryPush(42);
    });

    const bool popped = queue.waitPop(output);
    producer.join();

    TEST_ASSERT_TRUE(popped);
    TEST_ASSERT_TRUE(pushed.load());
    TEST_ASSERT_EQUAL_INT(42, output);
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(constructor_rejects_zero_capacity);
    RUN_TEST(try_push_and_try_pop_preserve_fifo_order);
    RUN_TEST(try_push_rejects_items_when_queue_is_full);
    RUN_TEST(close_rejects_new_pushes_and_wakes_waiters);
    RUN_TEST(reset_clears_items_drop_count_and_closed_state);
    RUN_TEST(wait_pop_receives_item_from_producer_thread);
    return UNITY_END();
}
