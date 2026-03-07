/*
 * Copyright (c) 2024-2026 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>

#include "loop.hpp"

int main()
{
    std::atomic<int> loop_calls{0};
    std::mutex mutex;
    std::condition_variable cv;
    bool handled = false;
    std::string handled_name;
    std::string handled_error;

    LoopFunc loop(
        "loop_exception_test",
        0.001f,
        [&loop_calls]()
        {
            if (loop_calls.fetch_add(1) == 0)
            {
                throw std::runtime_error("boom");
            }
        },
        -1,
        [&](const std::string& name, const std::string& error)
        {
            std::lock_guard<std::mutex> lock(mutex);
            handled = true;
            handled_name = name;
            handled_error = error;
            cv.notify_one();
        });

    loop.start();

    {
        std::unique_lock<std::mutex> lock(mutex);
        if (!cv.wait_for(lock, std::chrono::milliseconds(500), [&handled]() { return handled; }))
        {
            std::cerr << "Loop exception handler was not called within timeout\n";
            loop.shutdown();
            return 1;
        }
    }

    loop.shutdown();

    if (handled_name != "loop_exception_test")
    {
        std::cerr << "Unexpected loop name: " << handled_name << "\n";
        return 1;
    }
    if (handled_error.find("boom") == std::string::npos)
    {
        std::cerr << "Unexpected loop error: " << handled_error << "\n";
        return 1;
    }
    if (loop_calls.load() != 1)
    {
        std::cerr << "Loop kept running after exception, calls=" << loop_calls.load() << "\n";
        return 1;
    }

    std::cout << "test_loop_exception_handler passed\n";
    return 0;
}
