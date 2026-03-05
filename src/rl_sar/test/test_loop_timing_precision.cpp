/*
 * Copyright (c) 2024-2026 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include <chrono>
#include <iostream>

#include "loop.hpp"

int main()
{
    using namespace std::chrono;

    {
        const auto sleep_us = LoopFunc::ComputeSleepDuration(0.002f, microseconds(1500));
        if (sleep_us.count() != 500)
        {
            std::cerr << "Expected 500us, got " << sleep_us.count() << "us\n";
            return 1;
        }
    }

    {
        const auto sleep_us = LoopFunc::ComputeSleepDuration(0.002f, microseconds(2200));
        if (sleep_us.count() != 0)
        {
            std::cerr << "Expected 0us on overrun, got " << sleep_us.count() << "us\n";
            return 1;
        }
    }

    {
        const auto sleep_us = LoopFunc::ComputeSleepDuration(0.0005f, microseconds(0));
        if (sleep_us.count() != 500)
        {
            std::cerr << "Expected 500us period conversion, got " << sleep_us.count() << "us\n";
            return 1;
        }
    }

    {
        const auto sleep_us = LoopFunc::ComputeSleepDuration(0.0f, microseconds(0));
        if (sleep_us.count() != 0)
        {
            std::cerr << "Expected 0us for non-positive period, got " << sleep_us.count() << "us\n";
            return 1;
        }
    }

    std::cout << "test_loop_timing_precision passed\n";
    return 0;
}
