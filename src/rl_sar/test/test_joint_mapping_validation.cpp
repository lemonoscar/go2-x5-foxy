/*
 * Copyright (c) 2024-2026 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include <iostream>
#include <string>
#include <vector>

#include "rl_validation.hpp"

int main()
{
    std::string error;

    {
        const std::vector<int> mapping = {0, 2, 1, 3};
        if (!RLValidation::ValidateJointMapping(mapping, 4, 4, &error))
        {
            std::cerr << "Valid mapping failed: " << error << "\n";
            return 1;
        }
    }

    {
        const std::vector<int> mapping = {0, 1, 2};
        if (RLValidation::ValidateJointMapping(mapping, 4, 4, &error))
        {
            std::cerr << "Expected size mismatch to fail\n";
            return 1;
        }
    }

    {
        const std::vector<int> mapping = {0, 1, -1, 2};
        if (RLValidation::ValidateJointMapping(mapping, 4, 4, &error))
        {
            std::cerr << "Expected negative index to fail\n";
            return 1;
        }
    }

    {
        const std::vector<int> mapping = {0, 1, 2, 9};
        if (RLValidation::ValidateJointMapping(mapping, 4, 4, &error))
        {
            std::cerr << "Expected out-of-range index to fail\n";
            return 1;
        }
    }

    std::cout << "test_joint_mapping_validation passed\n";
    return 0;
}
