/*
 * Copyright (c) 2024-2026 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RL_VALIDATION_HPP
#define RL_VALIDATION_HPP

#include <string>
#include <vector>

namespace RLValidation
{
inline bool ValidateJointMapping(
    const std::vector<int>& joint_mapping,
    int num_dofs,
    int mapped_source_size,
    std::string* error = nullptr)
{
    if (num_dofs <= 0)
    {
        if (error)
        {
            *error = "num_of_dofs must be positive";
        }
        return false;
    }
    if (mapped_source_size <= 0)
    {
        if (error)
        {
            *error = "mapped source size must be positive";
        }
        return false;
    }
    if (joint_mapping.size() != static_cast<size_t>(num_dofs))
    {
        if (error)
        {
            *error = "joint_mapping size mismatch, expected " + std::to_string(num_dofs) +
                     ", got " + std::to_string(joint_mapping.size());
        }
        return false;
    }

    for (size_t i = 0; i < joint_mapping.size(); ++i)
    {
        const int mapped = joint_mapping[i];
        if (mapped < 0 || mapped >= mapped_source_size)
        {
            if (error)
            {
                *error = "joint_mapping[" + std::to_string(i) + "]=" + std::to_string(mapped) +
                         " out of range [0," + std::to_string(mapped_source_size - 1) + "]";
            }
            return false;
        }
    }
    return true;
}
} // namespace RLValidation

#endif // RL_VALIDATION_HPP
