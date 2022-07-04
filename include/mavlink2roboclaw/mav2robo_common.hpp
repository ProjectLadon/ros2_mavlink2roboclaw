// Copyright (c) 2022 Ladon Robotics
// Author: Pierce Nichols <pierce@ladonrobotics.com>

#pragma once

#include <string>
#include <algorithm>
#include <cctype>

namespace mav2robo
{
    enum class RoboclawCmdType {None, Position, Velocity, Duty};

    inline void tolower_str(std::string &data)
    {
        std::transform(data.begin(), data.end(), data.begin(),
            [](unsigned char c){ return std::tolower(c); });
    }

}