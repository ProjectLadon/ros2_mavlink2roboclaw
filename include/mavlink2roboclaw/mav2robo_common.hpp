// Copyright (c) 2022 Ladon Robotics
// Author: Pierce Nichols <pierce@ladonrobotics.com>

#pragma once

#include <string>
#include <algorithm>
#include <cctype>
#include <cmath>

namespace mav2robo
{
    enum class RoboclawCmdType {None, Position, Velocity, Duty};

    inline void tolower_str(std::string &data)
    {
        std::transform(data.begin(), data.end(), data.begin(),
            [](unsigned char c){ return std::tolower(c); });
    }

    inline float fbound(float val, float upper, float lower)
    {
        float max_val = fmax(upper, lower);
        float min_val = fmin(upper, lower);
        return (fmax(min_val, fmin(val, max_val)));
    }

}