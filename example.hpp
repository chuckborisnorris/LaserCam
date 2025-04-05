// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#pragma once

#include <librealsense2/rs.hpp>

#include <string>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <iomanip>
#include <cmath>
#include <map>
#include <functional>

#ifndef PI
const double PI = 3.14159265358979323846;
#endif
const size_t IMU_FRAME_WIDTH = 1280;
const size_t IMU_FRAME_HEIGHT = 720;
//////////////////////////////
// Basic Data Types         //
//////////////////////////////

struct float3 { 
    float x, y, z; 
    float3 operator*(float t)
    {
        return { x * t, y * t, z * t };
    }

    float3 operator-(float t)
    {
        return { x - t, y - t, z - t };
    }

    void operator*=(float t)
    {
        x = x * t;
        y = y * t;
        z = z * t;
    }

    void operator=(float3 other)
    {
        x = other.x;
        y = other.y;
        z = other.z;
    }

    void add(float t1, float t2, float t3)
    {
        x += t1;
        y += t2;
        z += t3;
    }
};
struct float2 { float x, y; };


