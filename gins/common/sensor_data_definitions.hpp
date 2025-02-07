#pragma once
#include "eigen_definitions.hpp"


namespace halo{
    struct IMU{
        Vec3d linear_acc_;
        Vec3d gyro_;
        double timestamp_;
    };
}

