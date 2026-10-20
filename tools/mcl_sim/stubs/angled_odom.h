#pragma once
#include "lemlib/api.hpp"

namespace angledOdom {
    void setPose(float x, float y, float theta);
    void offsetPose(float dx, float dy, float dtheta);
    lemlib::Pose getPose();
}
