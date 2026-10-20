#pragma once

namespace lemlib {
    class Pose {
        public:
            float x;
            float y;
            float theta;

            Pose(float x, float y, float theta = 0) : x(x), y(y), theta(theta) {}

            Pose operator+(const Pose& other) const { return Pose(x + other.x, y + other.y, theta); }
            Pose operator-(const Pose& other) const { return Pose(x - other.x, y - other.y, theta); }
    };
}
