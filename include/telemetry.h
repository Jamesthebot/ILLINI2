#pragma once
#include <stdio.h>

class telemetry {
    private:
        FILE* f;
    public:
        telemetry();
        ~telemetry();
        void writePose(double x, double y, double theta);
};