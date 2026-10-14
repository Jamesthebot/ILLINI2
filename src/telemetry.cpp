#include "telemetry.h"

telemetry::telemetry() {
    f = fopen("/usd/telemetry.txt", "w");
    fputs("X,Y,theta\n", f);
}

telemetry::~telemetry() {
    fclose(f);
}

void telemetry::writePose(double x, double y, double theta) {
    fprintf(f, "%f,%f,%f\n", x, y, theta);
}

