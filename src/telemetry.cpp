
#include "telemetry.h"

telemetry::telemetry() {
    f = fopen("/usd/telemetry.txt", "w");
    if (f) fputs("X,Y,theta\n", f);
}

telemetry::~telemetry() {
    if (f) fclose(f);
}

void telemetry::writePose(double x, double y, double theta) {
    if (!f) return;
    fprintf(f, "%f,%f,%f\n", x, y, theta);
    fflush(f);
}

