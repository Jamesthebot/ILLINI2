#include "cascade.h"

void Cascade::SetHeight(double targetHeight, uint32_t timeoutMs) {
    const double target = targetHeight2Deg(targetHeight);
    const double toleranceDeg = targetHeight2Deg(0.15);
    const uint32_t start = pros::millis();
    cascadePID.reset();

    while (pros::millis() - start < timeoutMs) {
        double currentDeg = rotationSensor.get_position() / 100.0;
        double error = target - currentDeg;

        if (std::abs(error) < toleranceDeg) {
            break;
        }

        move(cascadePID.update(error));
        pros::delay(10);
    }
    move(0);
}

double Cascade::targetHeight2Deg(double targetHeight) {
    double distPerRot = M_PI * spoolDiameter * stages;
    return (targetHeight / distPerRot) * 360.0;
}
