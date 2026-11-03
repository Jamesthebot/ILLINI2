#include "cascade.h"

void Cascade::SetHeight(double targetHeight, uint32_t timeoutMs) {
    if (heightMoveRunning) {
        return;
    }
    heightMoveRunning = true;

    pros::Task heightTask([this, targetHeight, timeoutMs] {
        const double target = targetHeight2Deg(targetHeight);
        const double toleranceDeg = targetHeight2Deg(0.15);
        const uint32_t start = pros::millis();
        cascadePID.reset();

        while (pros::millis() - start < timeoutMs) {
            std::int32_t position = rotationSensor.get_position();
            if (position == PROS_ERR) {
                break;
            }

            double error = target - position / 100.0;
            if (std::abs(error) < toleranceDeg) {
                break;
            }

            cascadeMotors.move(cascadePID.update(error));
            pros::delay(10);
        }

        cascadeMotors.move(0);
        heightMoveRunning = false;
    }, "cascade");
}

double Cascade::targetHeight2Deg(double targetHeight) {
    double distPerRot = M_PI * spoolDiameter * stages;
    return (targetHeight / distPerRot) * 360.0;
}
