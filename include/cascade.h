#pragma once

#include "api.h"
#include "lemlib/api.hpp"
#include <algorithm>
#include <cmath>
using namespace pros;

class Cascade {
    public:
        /**
         * Constructor
         *
         * @param motorPort1 first cascade motor port
         * @param motorPort2 second cascade motor port
         * @param rotationPort rotation sensor port
         * @param pulleyDiameter diameter of pulley in inches
         * @param numStages number of stages in the cascade
         */
        Cascade(std::int8_t motorPort1, std::int8_t motorPort2, std::int8_t rotationPort, double pulleyDiameter, int numStages)
            : cascadeMotors({motorPort1, motorPort2}), rotationSensor(rotationPort), spoolDiameter(pulleyDiameter), stages(numStages), cascadePID(kP, kI, kD, antiWindup, signFlipReset) {};

        /**
         * Sets the motors to hold and makes the current position height 0. Call once in initialize()
         */
        void init() {
            cascadeMotors.set_brake_mode_all(MotorBrake::hold);
            resetHeight();
        };

        /**
         * Moves the cascade @ speed until called again (move(0) to stop and hold). Ignored while SetHeight is running.
         * Upward movement stops at MAX_HEIGHT, but only when this is called, so keep calling it while moving
         *
         * @param speed -127 to 127, positive is up
         */
        void move(int speed) {
            if (heightMoveRunning) {
                return;
            }

            bool atMaxHeight = rotationSensor.get_position() / 100.0 >= targetHeight2Deg(MAX_HEIGHT);
            bool atMinHeight = rotationSensor.get_position() / 100.0 <= targetHeight2Deg(MIN_HEIGHT);
            if (speed == 0 || (speed > 0 && atMaxHeight) || (speed < 0 && atMinHeight)) {
                cascadeMotors.brake();
                return;
            }

            cascadeMotors.move(speed);
        };

        /**
         * Starts task to move the cascade to a height and prevents other cascade operations while the cascade is being moved
         *
         * @param inches target height in inches, limited to 0 to MAX_HEIGHT
         * @param timeoutMs stops trying after this many milliseconds
         */
        void SetHeight(double inches, uint32_t timeoutMs = 3000);

        void raiseToMin() {
            if (rotationSensor.get_position() / 100.0 < targetHeight2Deg(MIN_HEIGHT)) {
                SetHeight(MIN_HEIGHT);
            }
        }

        /**
         * Makes the current position height 0
         */
        void resetHeight() {
            if (heightMoveRunning) {
                return;
            }
            rotationSensor.reset_position();
        };
        double getDeg() { return rotationSensor.get_position() / 100.0; };
        double getHeight() { return getDeg() / 360.0 * M_PI * spoolDiameter * stages; };

    private:
        double targetHeight2Deg(double inches);

        static constexpr double MAX_HEIGHT = 50.0; // inches
        static constexpr double MIN_HEIGHT = 2.0; //inches

        float kP = 1.5; //24inches
        float kI = 0.0;
        float kD = 0.0;
        float antiWindup = 0.0;
        bool signFlipReset = false;

        MotorGroup cascadeMotors;
        Rotation rotationSensor;
        double spoolDiameter;
        int stages;
        lemlib::PID cascadePID;

        bool heightMoveRunning = false;
};
