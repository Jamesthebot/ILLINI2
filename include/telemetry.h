#pragma once
#include <stdio.h>

class telemetry {
    private:
        FILE* f;
        int maxParticles_;
        int numSensors_;
        int sensorsWritten_;
        int particlesWritten_;
        int framesSinceFlush_;
        bool inFrame_;

        void padSensors();
    public:
        telemetry(const char* path, int maxParticles, int numSensors, int framePeriodMs);
        ~telemetry();
        bool isOpen() const { return f != nullptr; }
        int getMaxParticles() const { return maxParticles_; }
        int getNumSensors() const { return numSensors_; }

        void beginFrame(double x, double y, double theta, int numParticles);
        void writeSensor(double reading, double offsetX, double offsetY, double direction);
        void writeParticle(double x, double y, double theta, double weight, const double* simulatedDistances, int count);
        void endFrame();
};
