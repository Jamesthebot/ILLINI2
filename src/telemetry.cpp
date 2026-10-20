#include "telemetry.h"

namespace {
constexpr int FLUSH_EVERY_FRAMES = 10;
constexpr size_t WRITE_BUFFER_BYTES = 16384;
}

telemetry::telemetry(const char* path, int maxParticles, int numSensors)
    : f(nullptr), maxParticles_(maxParticles < 0 ? 0 : maxParticles), numSensors_(numSensors < 0 ? 0 : numSensors),
      sensorsWritten_(0), particlesWritten_(0), framesSinceFlush_(0), inFrame_(false) {
    f = fopen(path, "w");
    if (!f) return;
    setvbuf(f, nullptr, _IOFBF, WRITE_BUFFER_BYTES);

    fprintf(f, "MaxParticles,%d\n", maxParticles_);
    fprintf(f, "NumDistanceSensors,%d\n", numSensors_);
    fputs("X,Y,Theta,NumParticles", f);
    for (int j = 0; j < numSensors_; j++) {
        fprintf(f, ",DistanceSensorReading%d,DistanceSensorOffset%d,DistanceSensorDirection%d", j, j, j);
    }
    for (int i = 0; i < maxParticles_; i++) {
        fprintf(f, ",ParticlePosition%d,ParticleWeight%d", i, i);
        for (int j = 0; j < numSensors_; j++) fprintf(f, ",SimulatedDistance%d_%d", i, j);
    }
    fputc('\n', f);
    fflush(f);
}

telemetry::~telemetry() {
    if (f) fclose(f);
}

void telemetry::beginFrame(double x, double y, double theta, int numParticles) {
    if (!f) return;
    if (inFrame_) endFrame();
    if (numParticles < 0) numParticles = 0;
    if (numParticles > maxParticles_) numParticles = maxParticles_;
    fprintf(f, "%.2f,%.2f,%.2f,%d", x, y, theta, numParticles);
    sensorsWritten_ = 0;
    particlesWritten_ = 0;
    inFrame_ = true;
}

void telemetry::writeSensor(double reading, double offsetX, double offsetY, double direction) {
    if (!f || !inFrame_ || particlesWritten_ > 0 || sensorsWritten_ >= numSensors_) return;
    fprintf(f, ",%.2f,%.2f;%.2f,%.2f", reading, offsetX, offsetY, direction);
    sensorsWritten_++;
}

void telemetry::padSensors() {
    for (; sensorsWritten_ < numSensors_; sensorsWritten_++) fputs(",,,", f);
}

void telemetry::writeParticle(double x, double y, double theta, double weight, const double* simulatedDistances, int count) {
    if (!f || !inFrame_ || particlesWritten_ >= maxParticles_) return;
    padSensors();
    fprintf(f, ",%.2f;%.2f;%.2f,%.4g", x, y, theta, weight);
    for (int j = 0; j < numSensors_; j++) {
        if (simulatedDistances && j < count) fprintf(f, ",%.2f", simulatedDistances[j]);
        else fputc(',', f);
    }
    particlesWritten_++;
}

void telemetry::endFrame() {
    if (!f || !inFrame_) return;
    padSensors();
    for (; particlesWritten_ < maxParticles_; particlesWritten_++) {
        fputs(",,", f);
        for (int j = 0; j < numSensors_; j++) fputc(',', f);
    }
    fputc('\n', f);
    inFrame_ = false;
    if (++framesSinceFlush_ >= FLUSH_EVERY_FRAMES) {
        fflush(f);
        framesSinceFlush_ = 0;
    }
}
