#include "mcl.h"
#include "fieldRepresentation.h"
#include "angled_odom.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <mutex>
#include <random>
#include <vector>

// ============================================================================
// Tuning
// ============================================================================
namespace {

constexpr int DEFAULT_PARTICLES = 300;

constexpr double INIT_POS_STD = 1.5;
constexpr double INIT_HEADING_STD = 2.0;

constexpr double MOTION_POS_BASE = 0.02;
constexpr double MOTION_POS_PER_INCH = 0.05;
constexpr double MOTION_HEADING_BASE = 0.02;
constexpr double MOTION_HEADING_PER_DEG = 0.02;

constexpr int32_t SENSOR_MIN_MM = 20;
constexpr int32_t SENSOR_NO_OBJECT_MM = 9999;
constexpr int32_t SENSOR_CONFIDENCE_MIN_MM = 200;
constexpr int32_t SENSOR_MIN_CONFIDENCE = 30;
constexpr double SENSOR_MAX_RANGE = fieldRepresentation::maxSensorRange;
constexpr double SENSOR_SIGMA_MIN = 15.0 / 25.4;
constexpr double SENSOR_SIGMA_FRACTION = 0.05;
constexpr double MODEL_SIGMA = 0.75;
constexpr double BEAM_SPREAD = 0.1;
constexpr double MIN_HIT_COS = 0.35;

constexpr double P_HIT = 0.8;
constexpr double P_SHORT = 0.12;
constexpr double P_RANDOM = 0.08;
constexpr double SHORT_LAMBDA = 0.05;

constexpr double P_NO_RETURN_WHEN_EXPECTED = 0.15;
constexpr double NO_RETURN_TRUST_RANGE = 48.0;
constexpr double NO_RETURN_TRUST_COS = 0.85;

constexpr double READING_NONE = -1;
constexpr double READING_INVALID = -2;

constexpr double FIELD_HALF = fieldRepresentation::fieldHalfWidth;
constexpr double FIELD_MARGIN = 2.0;

constexpr double POS_CORRECTION_GAIN = 0.3;
constexpr double HEADING_CORRECTION_GAIN = 0.1;
constexpr bool   CORRECT_HEADING = true;
constexpr double MAX_CORRECTION = 6.0;
constexpr double MAX_HEADING_CORRECTION = 5.0;
constexpr int    FORCE_AFTER_REJECTIONS = 10;
constexpr double MAX_SPREAD_TO_CORRECT = 3.0;

constexpr double WEIGH_MIN_MOVE = 0.5;
constexpr double WEIGH_MIN_TURN = 1.0;

constexpr double RESAMPLE_ESS_FRACTION = 0.5;
constexpr double ROUGHEN_POS_STD = 0.1;
constexpr double ROUGHEN_HEADING_STD = 0.1;

constexpr double ALPHA_SLOW = 0.01;
constexpr double ALPHA_FAST = 0.2;
constexpr double MAX_RANDOM_FRACTION = 0.1;
constexpr double RECOVERY_RADIUS = 12.0;

constexpr uint32_t LOOP_MS = 100;

constexpr double PI = 3.14159265358979323846;
constexpr double SQRT_2PI = 2.50662827463100050242;
constexpr double NEG_INF = -std::numeric_limits<double>::infinity();
constexpr double INF = std::numeric_limits<double>::infinity();

double toRad(double d) { return d * PI / 180.0; }
double toDeg(double r) { return r * 180.0 / PI; }
double wrap180(double d) {
    d = std::fmod(d + 180.0, 360.0);
    if (d < 0) d += 360.0;
    return d - 180.0;
}

std::mt19937& rng() {
    static std::mt19937 gen(static_cast<uint32_t>(pros::micros()));
    return gen;
}
double gaussian(double stddev) {
    if (stddev <= 0) return 0;
    std::normal_distribution<double> d(0.0, stddev);
    return d(rng());
}
double uniform(double lo, double hi) {
    std::uniform_real_distribution<double> d(lo, hi);
    return d(rng());
}

pros::Mutex& mclMutex() {
    static pros::Mutex m;
    return m;
}

bool started = false;
bool needsScatter = true;
int particleCount = DEFAULT_PARTICLES;
double latestImu = 0;
double prevImu = 0;
double lastSpread = INF;
double wSlow = 0, wFast = 0;
int rejectedTicks = 0;
bool weighedOnce = false;
double moveSinceWeigh = 0, turnSinceWeigh = 0;
lemlib::Pose estimatedPose(0, 0, 0);
telemetry* telemetryLog = nullptr;

bool finite(const lemlib::Pose& p) { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.theta); }

double offsetOf(const particle& p, double imuHeading) { return p.getHeading(imuHeading) - imuHeading; }

bool outOfField(const particle& p) {
    double limit = FIELD_HALF - FIELD_MARGIN;
    return !(std::fabs(p.getX()) <= limit && std::fabs(p.getY()) <= limit);
}

void scatter(std::vector<particle>& ps, double x, double y, double offset, double posStd, double headingStd) {
    for (auto& p : ps) {
        p.setPosition(x + gaussian(posStd), y + gaussian(posStd));
        p.setHeadingOffset(offset + gaussian(headingStd));
        p.setWeight(1.0 / ps.size());
    }
}

double readInches(pros::Distance& sensor) {
    int32_t mm = sensor.get();
    if (mm == PROS_ERR) return READING_INVALID;
    if (mm >= SENSOR_NO_OBJECT_MM) return READING_NONE;
    if (mm < SENSOR_MIN_MM) return READING_INVALID;
    if (mm > SENSOR_CONFIDENCE_MIN_MM) {
        int32_t conf = sensor.get_confidence();
        if (conf == PROS_ERR || conf < SENSOR_MIN_CONFIDENCE) return READING_INVALID;
    }
    double in = mm / 25.4;
    return in > SENSOR_MAX_RANGE ? READING_NONE : in;
}

void warnIfMissing(pros::Distance& sensor, const char* name) {
    if (!sensor.is_installed()) printf("[MCL] %s distance sensor not detected on port %d\n", name, sensor.get_port());
}

double readingAt(const fieldRepresentation::distanceSensorDistances& d, size_t i) {
    switch (i) {
        case 0: return d.frontDistance;
        case 1: return d.rightDistance;
        case 2: return d.backDistance;
        case 3: return d.leftDistance;
    }
    return READING_INVALID;
}

double sensorLogLikelihood(double simulated, double simulatedCos, double measured) {
    if (measured == READING_INVALID) return 0.0;
    bool simHit = simulated >= 0;

    if (measured == READING_NONE) {
        bool expected = simHit && simulated < NO_RETURN_TRUST_RANGE && simulatedCos > NO_RETURN_TRUST_COS;
        return expected ? std::log(P_NO_RETURN_WHEN_EXPECTED) : 0.0;
    }

    double p = P_RANDOM / SENSOR_MAX_RANGE;

    double shortRange = simHit ? simulated : SENSOR_MAX_RANGE;
    if (measured < shortRange) {
        p += P_SHORT * SHORT_LAMBDA * std::exp(-SHORT_LAMBDA * measured) / (1 - std::exp(-SHORT_LAMBDA * shortRange));
    }

    if (simHit) {
        double c = std::max(simulatedCos, MIN_HIT_COS);
        double tanIncidence = std::sqrt(1 - c * c) / c;
        double sensorSigma = std::max(SENSOR_SIGMA_MIN, SENSOR_SIGMA_FRACTION * simulated);
        double spreadSigma = BEAM_SPREAD * simulated * tanIncidence;
        double sigma = std::sqrt(sensorSigma * sensorSigma + MODEL_SIGMA * MODEL_SIGMA + spreadSigma * spreadSigma);
        double e = measured - simulated;
        p += P_HIT * std::exp(-e * e / (2 * sigma * sigma)) / (sigma * SQRT_2PI);
    }
    return std::log(p);
}

lemlib::Pose applyCorrection(const lemlib::Pose& est) {
    lemlib::Pose none(0, 0, 0);
    if (!finite(est) || !(lastSpread <= MAX_SPREAD_TO_CORRECT)) { rejectedTicks = 0; return none; }
    lemlib::Pose pose = angledOdom::getPose();
    if (!finite(pose)) return none;
    double cx = est.x - pose.x;
    double cy = est.y - pose.y;
    double ct = CORRECT_HEADING ? wrap180(est.theta - pose.theta) : 0.0;

    bool tooBig = std::hypot(cx, cy) > MAX_CORRECTION || std::fabs(ct) > MAX_HEADING_CORRECTION;
    if (tooBig && ++rejectedTicks < FORCE_AFTER_REJECTIONS) return none; // probably a glitch; persistent = real
    rejectedTicks = 0;

    lemlib::Pose correction(POS_CORRECTION_GAIN * cx, POS_CORRECTION_GAIN * cy, HEADING_CORRECTION_GAIN * ct);
    angledOdom::offsetPose(correction.x, correction.y, correction.theta);
    return correction;
}

}

fieldRepresentation::distanceSensorDistances MCL::sensorData_;
std::vector<particle> MCL::particles_;
lemlib::Pose MCL::prevPose(0, 0, 0);

fieldRepresentation& MCL::getField() {
    static fieldRepresentation field;
    return field;
}

// ============================================================================
// MCL
// ============================================================================
void MCL::init(int numParticles) {
    if (started || numParticles <= 0) return;
    started = true;
    particleCount = numParticles;

    warnIfMissing(distanceFront, "front");
    warnIfMissing(distanceRight, "right");
    warnIfMissing(distanceBack, "back");
    warnIfMissing(distanceLeft, "left");

    pros::Task mclTask([] {
        uint32_t now = pros::millis();
        while (true) {
            {
                std::lock_guard<pros::Mutex> lock(mclMutex());
                step();
            }
            pros::Task::delay_until(&now, LOOP_MS);
        }
    }, "MCL");
}

void MCL::setTelemetry(telemetry* log) {
    std::lock_guard<pros::Mutex> lock(mclMutex());
    telemetryLog = log;
}

void MCL::reset(lemlib::Pose pose) {
    std::lock_guard<pros::Mutex> lock(mclMutex());
    angledOdom::setPose(pose.x, pose.y, pose.theta);
    needsScatter = true;
}

void MCL::step() {
    double imuNow = imu.get_rotation();
    lemlib::Pose pose = angledOdom::getPose();
    if (!std::isfinite(imuNow) || !finite(pose)) return;

    if (needsScatter) {
        needsScatter = false;
        latestImu = prevImu = imuNow;
        prevPose = pose;
        particles_.assign(particleCount, particle());
        scatter(particles_, pose.x, pose.y, pose.theta - imuNow, INIT_POS_STD, INIT_HEADING_STD);
        wSlow = wFast = 0;
        rejectedTicks = 0;
        weighedOnce = false;
        moveSinceWeigh = turnSinceWeigh = 0;
        estimatedPose = pose;
        lastSpread = INF;
        return;
    }

    sensorData_ = readSensors();
    latestImu = imuNow;

    lemlib::Pose delta = pose - prevPose;
    double th = toRad(prevPose.theta + (pose.theta - prevPose.theta) / 2.0); // midpoint heading
    double forward = delta.x * std::sin(th) + delta.y * std::cos(th);
    double right   = delta.x * std::cos(th) - delta.y * std::sin(th);
    moveSinceWeigh += std::hypot(forward, right);
    turnSinceWeigh += std::fabs(latestImu - prevImu);
    update(forward, right, latestImu);

    bool moved = !weighedOnce || moveSinceWeigh >= WEIGH_MIN_MOVE || turnSinceWeigh >= WEIGH_MIN_TURN;
    if (moved && sensorData_.updated) {
        estimatedPose = weigh(getField());
        weighedOnce = true;
        moveSinceWeigh = turnSinceWeigh = 0;
    } else {
        estimatedPose = estimate(latestImu);
    }

    lemlib::Pose correction = applyCorrection(estimatedPose);
    prevPose = lemlib::Pose(pose.x + correction.x, pose.y + correction.y, pose.theta + correction.theta);

    writeTelemetry();

    if (effectiveParticleCount() < particles_.size() * RESAMPLE_ESS_FRACTION) resample();
}

void MCL::writeTelemetry() {
    if (!telemetryLog || !telemetryLog->isOpen()) return;
    fieldRepresentation& field = getField();
    const auto& sensors = field.getSensors();
    lemlib::Pose pose = angledOdom::getPose();

    telemetryLog->beginFrame(pose.x, pose.y, pose.theta, particles_.size());
    for (size_t i = 0; i < sensors.size(); i++) {
        vector3 offset = sensors[i].getCenterOffset();
        telemetryLog->writeSensor(readingAt(sensorData_, i), offset.x(), offset.y(), sensors[i].getDefaultAngle());
    }
    for (const auto& p : particles_) {
        double heading = p.getHeading(latestImu);
        fieldRepresentation::distanceSensorDistances sim = field.simulateCast(p.getPosition(), heading);
        double simulated[4];
        for (size_t i = 0; i < 4; i++) simulated[i] = readingAt(sim, i);
        telemetryLog->writeParticle(p.getX(), p.getY(), heading, p.getWeight(), simulated, 4);
    }
    telemetryLog->endFrame();
}

void MCL::update(double forward, double right, double imuHeading) {
    double dist = std::hypot(forward, right);
    double turned = std::fabs(imuHeading - prevImu);
    prevImu = imuHeading;

    double posStd = MOTION_POS_BASE + MOTION_POS_PER_INCH * dist;
    double headingStd = MOTION_HEADING_BASE + MOTION_HEADING_PER_DEG * turned;

    for (auto& p : particles_) {
        double offset = offsetOf(p, imuHeading) + gaussian(headingStd);
        p.setHeadingOffset(offset);

        double theta = toRad(imuHeading + offset);
        double dx = forward * std::sin(theta) + right * std::cos(theta);
        double dy = forward * std::cos(theta) - right * std::sin(theta);
        p.setPosition(p.getX() + dx + gaussian(posStd), p.getY() + dy + gaussian(posStd));
    }
}

lemlib::Pose MCL::weigh(fieldRepresentation& field) {
    double imuHeading = latestImu;
    if (particles_.empty() || !sensorData_.updated) return estimate(imuHeading);

    const auto& z = sensorData_;
    int informative = (z.frontDistance != READING_INVALID) + (z.rightDistance != READING_INVALID)
                    + (z.backDistance != READING_INVALID) + (z.leftDistance != READING_INVALID);
    size_t n = particles_.size();

    static std::vector<double> logW;
    logW.assign(n, NEG_INF);
    double maxLogW = NEG_INF;
    double likelihoodSum = 0;

    for (size_t i = 0; i < n; i++) {
        particle& p = particles_[i];
        if (outOfField(p) || field.insideObstacle(p.getX(), p.getY()) || !(p.getWeight() > 0)) continue;

        fieldRepresentation::distanceSensorDistances sim =
            field.simulateCast(p.getPosition(), p.getHeading(imuHeading));

        double ll = sensorLogLikelihood(sim.frontDistance, sim.frontCos, z.frontDistance)
                  + sensorLogLikelihood(sim.rightDistance, sim.rightCos, z.rightDistance)
                  + sensorLogLikelihood(sim.backDistance,  sim.backCos,  z.backDistance)
                  + sensorLogLikelihood(sim.leftDistance,  sim.leftCos,  z.leftDistance);
        if (!std::isfinite(ll)) continue;

        likelihoodSum += std::exp(ll / informative);
        logW[i] = std::log(p.getWeight()) + ll;
        maxLogW = std::max(maxLogW, logW[i]);
    }

    double wAvg = likelihoodSum / n;
    if (wSlow <= 0) { wSlow = wFast = wAvg; }
    else {
        wSlow += ALPHA_SLOW * (wAvg - wSlow);
        wFast += ALPHA_FAST * (wAvg - wFast);
    }

    if (maxLogW == NEG_INF) {
        lemlib::Pose current = angledOdom::getPose();
        if (finite(current)) {
            scatter(particles_, current.x, current.y, current.theta - imuHeading, INIT_POS_STD * 2, INIT_HEADING_STD);
        }
        lastSpread = INF;
        return current;
    }

    double total = 0;
    for (size_t i = 0; i < n; i++) total += std::exp(logW[i] - maxLogW);
    for (size_t i = 0; i < n; i++) particles_[i].setWeight(std::exp(logW[i] - maxLogW) / total);

    return estimate(imuHeading);
}

lemlib::Pose MCL::estimate(double imuHeading) {
    lemlib::Pose current = angledOdom::getPose();

    double sumW = 0, mx = 0, my = 0, sinSum = 0, cosSum = 0;
    for (const auto& p : particles_) {
        double w = p.getWeight();
        if (!(w > 0)) continue;
        sumW += w;
        mx += w * p.getX();
        my += w * p.getY();
        double off = toRad(offsetOf(p, imuHeading));
        sinSum += w * std::sin(off);
        cosSum += w * std::cos(off);
    }
    if (!(sumW > 0)) {
        lastSpread = INF;
        return current;
    }
    mx /= sumW;
    my /= sumW;

    double var = 0;
    for (const auto& p : particles_) {
        double w = p.getWeight();
        if (!(w > 0)) continue;
        double ex = p.getX() - mx, ey = p.getY() - my;
        var += w * (ex * ex + ey * ey);
    }
    lastSpread = std::sqrt(var / sumW);

    double estTheta = imuHeading + toDeg(std::atan2(sinSum, cosSum));
    estTheta = current.theta + wrap180(estTheta - current.theta);

    return lemlib::Pose(mx, my, estTheta);
}

double MCL::effectiveParticleCount() {
    double sumW = 0, sumW2 = 0;
    for (const auto& p : particles_) {
        sumW += p.getWeight();
        sumW2 += p.getWeight() * p.getWeight();
    }
    if (sumW2 <= 0) return 0;
    return sumW * sumW / sumW2;
}

fieldRepresentation::distanceSensorDistances MCL::readSensors() {
    fieldRepresentation::distanceSensorDistances data;
    data.frontDistance = readInches(distanceFront);
    data.rightDistance = readInches(distanceRight);
    data.backDistance  = readInches(distanceBack);
    data.leftDistance  = readInches(distanceLeft);
    data.updated = data.frontDistance != READING_INVALID || data.rightDistance != READING_INVALID
                || data.backDistance != READING_INVALID  || data.leftDistance != READING_INVALID;
    return data;
}

void MCL::resample() {
    size_t n = particles_.size();
    if (n == 0) return;

    double sumW = 0;
    for (const auto& p : particles_) sumW += p.getWeight();
    if (sumW <= 0) {
        for (auto& p : particles_) p.setWeight(1.0 / n);
        return;
    }

    double step = sumW / n;
    double pointer = uniform(0.0, step);
    double cumulative = particles_[0].getWeight();
    size_t i = 0;

    std::vector<particle> out;
    out.reserve(n);
    for (size_t m = 0; m < n; m++) {
        double target = pointer + m * step;
        while (target > cumulative && i < n - 1) {
            i++;
            cumulative += particles_[i].getWeight();
        }
        out.push_back(particles_[i]);
    }

    double randomFrac = wSlow > 0 ? std::clamp(1.0 - wFast / wSlow, 0.0, MAX_RANDOM_FRACTION) : 0.0;
    double imuHeading = latestImu;
    double meanOffset = estimatedPose.theta - imuHeading;
    lemlib::Pose odom = angledOdom::getPose();

    for (auto& p : out) {
        if (randomFrac > 0 && uniform(0.0, 1.0) < randomFrac) {
            double r = RECOVERY_RADIUS * std::sqrt(uniform(0.0, 1.0));
            double a = uniform(0.0, 2 * PI);
            p.setPosition(odom.x + r * std::cos(a), odom.y + r * std::sin(a));
            p.setHeadingOffset(meanOffset + gaussian(INIT_HEADING_STD));
        } else {
            double off = offsetOf(p, imuHeading);
            p.setPosition(p.getX() + gaussian(ROUGHEN_POS_STD), p.getY() + gaussian(ROUGHEN_POS_STD));
            p.setHeadingOffset(off + gaussian(ROUGHEN_HEADING_STD));
        }
        p.setWeight(1.0 / n);
    }

    particles_ = std::move(out);
}
