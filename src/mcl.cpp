#include "mcl.h"
#include "fieldRepresentation.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <vector>

// ============================================================================
// Tuning
// ============================================================================
namespace {

constexpr double INIT_POS_STD = 1.5;
constexpr double INIT_HEADING_STD = 2.0; 

constexpr double MOTION_POS_BASE = 0.02;
constexpr double MOTION_POS_PER_INCH = 0.05;
constexpr double MOTION_HEADING_BASE = 0.02;
constexpr double MOTION_HEADING_PER_DEG = 0.02;

constexpr double SENSOR_MAX_RANGE = 78.0;        
constexpr double SENSOR_SIGMA_BASE = 0.5; 
constexpr double SENSOR_SIGMA_PER_INCH = 0.05; 
constexpr double SENSOR_OUTLIER_PROB = 0.1;      
constexpr int32_t SENSOR_NO_OBJECT_MM = 9999;    
constexpr int32_t SENSOR_CONFIDENCE_MIN_MM = 200;
constexpr int32_t SENSOR_MIN_CONFIDENCE = 30;    

constexpr double FIELD_MIN = 0.0;
constexpr double FIELD_MAX = 144.0;
constexpr double FIELD_MARGIN = 2.0;             

constexpr double POS_CORRECTION_GAIN = 0.3;
constexpr double HEADING_CORRECTION_GAIN = 0.1;
constexpr bool   CORRECT_HEADING = true;
constexpr double MAX_CORRECTION = 6.0;           
constexpr double MAX_HEADING_CORRECTION = 5.0;  
constexpr int    FORCE_AFTER_REJECTIONS = 10;   
constexpr double MAX_SPREAD_TO_CORRECT = 3.0;    

constexpr double RESAMPLE_ESS_FRACTION = 0.5;
constexpr double ROUGHEN_POS_STD = 0.1;         
constexpr double ROUGHEN_HEADING_STD = 0.1;    

constexpr double ALPHA_SLOW = 0.01;
constexpr double ALPHA_FAST = 0.2;
constexpr double MAX_RANDOM_FRACTION = 0.1;

constexpr uint32_t LOOP_MS = 100;     

constexpr double PI = 3.14159265358979323846;
constexpr double SQRT_2PI = 2.50662827463100050242;
constexpr double NEG_INF = -std::numeric_limits<double>::infinity();

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

bool started = false;
double latestImu = 0;
double prevImu = 0;
double lastSpread = std::numeric_limits<double>::infinity();
double wSlow = 0, wFast = 0;
int rejectedTicks = 0;

double offsetOf(particle& p, double imuHeading) { return p.getHeading(imuHeading) - imuHeading; }

bool outOfField(particle& p) {
    double lo = FIELD_MIN + FIELD_MARGIN, hi = FIELD_MAX - FIELD_MARGIN;
    return p.getX() < lo || p.getX() > hi || p.getY() < lo || p.getY() > hi;
}

void scatter(std::vector<particle>& ps, double x, double y, double offset, double posStd, double headingStd) {
    for (auto& p : ps) {
        p.setPosition(x + gaussian(posStd), y + gaussian(posStd));
        p.setHeadingOffset(offset + gaussian(headingStd));
        p.setWeight(1.0 / ps.size());
    }
}

int32_t readOneMm(pros::Distance& sensor) {
    int32_t mm = sensor.get();
    if (mm == PROS_ERR || mm <= 0 || mm >= SENSOR_NO_OBJECT_MM) return -1;
    if (mm > SENSOR_CONFIDENCE_MIN_MM) {
        int32_t conf = sensor.get_confidence();
        if (conf == PROS_ERR || conf < SENSOR_MIN_CONFIDENCE) return -1;
    }
    return mm;
}

double sensorLogLikelihood(double simulated, double measured) {
    if (measured < 0) return 0.0;
    const double outlier = SENSOR_OUTLIER_PROB / SENSOR_MAX_RANGE;
    if (simulated > SENSOR_MAX_RANGE) return std::log(outlier); // particle says nothing should be in range
    double sigma = SENSOR_SIGMA_BASE + SENSOR_SIGMA_PER_INCH * measured;
    double e = simulated - measured;
    double gauss = std::exp(-e * e / (2 * sigma * sigma)) / (sigma * SQRT_2PI);
    return std::log((1 - SENSOR_OUTLIER_PROB) * gauss + outlier);
}

void applyCorrection(const lemlib::Pose& est) {
    if (lastSpread > MAX_SPREAD_TO_CORRECT) { rejectedTicks = 0; return; }
    lemlib::Pose pose = chassis.getPose();
    double cx = est.x - pose.x;
    double cy = est.y - pose.y;
    double ct = CORRECT_HEADING ? wrap180(est.theta - pose.theta) : 0.0;

    bool tooBig = std::hypot(cx, cy) > MAX_CORRECTION || std::fabs(ct) > MAX_HEADING_CORRECTION;
    if (tooBig && ++rejectedTicks < FORCE_AFTER_REJECTIONS) return; // probably a glitch; persistent = real
    rejectedTicks = 0;

    chassis.setPose(pose.x + POS_CORRECTION_GAIN * cx,
                    pose.y + POS_CORRECTION_GAIN * cy,
                    pose.theta + HEADING_CORRECTION_GAIN * ct);
}

}

double startingX = 36;
double startingY = 36;

lemlib::Pose estimatedPose(0, 0, 0);

fieldRepresentation::distanceSensorDistances MCL::sensorData_;
std::vector<particle> MCL::particles_;
lemlib::Pose MCL::prevPose(0, 0, 0);

fieldRepresentation& MCL::getField() {
    static fieldRepresentation field;
}

// ============================================================================
// MCL
// ============================================================================
void MCL::init(int numParticles) {
    if (started || numParticles <= 0) return; 
    started = true;

    latestImu = prevImu = imu.get_rotation(); 
    double startOffset = chassis.getPose().theta - latestImu;

    particles_.assign(numParticles, particle());
    scatter(particles_, startingX, startingY, startOffset, INIT_POS_STD, INIT_HEADING_STD);
    wSlow = wFast = 0;
    estimatedPose = chassis.getPose();

    pros::Task mclTask([] {
        prevPose = chassis.getPose();
        uint32_t now = pros::millis();
        while (true) {
            sensorData_ = readSensors();
            latestImu = imu.get_rotation();

            lemlib::Pose pose = chassis.getPose();
            lemlib::Pose delta = pose - prevPose;
            double th = toRad(prevPose.theta + delta.theta / 2.0); // midpoint heading
            double forward = delta.x * std::sin(th) + delta.y * std::cos(th);
            double right   = delta.x * std::cos(th) - delta.y * std::sin(th);
            update(forward, right, latestImu);

            estimatedPose = weigh(getField());
            applyCorrection(estimatedPose);
            prevPose = chassis.getPose(); 

            if (effectiveParticleCount() < particles_.size() * RESAMPLE_ESS_FRACTION) resample();
            pros::Task::delay_until(&now, LOOP_MS);
        }
    });
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
    lemlib::Pose current = chassis.getPose();
    if (particles_.empty() || !sensorData_.updated) {
        lastSpread = std::numeric_limits<double>::infinity();
        return current;
    }

    const auto& z = sensorData_;
    int validSensors = (z.frontDistance >= 0) + (z.rightDistance >= 0) + (z.backDistance >= 0) + (z.leftDistance >= 0);
    double imuHeading = latestImu;
    size_t n = particles_.size();

    static std::vector<double> logW;
    logW.assign(n, NEG_INF);
    double maxLogW = NEG_INF;
    double likelihoodSum = 0;

    for (size_t i = 0; i < n; i++) {
        particle& p = particles_[i];
        if (outOfField(p) || p.getWeight() <= 0) continue;

        fieldRepresentation::distanceSensorDistances sim =
            field.simulateCast(p.getPosition(), p.getHeading(imuHeading));

        double ll = sensorLogLikelihood(sim.frontDistance, z.frontDistance)
                  + sensorLogLikelihood(sim.rightDistance, z.rightDistance)
                  + sensorLogLikelihood(sim.backDistance,  z.backDistance)
                  + sensorLogLikelihood(sim.leftDistance,  z.leftDistance);

        likelihoodSum += std::exp(ll / validSensors);
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
        scatter(particles_, current.x, current.y, current.theta - imuHeading, INIT_POS_STD * 2, INIT_HEADING_STD);
        lastSpread = std::numeric_limits<double>::infinity();
        return current;
    }

    double total = 0;
    for (size_t i = 0; i < n; i++) total += std::exp(logW[i] - maxLogW);

    double mx = 0, my = 0, sinSum = 0, cosSum = 0;
    for (size_t i = 0; i < n; i++) {
        double w = std::exp(logW[i] - maxLogW) / total;
        particle& p = particles_[i];
        p.setWeight(w);
        mx += w * p.getX();
        my += w * p.getY();
        double off = toRad(offsetOf(p, imuHeading));
        sinSum += w * std::sin(off);
        cosSum += w * std::cos(off);
    }

    double var = 0;
    for (auto& p : particles_) {
        double ex = p.getX() - mx, ey = p.getY() - my;
        var += p.getWeight() * (ex * ex + ey * ey);
    }
    lastSpread = std::sqrt(var);

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
    int32_t front = readOneMm(distanceFront);
    int32_t right = readOneMm(distanceRight);
    int32_t back  = readOneMm(distanceBack);
    int32_t left  = readOneMm(distanceLeft);

    auto toIn = [](int32_t mm) {
        if (mm < 0) return -1.0;
        double in = mmToInches(mm);
        return in > SENSOR_MAX_RANGE ? -1.0 : in;
    };

    fieldRepresentation::distanceSensorDistances data;
    data.frontDistance = toIn(front);
    data.rightDistance = toIn(right);
    data.backDistance  = toIn(back);
    data.leftDistance  = toIn(left);
    data.updated = data.frontDistance >= 0 || data.rightDistance >= 0
                || data.backDistance >= 0  || data.leftDistance >= 0;
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

    for (auto& p : out) {
        if (randomFrac > 0 && uniform(0.0, 1.0) < randomFrac) {
            p.setPosition(uniform(FIELD_MIN + FIELD_MARGIN, FIELD_MAX - FIELD_MARGIN),
                          uniform(FIELD_MIN + FIELD_MARGIN, FIELD_MAX - FIELD_MARGIN));
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