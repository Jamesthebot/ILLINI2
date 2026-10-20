#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <mutex>
#include <random>
#include <thread>
#include <utility>
#include <vector>

#include "api.h"
#include "robot.h"
#include "angled_odom.h"
#include "fieldRepresentation.h"
#include "mcl.h"
#include "telemetry.h"

pros::Imu imu(11);
pros::Distance distanceFront(9);
pros::Distance distanceLeft(8);
pros::Distance distanceRight(20);
pros::Distance distanceBack(19);

namespace {

constexpr int NUM_PARTICLES = 300;
constexpr int NUM_SENSORS = 4;
constexpr uint32_t PHYSICS_MS = 10;
constexpr uint32_t MCL_MS = 100;
constexpr double MAX_SIM_SECONDS = 60.0;

constexpr double DRIVE_SPEED = 30.0;
constexpr double TURN_RATE = 180.0;
constexpr double ARRIVE_RADIUS = 0.5;
constexpr double HEADING_TOLERANCE = 1.5;

constexpr double ODOM_SCALE_ERROR = 0.02;
constexpr double ODOM_NOISE_PER_INCH = 0.01;
constexpr double LATERAL_SLIP_PER_INCH = 0.01;
constexpr double IMU_DRIFT_DEG_PER_S = 0.1;

constexpr double PUSH_TIME_S = 12.0;
constexpr double PUSH_X = 0.0;
constexpr double PUSH_Y = 4.0;

constexpr double GRAZING_COS = 0.2;
constexpr double SHORT_READING_P = 0.03;
constexpr double LOW_CONFIDENCE_P = 0.03;

constexpr double PI = 3.14159265358979323846;

struct pose2 { double x, y, theta; };

std::mt19937 rng(1234);
double gaussian(double stddev) { return stddev > 0 ? std::normal_distribution<double>(0.0, stddev)(rng) : 0.0; }
double uniform(double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(rng); }
double toRad(double d) { return d * PI / 180.0; }
double wrap180(double d) {
    d = std::fmod(d + 180.0, 360.0);
    if (d < 0) d += 360.0;
    return d - 180.0;
}

uint32_t simMs = 0;
pose2 truth{-34, -34, 90};
pose2 odom = truth;
double odomHeadingOffset = 0;
pose2 rawOdom = truth;
double rawHeadingOffset = 0;
std::map<int, std::pair<int32_t, int32_t>> sensorReadings;

double imuRotation() { return truth.theta + IMU_DRIFT_DEG_PER_S * simMs / 1000.0; }

std::mutex stepMutex;
std::condition_variable stepCv;
bool mclIdle = false;
bool mclGo = false;

void waitForMclIdle() {
    std::unique_lock<std::mutex> lock(stepMutex);
    stepCv.wait(lock, [] { return mclIdle; });
}

void runMclStep() {
    {
        std::lock_guard<std::mutex> lock(stepMutex);
        mclIdle = false;
        mclGo = true;
    }
    stepCv.notify_all();
    waitForMclIdle();
}

std::pair<int32_t, int32_t> fakeReading(double inches, double hitCos) {
    if (inches < 0 || hitCos < GRAZING_COS) return {9999, 0};
    double mm = inches * 25.4;
    if (uniform(0, 1) < SHORT_READING_P) mm = uniform(50, mm);
    double accuracy = mm < 200 ? 15.0 : 0.05 * mm;
    mm = std::max(0.0, mm + gaussian(accuracy / 2));
    if (mm > 2000) return {9999, 0};
    int32_t confidence = 63;
    if (mm > 200) confidence = uniform(0, 1) < LOW_CONFIDENCE_P ? 10 : 63 - static_cast<int32_t>(uniform(0, 20));
    return {static_cast<int32_t>(mm), confidence};
}

void updateSensors() {
    static fieldRepresentation field;
    fieldRepresentation::distanceSensorDistances d = field.simulateCast(vector3(truth.x, truth.y, 0), truth.theta);
    sensorReadings[distanceFront.get_port()] = fakeReading(d.frontDistance, d.frontCos);
    sensorReadings[distanceRight.get_port()] = fakeReading(d.rightDistance, d.rightCos);
    sensorReadings[distanceBack.get_port()] = fakeReading(d.backDistance, d.backCos);
    sensorReadings[distanceLeft.get_port()] = fakeReading(d.leftDistance, d.leftCos);
}

void integrate(pose2& p, double headingOffset, double forward, double right, double imuBefore, double imuAfter) {
    p.theta = imuAfter + headingOffset;
    double mid = toRad((imuBefore + imuAfter) / 2 + headingOffset);
    p.x += forward * std::sin(mid) + right * std::cos(mid);
    p.y += forward * std::cos(mid) - right * std::sin(mid);
}

const std::vector<std::pair<double, double>> WAYPOINTS = {
    {34, -34}, {34, 34}, {-34, 34}, {-34, -34},
    {34, -34}, {34, 34}, {-34, 34}, {-34, -34},
};
size_t waypoint = 0;

bool physicsStep(double dt) {
    if (waypoint >= WAYPOINTS.size()) return false;
    double dx = WAYPOINTS[waypoint].first - odom.x;
    double dy = WAYPOINTS[waypoint].second - odom.y;
    double dist = std::hypot(dx, dy);
    if (dist < ARRIVE_RADIUS) {
        waypoint++;
        return true;
    }

    double desired = std::atan2(dx, dy) * 180.0 / PI;
    double error = wrap180(desired - odom.theta);
    double turn = 0, forward = 0;
    if (std::fabs(error) > HEADING_TOLERANCE) {
        turn = std::clamp(error, -TURN_RATE * dt, TURN_RATE * dt);
    } else {
        turn = error;
        forward = std::min(DRIVE_SPEED * dt, dist);
    }

    double imuBefore = imuRotation();
    double slip = gaussian(LATERAL_SLIP_PER_INCH * forward);
    double mid = toRad(truth.theta + turn / 2);
    truth.x += forward * std::sin(mid) + slip * std::cos(mid);
    truth.y += forward * std::cos(mid) - slip * std::sin(mid);
    truth.theta += turn;
    simMs += PHYSICS_MS;
    double imuAfter = imuRotation();

    double measured = forward * (1 + ODOM_SCALE_ERROR) + gaussian(ODOM_NOISE_PER_INCH * forward);
    integrate(odom, odomHeadingOffset, measured, 0, imuBefore, imuAfter);
    integrate(rawOdom, rawHeadingOffset, measured, 0, imuBefore, imuAfter);
    return true;
}

double positionError(const pose2& p) { return std::hypot(p.x - truth.x, p.y - truth.y); }
double headingError(const pose2& p) { return wrap180(p.theta - truth.theta); }

}

uint32_t pros::millis() { return simMs; }
uint64_t pros::micros() { return 1; }

void pros::detail::startTask(std::function<void()> fn) { std::thread(fn).detach(); }

void pros::Task::delay_until(uint32_t* prev, uint32_t delta) {
    *prev += delta;
    std::unique_lock<std::mutex> lock(stepMutex);
    mclIdle = true;
    stepCv.notify_all();
    stepCv.wait(lock, [] { return mclGo; });
    mclGo = false;
}

int32_t pros::Distance::get() { return sensorReadings[port_].first; }
int32_t pros::Distance::get_confidence() { return sensorReadings[port_].second; }
double pros::Imu::get_rotation() { return imuRotation(); }

void angledOdom::setPose(float x, float y, float theta) {
    odom = {x, y, theta};
    odomHeadingOffset = theta - imuRotation();
}

void angledOdom::offsetPose(float dx, float dy, float dtheta) {
    odom.x += dx;
    odom.y += dy;
    odom.theta += dtheta;
    odomHeadingOffset += dtheta;
}

lemlib::Pose angledOdom::getPose() {
    return lemlib::Pose(static_cast<float>(odom.x), static_cast<float>(odom.y), static_cast<float>(odom.theta));
}

int main(int argc, char** argv) {
    const char* path = argc > 1 ? argv[1] : "sim_telemetry.csv";
    telemetry* log = new telemetry(path, NUM_PARTICLES, NUM_SENSORS);
    if (!log->isOpen()) {
        printf("could not open %s\n", path);
        return 1;
    }

    updateSensors();
    MCL::setTelemetry(log);
    MCL::reset(lemlib::Pose(truth.x, truth.y, truth.theta));
    rawHeadingOffset = odomHeadingOffset;
    MCL::init(NUM_PARTICLES);
    waitForMclIdle();

    printf("%6s  %10s %10s  %10s %10s\n", "t (s)", "MCL pos", "raw pos", "MCL head", "raw head");
    double sumMcl = 0, sumRaw = 0, maxMcl = 0, maxRaw = 0;
    int frames = 0;
    bool pushed = false;
    bool moving = true;
    uint32_t nextPrintMs = 1000;
    while (moving && simMs < MAX_SIM_SECONDS * 1000) {
        for (uint32_t t = 0; t < MCL_MS; t += PHYSICS_MS) {
            moving = physicsStep(PHYSICS_MS / 1000.0);
            if (!moving) break;
        }
        if (!pushed && simMs >= PUSH_TIME_S * 1000) {
            truth.x += PUSH_X;
            truth.y += PUSH_Y;
            pushed = true;
            printf("-- robot pushed (%.1f, %.1f) in at t = %.1f s; odometry doesn't see it --\n", PUSH_X, PUSH_Y, simMs / 1000.0);
        }
        updateSensors();
        runMclStep();

        double eMcl = positionError(odom), eRaw = positionError(rawOdom);
        sumMcl += eMcl;
        sumRaw += eRaw;
        maxMcl = std::max(maxMcl, eMcl);
        maxRaw = std::max(maxRaw, eRaw);
        frames++;
        if (simMs >= nextPrintMs) {
            nextPrintMs += 1000;
            printf("%6.1f  %9.2f\" %9.2f\"  %6.2f deg %6.2f deg\n", simMs / 1000.0, eMcl, eRaw, headingError(odom), headingError(rawOdom));
        }
    }

    printf("\n%d frames, %.1f s\n", frames, simMs / 1000.0);
    printf("position error  with MCL: mean %.2f\", max %.2f\", final %.2f\"\n", sumMcl / frames, maxMcl, positionError(odom));
    printf("position error  odom only: mean %.2f\", max %.2f\", final %.2f\"\n", sumRaw / frames, maxRaw, positionError(rawOdom));
    printf("heading error   with MCL: final %.2f deg; odom only: final %.2f deg\n", headingError(odom), headingError(rawOdom));
    delete log;
    printf("wrote %s\n", path);
    fflush(stdout);
    std::_Exit(0);
}
