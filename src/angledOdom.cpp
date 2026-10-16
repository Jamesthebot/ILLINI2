#include "angled_odom.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <mutex>

namespace angledOdom {

static constexpr float INV_SQRT2 = 0.70710678118f;
static constexpr float DEG_TO_RAD = M_PI / 180.0f;
static constexpr float RAD_TO_DEG = 180.0f / M_PI;
static constexpr float MAX_WHEEL_SPEED = 150.0f;
static constexpr float MAX_TURN_RATE = 2000.0f * DEG_TO_RAD;
static constexpr float MIN_ELAPSED = 0.01f;

static pros::adi::Encoder* s_enc1 = nullptr;
static pros::adi::Encoder* s_enc2 = nullptr;
static pros::IMU* s_imu = nullptr;
static lemlib::Chassis* s_chassis = nullptr;

static float s_inchesPerTick = 0;
static float s_enc1Offset = 0;
static float s_enc2Offset = 0;
static float s_enc1Scale = 1.0f;
static float s_enc2Scale = 1.0f;

// State
static float s_x = 0;
static float s_y = 0;
static float s_theta = 0;

static int s_lastEnc1 = 0;
static int s_lastEnc2 = 0;
static float s_lastImuRad = 0;
static uint32_t s_lastTime = 0;

static pros::Task* s_task = nullptr;
static lemlib::Pose s_published(0, 0, 0);

static pros::Mutex& poseMutex() {
    static pros::Mutex m;
    return m;
}

static void publish() {
    s_published = lemlib::Pose(s_x, s_y, s_theta * RAD_TO_DEG);
    if (s_chassis != nullptr) s_chassis->setPose(s_published, false);
}

static bool changedExternally(const lemlib::Pose& current) {
    constexpr float tolerance = 1e-3f;
    return std::fabs(current.x - s_published.x) > tolerance || std::fabs(current.y - s_published.y) > tolerance
        || std::fabs(current.theta - s_published.theta) > tolerance;
}

void configure(pros::adi::Encoder* enc1, pros::adi::Encoder* enc2,
               pros::IMU* imu, lemlib::Chassis* chassis,
               float wheelDiameter, float enc1Offset, float enc2Offset,
               float enc1Scale, float enc2Scale,
               float ticksPerRotation) {
    s_enc1 = enc1;
    s_enc2 = enc2;
    s_imu = imu;
    s_chassis = chassis;
    s_inchesPerTick = (wheelDiameter * M_PI) / ticksPerRotation;
    s_enc1Offset = enc1Offset;
    s_enc2Offset = enc2Offset;
    s_enc1Scale = enc1Scale;
    s_enc2Scale = enc2Scale;
}

static bool measure(float& localX, float& localY, float& deltaTheta) {
    int32_t raw1 = s_enc1->get_value();
    int32_t raw2 = s_enc2->get_value();
    double rotation = s_imu->get_rotation();
    if (raw1 == PROS_ERR || raw2 == PROS_ERR || !std::isfinite(rotation)) return false;

    uint32_t now = pros::millis();
    float elapsed = std::max((now - s_lastTime) / 1000.0f, MIN_ELAPSED);
    s_lastTime = now;

    float d1 = static_cast<float>(static_cast<int64_t>(raw1) - s_lastEnc1) * s_inchesPerTick * s_enc1Scale;
    float d2 = static_cast<float>(static_cast<int64_t>(raw2) - s_lastEnc2) * s_inchesPerTick * s_enc2Scale;
    s_lastEnc1 = raw1;
    s_lastEnc2 = raw2;
    float maxTravel = MAX_WHEEL_SPEED * elapsed;
    if (!(std::fabs(d1) <= maxTravel && std::fabs(d2) <= maxTravel)) d1 = d2 = 0;

    float deltaForward = (d1 + d2) * INV_SQRT2;
    float deltaStrafe  = (d1 - d2) * INV_SQRT2;

    float imuRad = rotation * DEG_TO_RAD;
    deltaTheta = imuRad - s_lastImuRad;
    s_lastImuRad = imuRad;
    if (!(std::fabs(deltaTheta) <= MAX_TURN_RATE * elapsed)) deltaTheta = 0;

    float fwdOffset = (s_enc1Offset + s_enc2Offset) * INV_SQRT2;
    float strOffset = (s_enc1Offset - s_enc2Offset) * INV_SQRT2;

    deltaForward -= fwdOffset * deltaTheta;
    deltaStrafe  -= strOffset * deltaTheta;

    if (std::fabs(deltaTheta) < 1e-6f) {
        localX = deltaStrafe;
        localY = deltaForward;
    } else {
        float halfAngle = deltaTheta / 2.0f;
        float sinRatio = 2.0f * std::sin(halfAngle);
        localX = sinRatio * (deltaStrafe / deltaTheta);
        localY = sinRatio * (deltaForward / deltaTheta);
    }
    return true;
}

static void integrate(float localX, float localY, float deltaTheta) {
    float avgTheta = s_theta + deltaTheta / 2.0f;
    float cosA = std::cos(avgTheta);
    float sinA = std::sin(avgTheta);

    s_x += localY * sinA + localX * cosA;
    s_y += localY * cosA - localX * sinA;
    //s_x += localY * cosA - localX * sinA;
    //s_y += -1*(localY * sinA + localX * cosA);
    s_theta += deltaTheta;
}

static void taskFn(void*) {
    uint32_t now = pros::millis();
    while (true) {
        float localX, localY, deltaTheta;
        bool measured = measure(localX, localY, deltaTheta);
        {
            std::lock_guard<pros::Mutex> lock(poseMutex());
            lemlib::Pose current = s_chassis->getPose();
            if (changedExternally(current)) {
                s_x = current.x;
                s_y = current.y;
                s_theta = current.theta * DEG_TO_RAD;
            }
            if (measured) integrate(localX, localY, deltaTheta);
            publish();
        }
        pros::Task::delay_until(&now, 10);
    }
}

void start(float x, float y, float theta) {
    if (s_enc1 == nullptr || s_enc2 == nullptr || s_imu == nullptr || s_chassis == nullptr || s_task != nullptr) return;
    {
        std::lock_guard<pros::Mutex> lock(poseMutex());
        s_x = x;
        s_y = y;
        s_theta = theta * DEG_TO_RAD;
        s_lastEnc1 = s_enc1->get_value();
        s_lastEnc2 = s_enc2->get_value();
        s_lastImuRad = s_imu->get_rotation() * DEG_TO_RAD;
        s_lastTime = pros::millis();
        publish();
    }
    s_task = new pros::Task(taskFn, nullptr, "angledOdom");
}

void stop() {
    std::lock_guard<pros::Mutex> lock(poseMutex());
    if (s_task != nullptr) {
        s_task->remove();
        delete s_task;
        s_task = nullptr;
    }
}

void setPose(float x, float y, float theta) {
    std::lock_guard<pros::Mutex> lock(poseMutex());
    s_x = x;
    s_y = y;
    s_theta = theta * DEG_TO_RAD;
    publish();
}

void offsetPose(float dx, float dy, float dtheta) {
    std::lock_guard<pros::Mutex> lock(poseMutex());
    s_x += dx;
    s_y += dy;
    s_theta += dtheta * DEG_TO_RAD;
    publish();
}

lemlib::Pose getPose() {
    std::lock_guard<pros::Mutex> lock(poseMutex());
    return {s_x, s_y, s_theta * RAD_TO_DEG};
}

}