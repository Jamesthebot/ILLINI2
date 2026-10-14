#include "angled_odom.h"
#include <cmath>

namespace angledOdom {

static constexpr float INV_SQRT2 = 0.70710678118f;
static constexpr float DEG_TO_RAD = M_PI / 180.0f;
static constexpr float RAD_TO_DEG = 180.0f / M_PI;

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

static pros::Task* s_task = nullptr;

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

static void update() {
    int raw1 = s_enc1->get_value();
    int raw2 = s_enc2->get_value();

    float d1 = (raw1 - s_lastEnc1) * s_inchesPerTick * s_enc1Scale;
    float d2 = (raw2 - s_lastEnc2) * s_inchesPerTick * s_enc2Scale;
    s_lastEnc1 = raw1;
    s_lastEnc2 = raw2;

    float deltaForward = (d1 + d2) * INV_SQRT2;
    float deltaStrafe  = (d1 - d2) * INV_SQRT2;

    float imuRad = s_imu->get_rotation() * DEG_TO_RAD;
    float deltaTheta = imuRad - s_lastImuRad;
    s_lastImuRad = imuRad;

    float fwdOffset = (s_enc1Offset + s_enc2Offset) * INV_SQRT2;
    float strOffset = (s_enc1Offset - s_enc2Offset) * INV_SQRT2;

    deltaForward -= fwdOffset * deltaTheta;
    deltaStrafe  -= strOffset * deltaTheta;

    float localX, localY;
    if (std::fabs(deltaTheta) < 1e-6f) {
        localX = deltaStrafe;
        localY = deltaForward;
    } else {
        float halfAngle = deltaTheta / 2.0f;
        float sinRatio = 2.0f * std::sin(halfAngle);
        localX = sinRatio * (deltaStrafe / deltaTheta);
        localY = sinRatio * (deltaForward / deltaTheta);
    }

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
    while (true) {
        update();
        s_chassis->setPose(s_x, s_y, s_theta * RAD_TO_DEG, false);
        pros::delay(10);
    }
}

void start(float x, float y, float theta) {
    s_x = x;
    s_y = y;
    s_theta = theta * DEG_TO_RAD;
    s_lastEnc1 = s_enc1->get_value();
    s_lastEnc2 = s_enc2->get_value();
    s_lastImuRad = s_imu->get_rotation() * DEG_TO_RAD;

    s_chassis->setPose(x, y, theta, false);
    s_task = new pros::Task(taskFn, nullptr, "angledOdom");
}

void stop() {
    if (s_task != nullptr) {
        s_task->remove();
        delete s_task;
        s_task = nullptr;
    }
}

void setPose(float x, float y, float theta) {
    s_x = x;
    s_y = y;
    s_theta = theta * DEG_TO_RAD;
    s_chassis->setPose(x, y, theta, false);
}

lemlib::Pose getPose() {
    return {s_x, s_y, s_theta * RAD_TO_DEG};
}

}