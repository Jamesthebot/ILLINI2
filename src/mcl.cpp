#include "mcl.h"
#include "fieldRepresentation.h"
#include <random>
#include <cmath>
#include <random>

// starting position
double startingX = 36;
double startingY = 36;

// noise
const double noise_position = 0.05;
const double noise_heading = 0.05;

lemlib::Pose estimatedPose(0,0,0);

fieldRepresentation::distanceSensorDistances MCL::sensorData_;
vector<particle> MCL::particles_;
lemlib::Pose MCL::prevPose(0, 0, 0);

fieldRepresentation& MCL::getField() {
    static fieldRepresentation field; // built on first call, after robot.cpp's distance sensors exist
    return field;
}

void MCL::init(int numParticles) {
    particles_.clear();
    particles_.reserve(numParticles);
    // particle heading = imu rotation + headingOffset, so this offset makes particles start at lemlib's heading
    double startOffset = chassis.getPose().theta - imu.get_rotation();
    // TODO: spread particles around a known start pose, headingOffset centred on startOffset
    std::random_device rd;
    std::mt19937 gen(rd());
    double mean = 0;
    double std = noise_position;
    std::normal_distribution<double> posDist(0.0, noise_position);
    std::normal_distribution<double> headingDist(0.0, noise_heading);  // or whatever your heading noise variable is
    for (auto& p : particles_) {
        p.setPosition(startingX + posDist(gen), startingY + posDist(gen));
        p.setHeadingOffset(startOffset + headingDist(gen));
    }
    
    pros::Task mcl_task([=]() {
        prevPose = chassis.getPose();
        while (true) {
            sensorData_ = readSensors();
            double imuHeading = imu.get_rotation(); // continuous, unlike get_heading which wraps at 360

            lemlib::Pose pose = chassis.getPose();
            lemlib::Pose delta = pose - prevPose; // 
            double th = deg2rad(prevPose.theta);
            double forward = delta.x * std::sin(th) + delta.y * std::cos(th);
            double right = delta.x * std::cos(th) - delta.y * std::sin(th);
            update(forward, right, imuHeading);

            estimatedPose = weigh(getField());
            lemlib::Pose correction = estimatedPose - pose;
            chassis.setPose(chassis.getPose() + correction);
            //todo: consider adding some way to smooth correction application to avoid jerkiness and reject overly large corrections

            if (effectiveParticleCount() < particles_.size() / 2.0) resample();
            pros::delay(100); // as low as we can without overloading the vex brain -> tune this
        }
    });
}

void MCL::update(double forward, double right, double imuHeading) {
    for (auto& p : particles_) {
        // TODO: rotate (forward, right) by p.getHeading(imuHeading) into the field frame, move p by it plus noise'
        double theta = deg2rad(p.getHeading(imuHeading));
        
        double deltaX = forward * std::cos(theta) + right * std::sin(theta);
        double deltaY = forward * std::sin(theta) - right * std::cos(theta);

        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::normal_distribution<double> noise(0.0, noise_position);
        
        double newX = p.getX() + deltaX + noise(gen);
        double newY = p.getY() + deltaY + noise(gen);
        p.setPosition(newX, newY);
    }
}

lemlib::Pose MCL::weigh(fieldRepresentation& field) {
    if (!sensorData_.updated || particles_.empty()) {
        return chassis.getPose(); // nothing to compare against, so no correction
    }

    double imuHeading = imu.get_rotation();
    lemlib::Pose estimatedPosition(0, 0, 0);
    double totalWeight = 0;

    for (auto& p : particles_) {
        fieldRepresentation::distanceSensorDistances simDistances =
            field.simulateCast(p.getPosition(), p.getHeading(imuHeading));

        double errFront = simDistances.frontDistance - sensorData_.frontDistance;
        double errRight = simDistances.rightDistance - sensorData_.rightDistance;
        double errBack  = simDistances.backDistance  - sensorData_.backDistance;
        double errLeft  = simDistances.leftDistance  - sensorData_.leftDistance;

        double totalErrorSquared = (errFront * errFront) + (errRight * errRight)
                                 + (errBack * errBack) + (errLeft * errLeft);

        double sigma = 2.0; // inches, tune this
        double weight = std::exp(-totalErrorSquared / (2 * sigma * sigma));
        weight = std::max(weight, 1e-12);

        p.setWeight(weight);
        totalWeight += weight;
    }

    for (auto& p : particles_) {
        double share = p.getWeight() / totalWeight;
        estimatedPosition.x += share * p.getX();
        estimatedPosition.y += share * p.getY();
    }
    estimatedPosition.theta = chassis.getPose().theta;
    return estimatedPosition;
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
    int32_t front = distanceFront.get();
    int32_t right = distanceRight.get();
    int32_t back = distanceBack.get();
    int32_t left = distanceLeft.get();

    fieldRepresentation::distanceSensorDistances data;
    data.frontDistance = mmToInches(front);
    data.rightDistance = mmToInches(right);
    data.backDistance = mmToInches(back);
    data.leftDistance = mmToInches(left);
    data.updated = front != PROS_ERR && right != PROS_ERR && back != PROS_ERR && left != PROS_ERR;
    return data;
}


// Someone should look at this 

void MCL::resample() {
    // TODO
    size_t count = particles_.size();
    if (count == 0 ) return;


    double sumW = 0;
    for (auto it = particles_.begin(); it != particles_.end(); it++) {
        sumW += (*it).getWeight();
    }
    static std::mt19937 rng{std::random_device{}()};
    std::uniform_real_distribution<double> dist(0.0, 1.0 / n);

    vector<particle> newParticles;
    newParticles.reserve(n);

    double step = sumW / n;                 
    double pointer = dist(rng) * sumW;      
    double cumulative = particles_[0].getWeight();
    size_t i = 0;

    for (size_t m = 0; m < n; m++) {
        double target = pointer + m * step;
        while (target > cumulative && i < n - 1) {
            i++;
            cumulative += particles_[i].getWeight();
        }
        newParticles.push_back(particles_[i]);   
    }

    for (auto& p : newParticles) {
        p.setWeight(1.0 / n);
    }

    particles_ = std::move(newParticles);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> posDist(0.0, noise_position);
    std::normal_distribution<double> headingDist(0.0, noise_heading);  // or whatever your heading noise variable is
    for (auto& p : particles_) {
        p.setPosition(estimatedPose.x + posDist(gen), estimatedPose.y + posDist(gen));
        p.setHeadingOffset(0 + headingDist(gen));
    }
}

