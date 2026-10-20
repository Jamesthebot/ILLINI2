#pragma once
#include <iostream>
#include <vector>
#include "api.h"
#include "utilities.hpp"
#include "robot.h"

using namespace std;
using namespace utilities;

class fieldRepresentation {
    public:
        struct distanceSensorDistances {
            double frontDistance;
            double leftDistance;
            double rightDistance;
            double backDistance;
            bool updated;
            double frontCos = 1;
            double leftCos = 1;
            double rightCos = 1;
            double backCos = 1;
        };

        static constexpr double maxSensorRange = 78.74;
        static constexpr double fieldHalfWidth = 70.1968;

    private:
        struct fieldObject {
            string type;
            vector3 min;
            vector3 max;
            bool interior = false;
        };

        double hitDistance(const fieldObject& object, vector3 sensorPos, vector3 sensorDir, double closestSoFar, int& hitAxis); //return -1 if not hit or not closer than closestSoFar
        double closestHit(const vector<fieldObject>& candidates, vector3 sensorPos, vector3 sensorDir, double& hitCos); //return -1 if nothing in range

        static vector<fieldObject> fieldObjects();
        static vector<distanceSensor> distanceSensors();

        vector<fieldObject> objects_;
        vector<distanceSensor> sensors_;
        vector<vector<fieldObject>> sensorCandidates_;

    public:
        fieldRepresentation(); // uses distanceSensors() and fieldObjects()
        fieldRepresentation(vector<distanceSensor> sensors); // uses fieldObjects()
        fieldRepresentation(vector<distanceSensor> sensors, vector<fieldObject> objects);
        distanceSensorDistances simulateCast(vector3 particlePos, double particleHeading);
        bool insideObstacle(double x, double y) const;
        const vector<distanceSensor>& getSensors() const { return sensors_; }
        vector3 getMax(const fieldObject& object) { return object.max; }
        vector3 getMin(const fieldObject& object) { return object.min; }
};
