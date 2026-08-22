#pragma once

#include <string>
#include <vector>


struct SteeringEvent {
    double start;
    double end;
    double angle;
};


struct Scenario {
    std::string name;
    std::string outputFile;

    double initialVelocity;
    double acceleration;
    double steeringAngle;
    double wheelbase;
    double duration;
    double dt;

    std::string integrationMethod;

    std::vector<SteeringEvent> steeringSchedule;


    // ==================================================
    // Optional cruise-control configuration
    // ==================================================

    bool cruiseControlEnabled;

    double targetVelocity;

    double cruiseKp;

    double cruiseKi;

    double minimumAcceleration;

    double maximumAcceleration;
};