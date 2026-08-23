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


    bool cruiseControlEnabled;

    double targetVelocity;

    double cruiseKp;
    double cruiseKi;

    double minimumAcceleration;
    double maximumAcceleration;

    bool adaptiveCruiseControlEnabled;

    double leadVehicleInitialDistance;

    double leadVehicleInitialVelocity;

    double leadVehicleBrakeStart;
    double leadVehicleBrakeEnd;
    double leadVehicleBrakeAcceleration;

    double desiredTimeHeadway;
    double minimumFollowingDistance;

    double accGapKp;
    double accRelativeVelocityKp;
};