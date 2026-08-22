#pragma once
#include "scenario.hpp"

enum class IntegrationMethod {
    Euler,
    RK4
};

struct VehicleState {
    double x;
    double y;
    double velocity;
    double heading;
};

void updateVehicleEuler(
    VehicleState& vehicle,
    const Scenario& scenario,
    double steeringAngle
);

void updateVehicleRK4(
    VehicleState& vehicle,
    const Scenario& scenario,
    double steeringAngle
);

void updateVehicle(
    VehicleState& vehicle,
    const Scenario& scenario,
    double steeringAngle,
    IntegrationMethod method
);