#pragma once

#include "scenario.hpp"

double getSteeringAngle(
    const Scenario& scenario,
    double currentTime
);

void runSimulation(
    const Scenario& scenario
);