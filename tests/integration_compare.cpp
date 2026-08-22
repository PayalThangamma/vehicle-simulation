#include "vehicle_model.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>


static VehicleState runSimulation(
    const Scenario& scenario,
    IntegrationMethod method
) {
    VehicleState vehicle{
        0.0,
        0.0,
        scenario.initialVelocity,
        0.0
    };

    const long long numberOfSteps =
        std::llround(
            scenario.duration
            /
            scenario.dt
        );

    for (
        long long step = 0;
        step < numberOfSteps;
        ++step
    ) {
        updateVehicle(
            vehicle,
            scenario,
            scenario.steeringAngle,
            method
        );
    }

    return vehicle;
}


int main() {
    Scenario scenario{};

    scenario.name =
        "Integration Method Comparison";

    scenario.outputFile =
        "";

    scenario.initialVelocity =
        10.0;

    scenario.acceleration =
        0.0;

    scenario.steeringAngle =
        0.1;

    scenario.wheelbase =
        2.7;

    scenario.duration =
        5.0;

    scenario.dt =
        0.1;

    scenario.integrationMethod =
        "Euler";

    scenario.cruiseControlEnabled =
        false;

    scenario.targetVelocity =
        scenario.initialVelocity;

    scenario.cruiseKp =
        0.0;

    scenario.cruiseKi =
        0.0;

    scenario.minimumAcceleration =
        -6.0;

    scenario.maximumAcceleration =
        3.0;


    const VehicleState eulerResult =
        runSimulation(
            scenario,
            IntegrationMethod::Euler
        );


    const VehicleState rk4Result =
        runSimulation(
            scenario,
            IntegrationMethod::RK4
        );


    const double positionDifference =
        std::hypot(
            eulerResult.x
            -
            rk4Result.x,

            eulerResult.y
            -
            rk4Result.y
        );


    const double headingDifference =
        std::abs(
            eulerResult.heading
            -
            rk4Result.heading
        );


    std::cout
        << std::fixed
        << std::setprecision(6);


    std::cout
        << "=========================================\n";

    std::cout
        << "Euler vs RK4 Integration Comparison\n";

    std::cout
        << "=========================================\n";


    std::cout
        << "dt: "
        << scenario.dt
        << " s\n";


    std::cout
        << "Duration: "
        << scenario.duration
        << " s\n";


    std::cout
        << "Steps: "
        << std::llround(
            scenario.duration
            /
            scenario.dt
        )
        << "\n\n";


    std::cout
        << "Euler final state\n";

    std::cout
        << "X: "
        << eulerResult.x
        << "\n";

    std::cout
        << "Y: "
        << eulerResult.y
        << "\n";

    std::cout
        << "Velocity: "
        << eulerResult.velocity
        << "\n";

    std::cout
        << "Heading: "
        << eulerResult.heading
        << "\n\n";


    std::cout
        << "RK4 final state\n";

    std::cout
        << "X: "
        << rk4Result.x
        << "\n";

    std::cout
        << "Y: "
        << rk4Result.y
        << "\n";

    std::cout
        << "Velocity: "
        << rk4Result.velocity
        << "\n";

    std::cout
        << "Heading: "
        << rk4Result.heading
        << "\n\n";


    std::cout
        << "Euler/RK4 position difference: "
        << positionDifference
        << " m\n";


    std::cout
        << "Euler/RK4 heading difference: "
        << headingDifference
        << " rad\n";


    return 0;
}