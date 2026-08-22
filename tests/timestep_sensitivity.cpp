#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

#include "vehicle_model.hpp"


VehicleState runSimulationWithMethod(
    Scenario scenario,
    IntegrationMethod method
) {
    VehicleState car{
        0.0,
        0.0,
        scenario.initialVelocity,
        0.0
    };

    double currentTime = 0.0;

    while (currentTime < scenario.duration) {

        updateVehicle(
            car,
            scenario,
            scenario.steeringAngle,
            method
        );

        currentTime += scenario.dt;
    }

    return car;
}


double positionDifference(
    const VehicleState& a,
    const VehicleState& b
) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;

    return std::sqrt(
        dx * dx + dy * dy
    );
}


int main() {

    std::vector<double> timeSteps{
        0.5,
        0.2,
        0.1,
        0.05,
        0.01
    };


    Scenario scenario{};

    scenario.initialVelocity = 15.0;
    scenario.acceleration = 0.0;
    scenario.steeringAngle = 0.1;
    scenario.wheelbase = 2.7;
    scenario.duration = 5.0;


    std::cout
        << "\n===============================================\n";

    std::cout
        << "Timestep Sensitivity: Euler vs RK4\n";

    std::cout
        << "===============================================\n\n";


    std::cout
        << std::left
        << std::setw(10) << "dt"
        << std::setw(18) << "Euler X"
        << std::setw(18) << "Euler Y"
        << std::setw(18) << "RK4 X"
        << std::setw(18) << "RK4 Y"
        << std::setw(18) << "Position Error"
        << "\n";


    for (double dt : timeSteps) {

        scenario.dt = dt;


        VehicleState euler =
            runSimulationWithMethod(
                scenario,
                IntegrationMethod::Euler
            );


        VehicleState rk4 =
            runSimulationWithMethod(
                scenario,
                IntegrationMethod::RK4
            );


        const double error =
            positionDifference(
                euler,
                rk4
            );


        std::cout
            << std::left
            << std::setw(10) << dt
            << std::setw(18) << euler.x
            << std::setw(18) << euler.y
            << std::setw(18) << rk4.x
            << std::setw(18) << rk4.y
            << std::setw(18) << error
            << "\n";
    }


    std::cout
        << "\nSmaller dt should reduce the difference "
        << "between Euler and RK4.\n";


    return 0;
}