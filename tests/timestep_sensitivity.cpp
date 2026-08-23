#include "scenario.hpp"
#include "vehicle_model.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>


namespace fs = std::filesystem;

struct SimulationResult {
    VehicleState state;
    int steps;
    double simulatedTime;
};

SimulationResult runSimulation(
    double dt,
    IntegrationMethod method
) {
    Scenario scenario{};

    scenario.name =
        "Timestep Sensitivity";

    scenario.outputFile =
        "";

    scenario.initialVelocity =
        15.0;

    scenario.acceleration =
        0.0;

    scenario.steeringAngle =
        0.08;

    scenario.wheelbase =
        2.7;

    scenario.duration =
        5.0;

    scenario.dt =
        dt;

    scenario.integrationMethod =
        (
            method == IntegrationMethod::Euler
            ? "Euler"
            : "RK4"
        );

    scenario.cruiseControlEnabled =
        false;

    scenario.targetVelocity =
        scenario.initialVelocity;

    scenario.cruiseKp =
        0.0;

    scenario.cruiseKi =
        0.0;

    scenario.minimumAcceleration =
        -10.0;

    scenario.maximumAcceleration =
        10.0;


    VehicleState vehicle{
        0.0,
        0.0,
        scenario.initialVelocity,
        0.0
    };


    const int numberOfSteps =
        static_cast<int>(
            std::llround(
                scenario.duration
                /
                scenario.dt
            )
        );


    for (
        int step = 0;
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


    const double simulatedTime =
        numberOfSteps
        *
        scenario.dt;


    return {
        vehicle,
        numberOfSteps,
        simulatedTime
    };
}


int main() {
    try {
        const std::vector<double> timesteps{
            0.50,
            0.20,
            0.10,
            0.05,
            0.01
        };

        const fs::path resultsDirectory =
            fs::path("results");


        fs::create_directories(
            resultsDirectory
        );


        const fs::path outputPath =
            resultsDirectory
            /
            "timestep_sensitivity.csv";


        std::ofstream outputFile(
            outputPath
        );


        if (
            !outputFile.is_open()
        ) {
            throw std::runtime_error(
                "Could not create output file: "
                +
                outputPath.string()
            );
        }


        outputFile
            << "dt,"
            << "steps,"
            << "simulated_time,"
            << "position_difference,"
            << "heading_difference\n";


        std::cout
            << "\n============================================\n"
            << "Timestep Sensitivity Analysis\n"
            << "============================================\n";


        std::cout
            << std::fixed
            << std::setprecision(6);


        for (
            const double dt :
            timesteps
        ) {
            const SimulationResult eulerResult =
                runSimulation(
                    dt,
                    IntegrationMethod::Euler
                );


            const SimulationResult rk4Result =
                runSimulation(
                    dt,
                    IntegrationMethod::RK4
                );


            const double deltaX =
                eulerResult.state.x
                -
                rk4Result.state.x;


            const double deltaY =
                eulerResult.state.y
                -
                rk4Result.state.y;


            const double positionDifference =
                std::sqrt(
                    deltaX * deltaX
                    +
                    deltaY * deltaY
                );


            const double headingDifference =
                std::abs(
                    eulerResult.state.heading
                    -
                    rk4Result.state.heading
                );


            std::cout
                << "\ndt: "
                << dt
                << " s\n";


            std::cout
                << "Steps: "
                << eulerResult.steps
                << "\n";


            std::cout
                << "Simulated time: "
                << eulerResult.simulatedTime
                << " s\n";


            std::cout
                << "Euler final position: ("
                << eulerResult.state.x
                << ", "
                << eulerResult.state.y
                << ")\n";


            std::cout
                << "RK4 final position: ("
                << rk4Result.state.x
                << ", "
                << rk4Result.state.y
                << ")\n";


            std::cout
                << "Position difference: "
                << positionDifference
                << " m\n";


            std::cout
                << "Heading difference: "
                << headingDifference
                << " rad\n";


            outputFile
                << dt << ","
                << eulerResult.steps << ","
                << eulerResult.simulatedTime << ","
                << positionDifference << ","
                << headingDifference
                << "\n";
        }


        outputFile.close();


        std::cout
            << "\n============================================\n";


        std::cout
            << "Results saved to: "
            << outputPath.string()
            << "\n";


        std::cout
            << "============================================\n";


        return 0;
    }
    catch (
        const std::exception& exception
    ) {
        std::cerr
            << "ERROR: "
            << exception.what()
            << "\n";

        return 1;
    }
}