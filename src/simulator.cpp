#include "simulator.hpp"

#include "vehicle_model.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>


namespace fs = std::filesystem;


static IntegrationMethod getIntegrationMethod(
    const Scenario& scenario
) {
    if (
        scenario.integrationMethod == "RK4"
        ||
        scenario.integrationMethod == "rk4"
    ) {
        return IntegrationMethod::RK4;
    }

    return IntegrationMethod::Euler;
}


double getSteeringAngle(
    const Scenario& scenario,
    double currentTime
) {
    for (
        const SteeringEvent& event :
        scenario.steeringSchedule
    ) {
        if (
            currentTime >= event.start
            &&
            currentTime < event.end
        ) {
            return event.angle;
        }
    }

    return scenario.steeringAngle;
}


static double calculateCruiseAcceleration(
    const Scenario& scenario,
    const VehicleState& vehicle,
    double& integralError
) {
    const double speedError =
        scenario.targetVelocity
        -
        vehicle.velocity;


    const double candidateIntegral =
        integralError
        +
        speedError
        *
        scenario.dt;


    const double proportionalTerm =
        scenario.cruiseKp
        *
        speedError;


    const double candidateIntegralTerm =
        scenario.cruiseKi
        *
        candidateIntegral;


    const double unconstrainedCommand =
        proportionalTerm
        +
        candidateIntegralTerm;


    const double accelerationCommand =
        std::clamp(
            unconstrainedCommand,
            scenario.minimumAcceleration,
            scenario.maximumAcceleration
        );



    const bool saturatedHigh =
        unconstrainedCommand
        >
        scenario.maximumAcceleration;


    const bool saturatedLow =
        unconstrainedCommand
        <
        scenario.minimumAcceleration;


    const bool allowIntegration =
        (
            !saturatedHigh
            &&
            !saturatedLow
        )
        ||
        (
            saturatedHigh
            &&
            speedError < 0.0
        )
        ||
        (
            saturatedLow
            &&
            speedError > 0.0
        );


    if (
        allowIntegration
    ) {
        integralError =
            candidateIntegral;
    }


    return accelerationCommand;
}

void runSimulation(
    const Scenario& scenario
) {
    VehicleState car{
        0.0,
        0.0,
        scenario.initialVelocity,
        0.0
    };


    double currentTime =
        0.0;


    double integralSpeedError =
        0.0;


    const IntegrationMethod integrationMethod =
        getIntegrationMethod(
            scenario
        );


    const fs::path outputPath(
        scenario.outputFile
    );


    if (
        outputPath.has_parent_path()
    ) {
        fs::create_directories(
            outputPath.parent_path()
        );
    }


    std::ofstream file(
        scenario.outputFile
    );


    if (
        !file.is_open()
    ) {
        throw std::runtime_error(
            "Could not create output file: "
            +
            scenario.outputFile
        );
    }

    file
        << "time,"
        << "x,"
        << "y,"
        << "velocity,"
        << "heading,"
        << "commanded_acceleration,"
        << "steering_angle,"
        << "wheelbase,"
        << "target_velocity,"
        << "speed_error\n";


    double currentSteering =
        getSteeringAngle(
            scenario,
            currentTime
        );


    double currentAcceleration =
        scenario.acceleration;


    double targetVelocityForLog =
        scenario.initialVelocity;


    double speedError =
        0.0;


    if (
        scenario.cruiseControlEnabled
    ) {
        currentAcceleration =
            0.0;

        targetVelocityForLog =
            scenario.targetVelocity;

        speedError =
            scenario.targetVelocity
            -
            car.velocity;
    }


    file
        << currentTime << ","
        << car.x << ","
        << car.y << ","
        << car.velocity << ","
        << car.heading << ","
        << currentAcceleration << ","
        << currentSteering << ","
        << scenario.wheelbase << ","
        << targetVelocityForLog << ","
        << speedError
        << "\n";


    std::cout
        << "\n============================================\n";


    std::cout
        << "Scenario: "
        << scenario.name
        << "\n";


    std::cout
        << "Initial velocity: "
        << scenario.initialVelocity
        << " m/s\n";


    std::cout
        << "Base acceleration: "
        << scenario.acceleration
        << " m/s^2\n";


    std::cout
        << "Default steering: "
        << scenario.steeringAngle
        << " rad\n";


    std::cout
        << "Steering events: "
        << scenario.steeringSchedule.size()
        << "\n";


    std::cout
        << "Wheelbase: "
        << scenario.wheelbase
        << " m\n";


    std::cout
        << "Duration: "
        << scenario.duration
        << " s\n";


    std::cout
        << "dt: "
        << scenario.dt
        << " s\n";


    std::cout
        << "Integration method: "
        << scenario.integrationMethod
        << "\n";


    std::cout
        << "Cruise control: "
        << (
            scenario.cruiseControlEnabled
            ?
            "Enabled"
            :
            "Disabled"
        )
        << "\n";


    if (
        scenario.cruiseControlEnabled
    ) {
        std::cout
            << "Target velocity: "
            << scenario.targetVelocity
            << " m/s\n";


        std::cout
            << "Cruise Kp: "
            << scenario.cruiseKp
            << "\n";


        std::cout
            << "Cruise Ki: "
            << scenario.cruiseKi
            << "\n";


        std::cout
            << "Anti-windup: Enabled\n";


        std::cout
            << "Acceleration limits: ["
            << scenario.minimumAcceleration
            << ", "
            << scenario.maximumAcceleration
            << "] m/s^2\n";
    }


    std::cout
        << "============================================\n";


    constexpr double timeTolerance =
        1e-9;

    while (true) {
        const double nextTime =
            currentTime
            +
            scenario.dt;


        if (
            nextTime
            >
            scenario.duration
            +
            timeTolerance
        ) {
            break;
        }


        currentSteering =
            getSteeringAngle(
                scenario,
                currentTime
            );


        if (
            scenario.cruiseControlEnabled
        ) {
            currentAcceleration =
                calculateCruiseAcceleration(
                    scenario,
                    car,
                    integralSpeedError
                );
        }
        else {
            currentAcceleration =
                scenario.acceleration;
        }

        Scenario stepScenario =
            scenario;


        stepScenario.acceleration =
            currentAcceleration;


        updateVehicle(
            car,
            stepScenario,
            currentSteering,
            integrationMethod
        );


        currentTime =
            nextTime;


        if (
            std::abs(
                currentTime
                -
                scenario.duration
            )
            <
            timeTolerance
        ) {
            currentTime =
                scenario.duration;
        }


        if (
            scenario.cruiseControlEnabled
        ) {
            targetVelocityForLog =
                scenario.targetVelocity;


            speedError =
                scenario.targetVelocity
                -
                car.velocity;
        }
        else {
            targetVelocityForLog =
                car.velocity;

            speedError =
                0.0;
        }


        std::cout
            << "Time: "
            << currentTime
            << " s"

            << " | X: "
            << car.x
            << " m"

            << " | Y: "
            << car.y
            << " m"

            << " | Velocity: "
            << car.velocity
            << " m/s"

            << " | Heading: "
            << car.heading
            << " rad"

            << " | Accel cmd: "
            << currentAcceleration
            << " m/s^2"

            << " | Steering: "
            << currentSteering
            << " rad";


        if (
            scenario.cruiseControlEnabled
        ) {
            std::cout
                << " | Target: "
                << scenario.targetVelocity
                << " m/s"

                << " | Speed error: "
                << speedError
                << " m/s";
        }


        std::cout
            << "\n";


        file
            << currentTime << ","
            << car.x << ","
            << car.y << ","
            << car.velocity << ","
            << car.heading << ","
            << currentAcceleration << ","
            << currentSteering << ","
            << scenario.wheelbase << ","
            << targetVelocityForLog << ","
            << speedError
            << "\n";


        if (
            !scenario.cruiseControlEnabled
            &&
            car.velocity <= 0.0
            &&
            scenario.acceleration < 0.0
        ) {
            std::cout
                << "Vehicle stopped.\n";

            break;
        }
    }


    file.close();


    std::cout
        << "Results saved to: "
        << scenario.outputFile
        << "\n";
}