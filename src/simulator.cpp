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


static double calculateAdaptiveCruiseAcceleration(
    const Scenario& scenario,
    const VehicleState& egoVehicle,
    double leadVehiclePosition,
    double leadVehicleVelocity
) {
    const double actualGap =
        leadVehiclePosition
        -
        egoVehicle.x;


    const double desiredGap =
        scenario.minimumFollowingDistance
        +
        scenario.desiredTimeHeadway
        *
        egoVehicle.velocity;


    const double gapError =
        actualGap
        -
        desiredGap;


    const double relativeVelocity =
        leadVehicleVelocity
        -
        egoVehicle.velocity;


    const double unconstrainedCommand =
        scenario.accGapKp
        *
        gapError
        +
        scenario.accRelativeVelocityKp
        *
        relativeVelocity;


    return std::clamp(
        unconstrainedCommand,
        scenario.minimumAcceleration,
        scenario.maximumAcceleration
    );
}


static double getLeadVehicleAcceleration(
    const Scenario& scenario,
    double currentTime
) {
    if (
        currentTime >= scenario.leadVehicleBrakeStart
        &&
        currentTime < scenario.leadVehicleBrakeEnd
    ) {
        return scenario.leadVehicleBrakeAcceleration;
    }

    return 0.0;
}


void runSimulation(
    const Scenario& scenario
) {
    VehicleState egoVehicle{
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



    double leadVehiclePosition =
        scenario.leadVehicleInitialDistance;


    double leadVehicleVelocity =
        scenario.leadVehicleInitialVelocity;


    double leadVehicleAcceleration =
        0.0;



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
        << "speed_error,"
        << "lead_vehicle_position,"
        << "lead_vehicle_velocity,"
        << "lead_vehicle_acceleration,"
        << "actual_gap,"
        << "desired_gap,"
        << "gap_error,"
        << "relative_velocity\n";


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


    double actualGap =
        0.0;


    double desiredGap =
        0.0;


    double gapError =
        0.0;


    double relativeVelocity =
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
            egoVehicle.velocity;
    }


    if (
        scenario.adaptiveCruiseControlEnabled
    ) {
        currentAcceleration =
            0.0;

        actualGap =
            leadVehiclePosition
            -
            egoVehicle.x;

        desiredGap =
            scenario.minimumFollowingDistance
            +
            scenario.desiredTimeHeadway
            *
            egoVehicle.velocity;

        gapError =
            actualGap
            -
            desiredGap;

        relativeVelocity =
            leadVehicleVelocity
            -
            egoVehicle.velocity;

        targetVelocityForLog =
            leadVehicleVelocity;

        speedError =
            relativeVelocity;
    }


    file
        << currentTime << ","
        << egoVehicle.x << ","
        << egoVehicle.y << ","
        << egoVehicle.velocity << ","
        << egoVehicle.heading << ","
        << currentAcceleration << ","
        << currentSteering << ","
        << scenario.wheelbase << ","
        << targetVelocityForLog << ","
        << speedError << ","
        << leadVehiclePosition << ","
        << leadVehicleVelocity << ","
        << leadVehicleAcceleration << ","
        << actualGap << ","
        << desiredGap << ","
        << gapError << ","
        << relativeVelocity
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


    std::cout
        << "Adaptive Cruise Control: "
        << (
            scenario.adaptiveCruiseControlEnabled
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


    if (
        scenario.adaptiveCruiseControlEnabled
    ) {
        std::cout
            << "Lead initial distance: "
            << scenario.leadVehicleInitialDistance
            << " m\n";


        std::cout
            << "Lead initial velocity: "
            << scenario.leadVehicleInitialVelocity
            << " m/s\n";


        std::cout
            << "Lead braking interval: ["
            << scenario.leadVehicleBrakeStart
            << ", "
            << scenario.leadVehicleBrakeEnd
            << "] s\n";


        std::cout
            << "Lead braking acceleration: "
            << scenario.leadVehicleBrakeAcceleration
            << " m/s^2\n";


        std::cout
            << "Desired time headway: "
            << scenario.desiredTimeHeadway
            << " s\n";


        std::cout
            << "Minimum following distance: "
            << scenario.minimumFollowingDistance
            << " m\n";


        std::cout
            << "ACC gap gain: "
            << scenario.accGapKp
            << "\n";


        std::cout
            << "ACC relative velocity gain: "
            << scenario.accRelativeVelocityKp
            << "\n";


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

    while (
        true
    ) {
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
            scenario.adaptiveCruiseControlEnabled
        ) {
            leadVehicleAcceleration =
                getLeadVehicleAcceleration(
                    scenario,
                    currentTime
                );


            leadVehiclePosition =
                leadVehiclePosition
                +
                leadVehicleVelocity
                *
                scenario.dt;


            leadVehicleVelocity =
                leadVehicleVelocity
                +
                leadVehicleAcceleration
                *
                scenario.dt;


            if (
                leadVehicleVelocity < 0.0
            ) {
                leadVehicleVelocity =
                    0.0;
            }
        }


        if (
            scenario.cruiseControlEnabled
        ) {
            currentAcceleration =
                calculateCruiseAcceleration(
                    scenario,
                    egoVehicle,
                    integralSpeedError
                );
        }
        else if (
            scenario.adaptiveCruiseControlEnabled
        ) {
            currentAcceleration =
                calculateAdaptiveCruiseAcceleration(
                    scenario,
                    egoVehicle,
                    leadVehiclePosition,
                    leadVehicleVelocity
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
            egoVehicle,
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
                egoVehicle.velocity;


            actualGap =
                0.0;


            desiredGap =
                0.0;


            gapError =
                0.0;


            relativeVelocity =
                0.0;
        }
        else if (
            scenario.adaptiveCruiseControlEnabled
        ) {
            actualGap =
                leadVehiclePosition
                -
                egoVehicle.x;


            desiredGap =
                scenario.minimumFollowingDistance
                +
                scenario.desiredTimeHeadway
                *
                egoVehicle.velocity;


            gapError =
                actualGap
                -
                desiredGap;


            relativeVelocity =
                leadVehicleVelocity
                -
                egoVehicle.velocity;


            targetVelocityForLog =
                leadVehicleVelocity;


            speedError =
                relativeVelocity;
        }
        else {
            targetVelocityForLog =
                egoVehicle.velocity;


            speedError =
                0.0;


            actualGap =
                0.0;


            desiredGap =
                0.0;


            gapError =
                0.0;


            relativeVelocity =
                0.0;
        }

        std::cout
            << "Time: "
            << currentTime
            << " s"

            << " | X: "
            << egoVehicle.x
            << " m"

            << " | Y: "
            << egoVehicle.y
            << " m"

            << " | Velocity: "
            << egoVehicle.velocity
            << " m/s"

            << " | Heading: "
            << egoVehicle.heading
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


        if (
            scenario.adaptiveCruiseControlEnabled
        ) {
            std::cout
                << " | Lead velocity: "
                << leadVehicleVelocity
                << " m/s"

                << " | Gap: "
                << actualGap
                << " m"

                << " | Desired gap: "
                << desiredGap
                << " m"

                << " | Gap error: "
                << gapError
                << " m"

                << " | Relative velocity: "
                << relativeVelocity
                << " m/s";
        }


        std::cout
            << "\n";

        file
            << currentTime << ","
            << egoVehicle.x << ","
            << egoVehicle.y << ","
            << egoVehicle.velocity << ","
            << egoVehicle.heading << ","
            << currentAcceleration << ","
            << currentSteering << ","
            << scenario.wheelbase << ","
            << targetVelocityForLog << ","
            << speedError << ","
            << leadVehiclePosition << ","
            << leadVehicleVelocity << ","
            << leadVehicleAcceleration << ","
            << actualGap << ","
            << desiredGap << ","
            << gapError << ","
            << relativeVelocity
            << "\n";


        if (
            scenario.adaptiveCruiseControlEnabled
            &&
            actualGap <= 0.0
        ) {
            std::cout
                << "WARNING: Ego vehicle reached "
                << "the lead vehicle position.\n";

            break;
        }

        if (
            !scenario.cruiseControlEnabled
            &&
            !scenario.adaptiveCruiseControlEnabled
            &&
            egoVehicle.velocity <= 0.0
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