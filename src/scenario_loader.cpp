#include "scenario_loader.hpp"

#include <fstream>
#include <iostream>
#include <stdexcept>

#include "json.hpp"


using json = nlohmann::json;


Scenario loadScenario(
    const std::string& filePath
) {
    std::ifstream file(
        filePath
    );


    if (
        !file.is_open()
    ) {
        throw std::runtime_error(
            "Could not open scenario file: "
            +
            filePath
        );
    }


    json scenarioJson;


    try {
        file >> scenarioJson;
    }
    catch (
        const std::exception& exception
    ) {
        throw std::runtime_error(
            "Failed to parse JSON file: "
            +
            filePath
            +
            "\nReason: "
            +
            exception.what()
        );
    }


    Scenario scenario{};

    scenario.name =
        scenarioJson.at(
            "name"
        ).get<std::string>();


    scenario.outputFile =
        scenarioJson.at(
            "outputFile"
        ).get<std::string>();


    scenario.initialVelocity =
        scenarioJson.at(
            "initialVelocity"
        ).get<double>();


    scenario.acceleration =
        scenarioJson.at(
            "acceleration"
        ).get<double>();


    scenario.steeringAngle =
        scenarioJson.at(
            "steeringAngle"
        ).get<double>();


    scenario.wheelbase =
        scenarioJson.at(
            "wheelbase"
        ).get<double>();


    scenario.duration =
        scenarioJson.at(
            "duration"
        ).get<double>();


    scenario.dt =
        scenarioJson.at(
            "dt"
        ).get<double>();


    scenario.integrationMethod =
        scenarioJson.value(
            "integrationMethod",
            "Euler"
        );


    scenario.steeringSchedule.clear();


    if (
        scenarioJson.contains(
            "steeringSchedule"
        )
    ) {
        for (
            const auto& eventJson :
            scenarioJson.at(
                "steeringSchedule"
            )
        ) {
            SteeringEvent event{};

            event.start =
                eventJson.at(
                    "start"
                ).get<double>();

            event.end =
                eventJson.at(
                    "end"
                ).get<double>();

            event.angle =
                eventJson.at(
                    "angle"
                ).get<double>();


            if (
                event.start < 0.0
            ) {
                throw std::runtime_error(
                    "Steering event start time "
                    "cannot be negative."
                );
            }


            if (
                event.end <= event.start
            ) {
                throw std::runtime_error(
                    "Steering event end time must "
                    "be greater than start time."
                );
            }


            scenario.steeringSchedule.push_back(
                event
            );
        }
    }


    scenario.cruiseControlEnabled =
        scenarioJson.value(
            "cruiseControlEnabled",
            false
        );


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


    scenario.adaptiveCruiseControlEnabled =
        scenarioJson.value(
            "adaptiveCruiseControlEnabled",
            false
        );


    scenario.leadVehicleInitialDistance =
        0.0;


    scenario.leadVehicleInitialVelocity =
        0.0;


    scenario.leadVehicleBrakeStart =
        0.0;


    scenario.leadVehicleBrakeEnd =
        0.0;


    scenario.leadVehicleBrakeAcceleration =
        0.0;


    scenario.desiredTimeHeadway =
        0.0;


    scenario.minimumFollowingDistance =
        0.0;


    scenario.accGapKp =
        0.0;


    scenario.accRelativeVelocityKp =
        0.0;

    if (
        scenario.cruiseControlEnabled
    ) {
        scenario.targetVelocity =
            scenarioJson.at(
                "targetVelocity"
            ).get<double>();


        scenario.cruiseKp =
            scenarioJson.at(
                "cruiseKp"
            ).get<double>();


        scenario.cruiseKi =
            scenarioJson.at(
                "cruiseKi"
            ).get<double>();


        scenario.minimumAcceleration =
            scenarioJson.at(
                "minimumAcceleration"
            ).get<double>();


        scenario.maximumAcceleration =
            scenarioJson.at(
                "maximumAcceleration"
            ).get<double>();
    }


    if (
        scenario.adaptiveCruiseControlEnabled
    ) {
        scenario.leadVehicleInitialDistance =
            scenarioJson.at(
                "leadVehicleInitialDistance"
            ).get<double>();


        scenario.leadVehicleInitialVelocity =
            scenarioJson.at(
                "leadVehicleInitialVelocity"
            ).get<double>();


        scenario.leadVehicleBrakeStart =
            scenarioJson.at(
                "leadVehicleBrakeStart"
            ).get<double>();


        scenario.leadVehicleBrakeEnd =
            scenarioJson.at(
                "leadVehicleBrakeEnd"
            ).get<double>();


        scenario.leadVehicleBrakeAcceleration =
            scenarioJson.at(
                "leadVehicleBrakeAcceleration"
            ).get<double>();


        scenario.desiredTimeHeadway =
            scenarioJson.at(
                "desiredTimeHeadway"
            ).get<double>();


        scenario.minimumFollowingDistance =
            scenarioJson.at(
                "minimumFollowingDistance"
            ).get<double>();


        scenario.accGapKp =
            scenarioJson.at(
                "accGapKp"
            ).get<double>();


        scenario.accRelativeVelocityKp =
            scenarioJson.at(
                "accRelativeVelocityKp"
            ).get<double>();


        scenario.minimumAcceleration =
            scenarioJson.at(
                "minimumAcceleration"
            ).get<double>();


        scenario.maximumAcceleration =
            scenarioJson.at(
                "maximumAcceleration"
            ).get<double>();
    }

    if (
        scenario.initialVelocity < 0.0
    ) {
        throw std::runtime_error(
            "Initial velocity cannot be negative."
        );
    }


    if (
        scenario.wheelbase <= 0.0
    ) {
        throw std::runtime_error(
            "Wheelbase must be greater than zero."
        );
    }


    if (
        scenario.duration <= 0.0
    ) {
        throw std::runtime_error(
            "Scenario duration must be greater than zero."
        );
    }


    if (
        scenario.dt <= 0.0
    ) {
        throw std::runtime_error(
            "Simulation timestep must be greater than zero."
        );
    }


    if (
        scenario.dt > scenario.duration
    ) {
        throw std::runtime_error(
            "Simulation timestep cannot be larger "
            "than scenario duration."
        );
    }


    if (
        scenario.integrationMethod != "Euler"
        &&
        scenario.integrationMethod != "euler"
        &&
        scenario.integrationMethod != "RK4"
        &&
        scenario.integrationMethod != "rk4"
    ) {
        throw std::runtime_error(
            "Unsupported integration method: "
            +
            scenario.integrationMethod
        );
    }


    if (
        scenario.minimumAcceleration
        >
        scenario.maximumAcceleration
    ) {
        throw std::runtime_error(
            "minimumAcceleration must be less than "
            "or equal to maximumAcceleration."
        );
    }

    if (
        scenario.cruiseControlEnabled
        &&
        scenario.adaptiveCruiseControlEnabled
    ) {
        throw std::runtime_error(
            "Cruise control and Adaptive Cruise Control "
            "cannot both be enabled in the same scenario."
        );
    }


    if (
        scenario.cruiseControlEnabled
    ) {
        if (
            scenario.targetVelocity < 0.0
        ) {
            throw std::runtime_error(
                "Cruise-control target velocity "
                "cannot be negative."
            );
        }


        if (
            scenario.cruiseKp < 0.0
            ||
            scenario.cruiseKi < 0.0
        ) {
            throw std::runtime_error(
                "Cruise-control gains cannot be negative."
            );
        }
    }

    if (
        scenario.adaptiveCruiseControlEnabled
    ) {
        if (
            scenario.leadVehicleInitialDistance <= 0.0
        ) {
            throw std::runtime_error(
                "ACC leadVehicleInitialDistance "
                "must be greater than zero."
            );
        }


        if (
            scenario.leadVehicleInitialVelocity < 0.0
        ) {
            throw std::runtime_error(
                "ACC leadVehicleInitialVelocity "
                "cannot be negative."
            );
        }


        if (
            scenario.leadVehicleBrakeStart < 0.0
        ) {
            throw std::runtime_error(
                "ACC leadVehicleBrakeStart "
                "cannot be negative."
            );
        }


        if (
            scenario.leadVehicleBrakeEnd
            <=
            scenario.leadVehicleBrakeStart
        ) {
            throw std::runtime_error(
                "ACC leadVehicleBrakeEnd must be "
                "greater than leadVehicleBrakeStart."
            );
        }


        if (
            scenario.leadVehicleBrakeEnd
            >
            scenario.duration
        ) {
            throw std::runtime_error(
                "ACC lead-vehicle braking event "
                "cannot extend past scenario duration."
            );
        }


        if (
            scenario.desiredTimeHeadway <= 0.0
        ) {
            throw std::runtime_error(
                "ACC desiredTimeHeadway "
                "must be greater than zero."
            );
        }


        if (
            scenario.minimumFollowingDistance <= 0.0
        ) {
            throw std::runtime_error(
                "ACC minimumFollowingDistance "
                "must be greater than zero."
            );
        }


        if (
            scenario.accGapKp < 0.0
            ||
            scenario.accRelativeVelocityKp < 0.0
        ) {
            throw std::runtime_error(
                "ACC controller gains cannot be negative."
            );
        }
    }

    std::cout
        << "\n[Scenario Loader]\n";


    std::cout
        << "File: "
        << filePath
        << "\n";


    std::cout
        << "Name: "
        << scenario.name
        << "\n";


    std::cout
        << "Initial velocity: "
        << scenario.initialVelocity
        << "\n";


    std::cout
        << "Cruise enabled: "
        << (
            scenario.cruiseControlEnabled
            ? "true"
            : "false"
        )
        << "\n";


    std::cout
        << "ACC enabled: "
        << (
            scenario.adaptiveCruiseControlEnabled
            ? "true"
            : "false"
        )
        << "\n";


    if (
        scenario.cruiseControlEnabled
    ) {
        std::cout
            << "Target velocity: "
            << scenario.targetVelocity
            << "\n";


        std::cout
            << "Kp: "
            << scenario.cruiseKp
            << "\n";


        std::cout
            << "Ki: "
            << scenario.cruiseKi
            << "\n";
    }


    if (
        scenario.adaptiveCruiseControlEnabled
    ) {
        std::cout
            << "Lead initial distance: "
            << scenario.leadVehicleInitialDistance
            << "\n";


        std::cout
            << "Lead initial velocity: "
            << scenario.leadVehicleInitialVelocity
            << "\n";


        std::cout
            << "Desired headway: "
            << scenario.desiredTimeHeadway
            << "\n";


        std::cout
            << "Minimum following distance: "
            << scenario.minimumFollowingDistance
            << "\n";


        std::cout
            << "ACC gap Kp: "
            << scenario.accGapKp
            << "\n";


        std::cout
            << "ACC relative velocity Kp: "
            << scenario.accRelativeVelocityKp
            << "\n";
    }


    return scenario;
}