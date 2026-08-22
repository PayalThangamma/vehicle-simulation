#include "scenario_loader.hpp"

#include <fstream>
#include <iostream>
#include <stdexcept>

#include "json.hpp"


using json = nlohmann::json;


Scenario loadScenario(
    const std::string& filePath
) {
    std::ifstream file(filePath);

    if (!file.is_open()) {
        throw std::runtime_error(
            "Could not open scenario file: "
            + filePath
        );
    }


    json jsonData;


    try {
        file >> jsonData;
    }
    catch (
        const json::parse_error& error
    ) {
        throw std::runtime_error(
            "Invalid JSON in scenario file: "
            + filePath
            + "\n"
            + error.what()
        );
    }


    Scenario scenario{};


    try {

        scenario.name =
            jsonData.at("name")
            .get<std::string>();


        scenario.outputFile =
            jsonData.at("outputFile")
            .get<std::string>();


        scenario.initialVelocity =
            jsonData.at("initialVelocity")
            .get<double>();


        scenario.acceleration =
            jsonData.at("acceleration")
            .get<double>();


        scenario.steeringAngle =
            jsonData.at("steeringAngle")
            .get<double>();


        scenario.wheelbase =
            jsonData.at("wheelbase")
            .get<double>();


        scenario.duration =
            jsonData.at("duration")
            .get<double>();


        scenario.dt =
            jsonData.at("dt")
            .get<double>();

        scenario.integrationMethod =
            jsonData.value(
                "integrationMethod",
                std::string("Euler")
            );


        scenario.cruiseControlEnabled = false;

        scenario.targetVelocity =
            scenario.initialVelocity;

        scenario.cruiseKp = 0.0;

        scenario.cruiseKi = 0.0;

        scenario.minimumAcceleration = -6.0;

        scenario.maximumAcceleration = 3.0;


        if (
            jsonData.contains(
                "cruiseControlEnabled"
            )
        ) {

            scenario.cruiseControlEnabled =
                jsonData.at(
                    "cruiseControlEnabled"
                )
                .get<bool>();
        }


        if (
            scenario.cruiseControlEnabled
        ) {

            scenario.targetVelocity =
                jsonData.at(
                    "targetVelocity"
                )
                .get<double>();


            scenario.cruiseKp =
                jsonData.at(
                    "cruiseKp"
                )
                .get<double>();


            scenario.cruiseKi =
                jsonData.at(
                    "cruiseKi"
                )
                .get<double>();


            scenario.minimumAcceleration =
                jsonData.at(
                    "minimumAcceleration"
                )
                .get<double>();


            scenario.maximumAcceleration =
                jsonData.at(
                    "maximumAcceleration"
                )
                .get<double>();
        }

        if (
            jsonData.contains(
                "steeringSchedule"
            )
        ) {

            for (
                const auto& eventJson :
                jsonData.at(
                    "steeringSchedule"
                )
            ) {

                SteeringEvent event{};


                event.start =
                    eventJson.at("start")
                    .get<double>();


                event.end =
                    eventJson.at("end")
                    .get<double>();


                event.angle =
                    eventJson.at("angle")
                    .get<double>();


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
                        "Steering event end time must be "
                        "greater than start time."
                    );
                }


                scenario.steeringSchedule.push_back(
                    event
                );
            }
        }
    }
    catch (
        const json::exception& error
    ) {
        throw std::runtime_error(
            "Invalid scenario configuration in: "
            + filePath
            + "\n"
            + error.what()
        );
    }

    if (
        scenario.dt <= 0.0
    ) {
        throw std::runtime_error(
            "Scenario dt must be greater than zero."
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
        scenario.wheelbase <= 0.0
    ) {
        throw std::runtime_error(
            "Scenario wheelbase must be greater than zero."
        );
    }


    if (
        scenario.initialVelocity < 0.0
    ) {
        throw std::runtime_error(
            "Initial velocity cannot be negative."
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
            "Invalid integration method: "
            + scenario.integrationMethod
            + ". Expected Euler or RK4."
        );
    }

    if (
        scenario.cruiseControlEnabled
    ) {

        if (
            scenario.targetVelocity < 0.0
        ) {
            throw std::runtime_error(
                "Cruise target velocity cannot "
                "be negative."
            );
        }


        if (
            scenario.cruiseKp < 0.0
        ) {
            throw std::runtime_error(
                "Cruise Kp must be non-negative."
            );
        }


        if (
            scenario.cruiseKi < 0.0
        ) {
            throw std::runtime_error(
                "Cruise Ki must be non-negative."
            );
        }


        if (
            scenario.minimumAcceleration
            >=
            scenario.maximumAcceleration
        ) {
            throw std::runtime_error(
                "minimumAcceleration must be smaller "
                "than maximumAcceleration."
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
        << "Target velocity: "
        << scenario.targetVelocity
        << "\n";


    if (
        scenario.cruiseControlEnabled
    ) {

        std::cout
            << "Kp: "
            << scenario.cruiseKp
            << "\n";

        std::cout
            << "Ki: "
            << scenario.cruiseKi
            << "\n";
    }


    return scenario;
}