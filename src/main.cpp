#include <algorithm>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include "scenario_loader.hpp"
#include "simulator.hpp"


namespace fs = std::filesystem;


int main(
    int argc,
    char* argv[]
) {
    try {

        std::cout
            << "=========================================\n";

        std::cout
            << "Vehicle Simulation Factory\n";

        std::cout
            << "=========================================\n";


        if (
            argc > 1
        ) {

            const std::string filePath =
                argv[1];


            if (
                !fs::exists(
                    filePath
                )
            ) {

                std::cerr
                    << "Scenario file not found: "
                    << filePath
                    << "\n";

                return 1;
            }


            std::cout
                << "\nSingle-scenario mode\n";

            std::cout
                << "Scenario file: "
                << filePath
                << "\n";


            const Scenario scenario =
                loadScenario(
                    filePath
                );


            runSimulation(
                scenario
            );


            std::cout
                << "\n=========================================\n";

            std::cout
                << "Simulation completed successfully.\n";

            std::cout
                << "=========================================\n";


            return 0;
        }

        const std::vector<std::string>
            scenarioFiles = {

                "scenarios/acceleration.json",

                "scenarios/emergency_braking.json",

                "scenarios/constant_turn.json",

                "scenarios/lane_change.json",

                "scenarios/cruise_control.json"
            };

        for (
            const std::string& filePath :
            scenarioFiles
        ) {

            if (
                !fs::exists(
                    filePath
                )
            ) {

                std::cerr
                    << "Scenario file not found: "
                    << filePath
                    << "\n";

                return 1;
            }


            const Scenario scenario =
                loadScenario(
                    filePath
                );


            runSimulation(
                scenario
            );
        }

        const fs::path generatedDirectory =
            "scenarios/generated";


        if (
            fs::exists(
                generatedDirectory
            )
        ) {

            std::vector<fs::path>
                generatedScenarioFiles;


            for (
                const auto& entry :
                fs::directory_iterator(
                    generatedDirectory
                )
            ) {

                if (
                    entry.is_regular_file()
                    &&
                    entry.path().extension()
                    ==
                    ".json"
                ) {

                    generatedScenarioFiles.push_back(
                        entry.path()
                    );
                }
            }


            std::sort(
                generatedScenarioFiles.begin(),
                generatedScenarioFiles.end()
            );


            for (
                const fs::path& filePath :
                generatedScenarioFiles
            ) {

                const Scenario scenario =
                    loadScenario(
                        filePath.string()
                    );


                runSimulation(
                    scenario
                );
            }
        }


        std::cout
            << "\n=========================================\n";

        std::cout
            << "All simulations completed successfully.\n";

        std::cout
            << "=========================================\n";


        return 0;
    }
    catch (
        const std::exception& error
    ) {

        std::cerr
            << "\nSimulation failed:\n"
            << error.what()
            << "\n";

        return 1;
    }
}