from pathlib import Path
import json


SWEEP_FILE = Path(
    "sweeps/turn_sweep.json"
)

OUTPUT_DIR = Path(
    "scenarios/generated"
)


def sanitize_number(value):
    return (
        str(value)
        .replace(".", "_")
        .replace("-", "m")
    )


def main():
    if not SWEEP_FILE.exists():
        raise FileNotFoundError(
            f"Missing sweep file: {SWEEP_FILE}"
        )


    with SWEEP_FILE.open(
        "r",
        encoding="utf-8"
    ) as file:
        config = json.load(
            file
        )


    OUTPUT_DIR.mkdir(
        parents=True,
        exist_ok=True
    )


    # Remove old generated JSON scenarios so that
    # stale sweep configurations are not accidentally run.

    for old_file in OUTPUT_DIR.glob(
        "*.json"
    ):
        old_file.unlink()


    velocity_values = config[
        "initialVelocityValues"
    ]

    steering_values = config[
        "steeringAngleValues"
    ]

    integration_methods = config.get(
        "integrationMethods",
        ["Euler"]
    )


    generated_count = 0


    for velocity in velocity_values:

        for steering in steering_values:

            for method in integration_methods:

                velocity_id = sanitize_number(
                    velocity
                )

                steering_id = sanitize_number(
                    steering
                )

                method_id = (
                    method.lower()
                )


                scenario_id = (
                    f"turn_"
                    f"{method_id}_"
                    f"v{velocity_id}_"
                    f"s{steering_id}"
                )


                scenario = {
                    "name": (
                        f"Turn Sweep "
                        f"{method} "
                        f"v={velocity} "
                        f"steering={steering}"
                    ),

                    "outputFile": (
                        f"results/"
                        f"{scenario_id}.csv"
                    ),

                    "initialVelocity":
                        velocity,

                    "acceleration":
                        config[
                            "acceleration"
                        ],

                    "steeringAngle":
                        steering,

                    "wheelbase":
                        config[
                            "wheelbase"
                        ],

                    "duration":
                        config[
                            "duration"
                        ],

                    "dt":
                        config[
                            "dt"
                        ],

                    "integrationMethod":
                        method,
                }


                output_file = (
                    OUTPUT_DIR
                    /
                    f"{scenario_id}.json"
                )


                with output_file.open(
                    "w",
                    encoding="utf-8"
                ) as file:

                    json.dump(
                        scenario,
                        file,
                        indent=2
                    )


                generated_count += 1


    print(
        f"Generated {generated_count} "
        f"simulation scenarios."
    )


if __name__ == "__main__":
    main()