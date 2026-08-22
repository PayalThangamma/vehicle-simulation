from pathlib import Path
import json
import math
import subprocess
import sys

import pandas as pd


BASE_SCENARIO_FILE = Path(
    "scenarios/cruise_control.json"
)

TUNING_SCENARIO_FILE = Path(
    "scenarios/cruise_tuning_current.json"
)

TUNING_RESULT_FILE = Path(
    "results/cruise_tuning_current.csv"
)

OUTPUT_FILE = Path(
    "results/cruise_control_tuning_summary.csv"
)

SIMULATOR = Path(
    "build-cmake/main.exe"
)


KP_VALUES = [
    0.4,
    0.6,
    0.8,
    1.0,
    1.2,
]


KI_VALUES = [
    0.05,
    0.10,
    0.15,
    0.20,
]


def validate_simulation_data(
    df
):
    required_columns = {
        "time",
        "velocity",
        "target_velocity",
        "speed_error",
        "commanded_acceleration",
    }


    missing_columns = (
        required_columns
        -
        set(
            df.columns
        )
    )


    if missing_columns:

        return (
            False,
            "Missing columns: "
            + str(
                sorted(
                    missing_columns
                )
            )
        )


    if df.empty:

        return (
            False,
            "Simulation result is empty."
        )


    initial_velocity = float(
        df[
            "velocity"
        ].iloc[0]
    )


    target_velocity = float(
        df[
            "target_velocity"
        ].iloc[0]
    )


    initial_error = float(
        df[
            "speed_error"
        ].iloc[0]
    )


    final_velocity = float(
        df[
            "velocity"
        ].iloc[-1]
    )


    expected_initial_error = (
        target_velocity
        -
        initial_velocity
    )


    if (
        target_velocity
        <=
        initial_velocity
    ):

        return (
            False,
            (
                "Target velocity must be greater "
                "than initial velocity. "
                f"Initial={initial_velocity}, "
                f"Target={target_velocity}"
            )
        )


    if (
        abs(
            initial_error
            -
            expected_initial_error
        )
        >
        1e-6
    ):

        return (
            False,
            (
                "Initial speed error inconsistent. "
                f"Expected={expected_initial_error}, "
                f"actual={initial_error}"
            )
        )


    if (
        abs(
            final_velocity
            -
            initial_velocity
        )
        <
        0.1
    ):

        return (
            False,
            "Vehicle velocity did not change."
        )


    target_variation = (
        df[
            "target_velocity"
        ].max()
        -
        df[
            "target_velocity"
        ].min()
    )


    if (
        abs(
            target_variation
        )
        >
        1e-9
    ):

        return (
            False,
            "Target velocity changed during simulation."
        )


    return (
        True,
        "VALID"
    )


def calculate_metrics(
    df
):
    initial_velocity = float(
        df[
            "velocity"
        ].iloc[0]
    )


    target_velocity = float(
        df[
            "target_velocity"
        ].iloc[0]
    )


    final_velocity = float(
        df[
            "velocity"
        ].iloc[-1]
    )


    maximum_velocity = float(
        df[
            "velocity"
        ].max()
    )


    requested_change = (
        target_velocity
        -
        initial_velocity
    )

    lower_threshold = (
        initial_velocity
        +
        0.10
        *
        requested_change
    )


    upper_threshold = (
        initial_velocity
        +
        0.90
        *
        requested_change
    )


    lower_rows = df[
        df[
            "velocity"
        ]
        >=
        lower_threshold
    ]


    upper_rows = df[
        df[
            "velocity"
        ]
        >=
        upper_threshold
    ]


    rise_time = math.nan


    if (
        not lower_rows.empty
        and
        not upper_rows.empty
    ):

        rise_time = (
            float(
                upper_rows[
                    "time"
                ].iloc[0]
            )
            -
            float(
                lower_rows[
                    "time"
                ].iloc[0]
            )
        )

    overshoot_percent = max(
        0.0,
        (
            maximum_velocity
            -
            target_velocity
        )
        /
        target_velocity
        *
        100.0
    )


    settling_band = (
        0.02
        *
        target_velocity
    )


    settling_time = math.nan


    for index in range(
        len(
            df
        )
    ):

        remaining_error = (
            df[
                "speed_error"
            ]
            .iloc[
                index:
            ]
            .abs()
        )


        if (
            remaining_error
            <=
            settling_band
        ).all():

            settling_time = float(
                df[
                    "time"
                ].iloc[
                    index
                ]
            )

            break


    steady_start = int(
        0.8
        *
        len(
            df
        )
    )


    steady_state_error = float(
        df[
            "speed_error"
        ]
        .iloc[
            steady_start:
        ]
        .abs()
        .mean()
    )


    final_error = abs(
        target_velocity
        -
        final_velocity
    )


    # ==================================================
    # Controller effort
    # ==================================================

    maximum_acceleration = float(
        df[
            "commanded_acceleration"
        ].max()
    )


    minimum_acceleration = float(
        df[
            "commanded_acceleration"
        ].min()
    )


    mean_acceleration_effort = float(
        df[
            "commanded_acceleration"
        ]
        .abs()
        .mean()
    )


    return {

        "rise_time_s":
            rise_time,

        "settling_time_s":
            settling_time,

        "overshoot_percent":
            overshoot_percent,

        "steady_state_error_mps":
            steady_state_error,

        "final_error_mps":
            final_error,

        "max_acceleration_mps2":
            maximum_acceleration,

        "min_acceleration_mps2":
            minimum_acceleration,

        "mean_absolute_acceleration_mps2":
            mean_acceleration_effort,
    }


def calculate_score(
    metrics
):
    rise_time = (
        metrics[
            "rise_time_s"
        ]
    )


    settling_time = (
        metrics[
            "settling_time_s"
        ]
    )


    if math.isnan(
        rise_time
    ):

        rise_time = 100.0


    if math.isnan(
        settling_time
    ):

        settling_time = 100.0


    return (
        rise_time

        +

        2.0
        *
        settling_time

        +

        3.0
        *
        metrics[
            "overshoot_percent"
        ]

        +

        10.0
        *
        metrics[
            "steady_state_error_mps"
        ]

        +

        10.0
        *
        metrics[
            "final_error_mps"
        ]

        +

        0.25
        *
        metrics[
            "mean_absolute_acceleration_mps2"
        ]
    )


def main():

    if not BASE_SCENARIO_FILE.exists():

        print(
            "Missing base scenario:",
            BASE_SCENARIO_FILE
        )

        return 1


    if not SIMULATOR.exists():

        print(
            "Missing simulator executable:",
            SIMULATOR
        )

        return 1


    with BASE_SCENARIO_FILE.open(
        "r",
        encoding="utf-8"
    ) as file:

        base_config = json.load(
            file
        )


    results = []


    try:

        for kp in KP_VALUES:

            for ki in KI_VALUES:

                print()

                print(
                    "=" * 60
                )

                print(
                    f"Testing Kp={kp}, Ki={ki}"
                )

                print(
                    "=" * 60
                )


                test_config = (
                    base_config.copy()
                )


                test_config[
                    "name"
                ] = (
                    f"Cruise Tuning "
                    f"Kp={kp} Ki={ki}"
                )


                test_config[
                    "outputFile"
                ] = (
                    "results/"
                    "cruise_tuning_current.csv"
                )


                test_config[
                    "cruiseControlEnabled"
                ] = True


                test_config[
                    "targetVelocity"
                ] = 20.0


                test_config[
                    "cruiseKp"
                ] = kp


                test_config[
                    "cruiseKi"
                ] = ki


                with TUNING_SCENARIO_FILE.open(
                    "w",
                    encoding="utf-8"
                ) as file:

                    json.dump(
                        test_config,
                        file,
                        indent=2
                    )


                if (
                    TUNING_RESULT_FILE.exists()
                ):

                    TUNING_RESULT_FILE.unlink()


                completed = subprocess.run(
                    [
                        str(
                            SIMULATOR
                        ),

                        str(
                            TUNING_SCENARIO_FILE
                        ),
                    ],

                    stdout=subprocess.DEVNULL,

                    stderr=subprocess.PIPE,

                    text=True,
                )


                if (
                    completed.returncode
                    !=
                    0
                ):

                    print(
                        "Simulation failed."
                    )

                    print(
                        completed.stderr
                    )

                    continue


                if (
                    not TUNING_RESULT_FILE.exists()
                ):

                    print(
                        "FAIL: tuning result "
                        "was not generated."
                    )

                    continue


                df = pd.read_csv(
                    TUNING_RESULT_FILE
                )


                (
                    valid,
                    message
                ) = validate_simulation_data(
                    df
                )


                if not valid:

                    print(
                        "INVALID SIMULATION RESULT:"
                    )

                    print(
                        message
                    )

                    continue


                metrics = (
                    calculate_metrics(
                        df
                    )
                )


                score = (
                    calculate_score(
                        metrics
                    )
                )


                results.append(
                    {

                        "kp":
                            kp,

                        "ki":
                            ki,

                        **metrics,

                        "score":
                            score,
                    }
                )


                print(
                    "Simulation data: VALID"
                )


                print(
                    f"Rise time: "
                    f"{metrics['rise_time_s']:.3f} s"
                )


                if math.isnan(
                    metrics[
                        "settling_time_s"
                    ]
                ):

                    print(
                        "Settling time: not settled"
                    )

                else:

                    print(
                        f"Settling time: "
                        f"{metrics['settling_time_s']:.3f} s"
                    )


                print(
                    f"Overshoot: "
                    f"{metrics['overshoot_percent']:.3f}%"
                )


                print(
                    f"Steady-state error: "
                    f"{metrics['steady_state_error_mps']:.4f} m/s"
                )


                print(
                    f"Final error: "
                    f"{metrics['final_error_mps']:.4f} m/s"
                )


                print(
                    f"Score: "
                    f"{score:.4f}"
                )


    finally:

        if (
            TUNING_SCENARIO_FILE.exists()
        ):

            TUNING_SCENARIO_FILE.unlink()


        if (
            TUNING_RESULT_FILE.exists()
        ):

            TUNING_RESULT_FILE.unlink()


    if not results:

        print()

        print(
            "No valid controller "
            "configurations were evaluated."
        )

        return 1


    summary = pd.DataFrame(
        results
    )


    summary = (
        summary.sort_values(
            "score"
        )
        .reset_index(
            drop=True
        )
    )


    summary.to_csv(
        OUTPUT_FILE,
        index=False
    )


    print()

    print(
        "=" * 100
    )

    print(
        "CRUISE CONTROL TUNING RESULTS"
    )

    print(
        "=" * 100
    )


    print(
        summary.to_string(
            index=False
        )
    )


    best = (
        summary.iloc[0]
    )


    print()

    print(
        "BEST CONFIGURATION"
    )

    print(
        "------------------"
    )


    print(
        f"Kp: "
        f"{best['kp']}"
    )


    print(
        f"Ki: "
        f"{best['ki']}"
    )


    print(
        f"Rise time: "
        f"{best['rise_time_s']:.3f} s"
    )


    if pd.isna(
        best[
            "settling_time_s"
        ]
    ):

        print(
            "Settling time: not settled"
        )

    else:

        print(
            f"Settling time: "
            f"{best['settling_time_s']:.3f} s"
        )


    print(
        f"Overshoot: "
        f"{best['overshoot_percent']:.3f}%"
    )


    print(
        f"Steady-state error: "
        f"{best['steady_state_error_mps']:.4f} m/s"
    )


    print(
        f"Final error: "
        f"{best['final_error_mps']:.4f} m/s"
    )


    print(
        f"Score: "
        f"{best['score']:.4f}"
    )


    print()

    print(
        "Summary saved to:",
        OUTPUT_FILE
    )


    return 0


if __name__ == "__main__":

    sys.exit(
        main()
    )