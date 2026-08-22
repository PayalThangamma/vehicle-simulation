from pathlib import Path
import sys

import matplotlib.pyplot as plt
import pandas as pd


RESULT_FILE = Path(
    "results/cruise_control.csv"
)

PLOT_FILE = Path(
    "results/cruise_control_tracking.png"
)


def main():

    if not RESULT_FILE.exists():

        print(
            f"Missing cruise-control result: "
            f"{RESULT_FILE}"
        )

        return 1


    df = pd.read_csv(
        RESULT_FILE
    )


    required_columns = {
        "time",
        "velocity",
        "commanded_acceleration",
        "target_velocity",
        "speed_error",
    }


    missing_columns = (
        required_columns
        -
        set(
            df.columns
        )
    )


    if missing_columns:

        print(
            "Missing required columns:"
        )

        print(
            sorted(
                missing_columns
            )
        )

        return 1


    if df.empty:

        print(
            "Cruise-control result is empty."
        )

        return 1


    initial_velocity = float(
        df["velocity"].iloc[0]
    )

    final_velocity = float(
        df["velocity"].iloc[-1]
    )

    target_velocity = float(
        df["target_velocity"].iloc[-1]
    )

    maximum_velocity = float(
        df["velocity"].max()
    )

    final_error = abs(
        target_velocity
        -
        final_velocity
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
        df["velocity"]
        >=
        lower_threshold
    ]


    upper_rows = df[
        df["velocity"]
        >=
        upper_threshold
    ]


    rise_time = None


    if (
        not lower_rows.empty
        and
        not upper_rows.empty
    ):

        lower_time = float(
            lower_rows["time"].iloc[0]
        )

        upper_time = float(
            upper_rows["time"].iloc[0]
        )

        rise_time = (
            upper_time
            -
            lower_time
        )

    if target_velocity > 0.0:

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

    else:

        overshoot_percent = 0.0


    settling_band = (
        0.02
        *
        max(
            abs(
                target_velocity
            ),
            1.0
        )
    )


    settling_time = None


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

    steady_start_index = int(
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
            steady_start_index:
        ]
        .abs()
        .mean()
    )

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


    maximum_acceleration_limit = 3.0

    minimum_acceleration_limit = -4.0


    saturation_tolerance = 1e-9


    positive_saturation_count = int(
        (
            df[
                "commanded_acceleration"
            ]
            >=
            (
                maximum_acceleration_limit
                -
                saturation_tolerance
            )
        ).sum()
    )


    negative_saturation_count = int(
        (
            df[
                "commanded_acceleration"
            ]
            <=
            (
                minimum_acceleration_limit
                +
                saturation_tolerance
            )
        ).sum()
    )


    final_error_ok = (
        final_error
        <=
        0.5
    )


    overshoot_ok = (
        overshoot_percent
        <=
        5.0
    )


    steady_state_error_ok = (
        steady_state_error
        <=
        0.5
    )


    settling_ok = (
        settling_time
        is not None
    )


    overall_pass = (
        final_error_ok
        and
        overshoot_ok
        and
        steady_state_error_ok
        and
        settling_ok
    )

    print()
    print(
        "=" * 60
    )

    print(
        "CRUISE CONTROL ANALYSIS"
    )

    print(
        "=" * 60
    )


    print(
        f"Initial velocity: "
        f"{initial_velocity:.3f} m/s"
    )

    print(
        f"Target velocity: "
        f"{target_velocity:.3f} m/s"
    )

    print(
        f"Final velocity: "
        f"{final_velocity:.3f} m/s"
    )

    print(
        f"Maximum velocity: "
        f"{maximum_velocity:.3f} m/s"
    )

    print()


    if rise_time is None:

        print(
            "Rise time: not reached"
        )

    else:

        print(
            f"Rise time (10%-90%): "
            f"{rise_time:.3f} s"
        )


    if settling_time is None:

        print(
            "Settling time (2% band): "
            "not settled"
        )

    else:

        print(
            f"Settling time (2% band): "
            f"{settling_time:.3f} s"
        )


    print(
        f"Overshoot: "
        f"{overshoot_percent:.3f} %"
    )

    print(
        f"Final speed error: "
        f"{final_error:.3f} m/s"
    )

    print(
        f"Steady-state error: "
        f"{steady_state_error:.3f} m/s"
    )


    print()
    print(
        "Controller effort"
    )

    print(
        "-----------------"
    )


    print(
        f"Maximum acceleration command: "
        f"{maximum_acceleration:.3f} m/s^2"
    )

    print(
        f"Minimum acceleration command: "
        f"{minimum_acceleration:.3f} m/s^2"
    )

    print(
        f"Positive saturation samples: "
        f"{positive_saturation_count}"
    )

    print(
        f"Negative saturation samples: "
        f"{negative_saturation_count}"
    )


    print()
    print(
        "Validation"
    )

    print(
        "----------"
    )


    print(
        "Final speed error <= 0.5 m/s:",
        final_error_ok
    )

    print(
        "Overshoot <= 5%:",
        overshoot_ok
    )

    print(
        "Steady-state error <= 0.5 m/s:",
        steady_state_error_ok
    )

    print(
        "Settled inside 2% band:",
        settling_ok
    )

    plt.figure()

    plt.plot(
        df["time"],
        df["velocity"],
        label="Vehicle speed"
    )

    plt.plot(
        df["time"],
        df["target_velocity"],
        linestyle="--",
        label="Target speed"
    )

    plt.xlabel(
        "Time (s)"
    )

    plt.ylabel(
        "Velocity (m/s)"
    )

    plt.title(
        "Cruise Control Speed Tracking"
    )

    plt.grid(
        True
    )

    plt.legend()

    plt.tight_layout()

    plt.savefig(
        PLOT_FILE,
        dpi=150
    )

    plt.close()


    print()
    print(
        "Tracking plot saved to:",
        PLOT_FILE
    )


    print()


    if overall_pass:

        print(
            "CRUISE CONTROL RESULT: PASS"
        )

        return 0


    print(
        "CRUISE CONTROL RESULT: FAIL"
    )

    return 1


if __name__ == "__main__":

    sys.exit(
        main()
    )