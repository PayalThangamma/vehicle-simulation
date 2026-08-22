from pathlib import Path
import sys

import matplotlib.pyplot as plt
import pandas as pd


RESULTS_DIR = Path("results")

INPUT_FILE = (
    RESULTS_DIR
    / "integration_method_comparison.csv"
)

ERROR_PLOT_FILE = (
    RESULTS_DIR
    / "euler_rk4_position_difference.png"
)

RADIUS_PLOT_FILE = (
    RESULTS_DIR
    / "euler_rk4_radius_error.png"
)


def load_results():
    if not INPUT_FILE.exists():

        print(
            f"Missing comparison file: "
            f"{INPUT_FILE}"
        )

        return None


    try:
        df = pd.read_csv(
            INPUT_FILE
        )

    except Exception as error:

        print(
            f"Could not read "
            f"{INPUT_FILE}: {error}"
        )

        return None


    required_columns = {
        "initial_velocity_mps",
        "steering_angle_rad",
        "euler_radius_error_m",
        "rk4_radius_error_m",
        "final_position_difference_m",
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
            "Comparison file is missing "
            f"columns: {sorted(missing_columns)}"
        )

        return None


    if df.empty:

        print(
            "Comparison file is empty."
        )

        return None


    return df


def plot_position_difference(
    df
):
    plt.figure()


    for velocity in sorted(
        df[
            "initial_velocity_mps"
        ].unique()
    ):

        subset = (
            df[
                df[
                    "initial_velocity_mps"
                ]
                ==
                velocity
            ]
            .sort_values(
                "steering_angle_rad"
            )
        )


        plt.plot(
            subset[
                "steering_angle_rad"
            ],
            subset[
                "final_position_difference_m"
            ],
            marker="o",
            label=(
                f"{velocity:g} m/s"
            ),
        )


    plt.xlabel(
        "Steering angle (rad)"
    )

    plt.ylabel(
        "Euler vs RK4 final position difference (m)"
    )

    plt.title(
        "Euler vs RK4 Position Difference"
    )

    plt.grid(
        True
    )

    plt.legend(
        title="Vehicle speed"
    )

    plt.tight_layout()

    plt.savefig(
        ERROR_PLOT_FILE,
        dpi=150
    )

    plt.close()


    print(
        "Position comparison plot saved to:",
        ERROR_PLOT_FILE
    )


def plot_radius_error(
    df
):
    sorted_df = (
        df.sort_values(
            [
                "initial_velocity_mps",
                "steering_angle_rad",
            ]
        )
        .reset_index(
            drop=True
        )
    )


    labels = []


    for _, row in sorted_df.iterrows():

        label = (
            f"v={row['initial_velocity_mps']:g}\n"
            f"s={row['steering_angle_rad']:.2f}"
        )

        labels.append(
            label
        )


    x_positions = list(
        range(
            len(
                sorted_df
            )
        )
    )


    bar_width = 0.35


    euler_positions = [
        position
        -
        bar_width / 2
        for position in x_positions
    ]


    rk4_positions = [
        position
        +
        bar_width / 2
        for position in x_positions
    ]


    plt.figure(
        figsize=(12, 6)
    )


    plt.bar(
        euler_positions,
        sorted_df[
            "euler_radius_error_m"
        ],
        width=bar_width,
        label="Euler",
    )


    plt.bar(
        rk4_positions,
        sorted_df[
            "rk4_radius_error_m"
        ],
        width=bar_width,
        label="RK4",
    )


    plt.xlabel(
        "Simulation configuration"
    )

    plt.ylabel(
        "Turning-radius error (m)"
    )

    plt.title(
        "Euler vs RK4 Turning-Radius Accuracy"
    )


    plt.xticks(
        x_positions,
        labels,
    )


    plt.grid(
        True,
        axis="y"
    )

    plt.legend()

    plt.tight_layout()

    plt.savefig(
        RADIUS_PLOT_FILE,
        dpi=150
    )

    plt.close()


    print(
        "Radius comparison plot saved to:",
        RADIUS_PLOT_FILE
    )


def print_summary(
    df
):
    print()
    print(
        "=" * 70
    )

    print(
        "EULER VS RK4 SWEEP SUMMARY"
    )

    print(
        "=" * 70
    )


    mean_position_difference = (
        df[
            "final_position_difference_m"
        ]
        .mean()
    )


    maximum_position_difference = (
        df[
            "final_position_difference_m"
        ]
        .max()
    )


    minimum_position_difference = (
        df[
            "final_position_difference_m"
        ]
        .min()
    )


    mean_euler_radius_error = (
        df[
            "euler_radius_error_m"
        ]
        .mean()
    )


    mean_rk4_radius_error = (
        df[
            "rk4_radius_error_m"
        ]
        .mean()
    )


    print(
        "Mean final position difference:",
        f"{mean_position_difference:.6f} m"
    )


    print(
        "Maximum final position difference:",
        f"{maximum_position_difference:.6f} m"
    )


    print(
        "Minimum final position difference:",
        f"{minimum_position_difference:.6f} m"
    )


    print()

    print(
        "Mean Euler radius error:",
        f"{mean_euler_radius_error:.6f} m"
    )


    print(
        "Mean RK4 radius error:",
        f"{mean_rk4_radius_error:.6f} m"
    )


    if (
        mean_rk4_radius_error
        <
        mean_euler_radius_error
    ):

        print()

        print(
            "RK4 produced lower average "
            "turning-radius error."
        )


    elif (
        mean_euler_radius_error
        <
        mean_rk4_radius_error
    ):

        print()

        print(
            "Euler produced lower average "
            "turning-radius error."
        )


    else:

        print()

        print(
            "Euler and RK4 produced equal "
            "average turning-radius error."
        )


def main():
    df = load_results()


    if df is None:

        return 1


    print(
        "\nIntegration comparison results:"
    )


    print(
        df.to_string(
            index=False
        )
    )


    plot_position_difference(
        df
    )


    plot_radius_error(
        df
    )


    print_summary(
        df
    )


    return 0


if __name__ == "__main__":

    sys.exit(
        main()
    )