from pathlib import Path
import math
import re
import sys

import pandas as pd


RESULTS_DIR = Path("results")

SUMMARY_FILE = (
    RESULTS_DIR
    / "turn_sweep_summary.csv"
)

COMPARISON_FILE = (
    RESULTS_DIR
    / "integration_method_comparison.csv"
)

SCENARIO_PATTERN = re.compile(
    r"^turn_"
    r"(euler|rk4)_"
    r"v(\d+)_(\d+)_"
    r"s(\d+)_(\d+)$",
    re.IGNORECASE,
)


def parse_scenario_name(
    file_stem
):
    """
    Example:

    turn_euler_v10_0_s0_03

    becomes:

    method   = Euler
    velocity = 10.0
    steering = 0.03
    """

    match = SCENARIO_PATTERN.match(
        file_stem
    )


    if match is None:
        print(
            f"Skipping unrecognized sweep filename: "
            f"{file_stem}"
        )

        return None


    method_text = (
        match.group(1)
        .lower()
    )


    if method_text == "euler":

        integration_method = (
            "Euler"
        )

    else:

        integration_method = (
            "RK4"
        )


    velocity = float(
        f"{match.group(2)}."
        f"{match.group(3)}"
    )


    steering = float(
        f"{match.group(4)}."
        f"{match.group(5)}"
    )


    return {
        "integration_method":
            integration_method,

        "initial_velocity_mps":
            velocity,

        "steering_angle_rad":
            steering,
    }

def calculate_expected_radius(
    wheelbase,
    steering_angle
):

    if abs(
        steering_angle
    ) < 1e-12:

        return math.inf


    return (
        wheelbase
        /
        math.tan(
            steering_angle
        )
    )


# ============================================================
# Measured turning radius
# ============================================================

def calculate_measured_radius(
    df,
    expected_radius
):

    if math.isinf(
        expected_radius
    ):

        return math.inf


    center_x = 0.0

    center_y = (
        expected_radius
    )


    radius_samples = (
        (
            df["x"]
            -
            center_x
        )
        ** 2

        +

        (
            df["y"]
            -
            center_y
        )
        ** 2
    ) ** 0.5


    return (
        radius_samples.mean()
    )

def analyze_scenario(
    csv_file
):

    metadata = (
        parse_scenario_name(
            csv_file.stem
        )
    )


    if metadata is None:

        return None


    print(
        f"Reading: {csv_file.name}"
    )


    try:

        df = pd.read_csv(
            csv_file
        )

    except Exception as error:

        print(
            f"Could not read "
            f"{csv_file}: {error}"
        )

        return None


    required_columns = {
        "time",
        "x",
        "y",
        "velocity",
        "heading",
        "steering_angle",
        "wheelbase",
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
            f"Skipping {csv_file.name}. "
            f"Missing columns: "
            f"{sorted(missing_columns)}"
        )

        return None


    if df.empty:

        print(
            f"Skipping empty file: "
            f"{csv_file.name}"
        )

        return None


    steering_angle = (
        metadata[
            "steering_angle_rad"
        ]
    )


    wheelbase = float(
        df[
            "wheelbase"
        ].iloc[0]
    )


    expected_radius = (
        calculate_expected_radius(
            wheelbase,
            steering_angle
        )
    )


    measured_radius = (
        calculate_measured_radius(
            df,
            expected_radius
        )
    )


    radius_error = abs(
        measured_radius
        -
        abs(
            expected_radius
        )
    )


    radius_error_percent = (
        radius_error
        /
        abs(
            expected_radius
        )
        *
        100.0
    )


    radius_tolerance = 1.0


    result = (
        "PASS"
        if radius_error
        <= radius_tolerance
        else "FAIL"
    )


    return {

        "scenario":
            csv_file.stem,

        "integration_method":
            metadata[
                "integration_method"
            ],

        "initial_velocity_mps":
            metadata[
                "initial_velocity_mps"
            ],

        "steering_angle_rad":
            steering_angle,

        "expected_radius_m":
            expected_radius,

        "measured_radius_m":
            measured_radius,

        "radius_error_m":
            radius_error,

        "radius_error_percent":
            radius_error_percent,

        "final_x_m":
            float(
                df[
                    "x"
                ].iloc[-1]
            ),

        "final_y_m":
            float(
                df[
                    "y"
                ].iloc[-1]
            ),

        "final_heading_rad":
            float(
                df[
                    "heading"
                ].iloc[-1]
            ),

        "result":
            result,
    }

def build_method_comparison(
    summary
):

    comparison_rows = []


    grouped = (
        summary.groupby(
            [
                "initial_velocity_mps",
                "steering_angle_rad",
            ]
        )
    )


    for (
        velocity,
        steering
    ), group in grouped:


        euler_rows = (
            group[
                group[
                    "integration_method"
                ]
                ==
                "Euler"
            ]
        )


        rk4_rows = (
            group[
                group[
                    "integration_method"
                ]
                ==
                "RK4"
            ]
        )


        if euler_rows.empty:

            print(
                f"Missing Euler result for "
                f"v={velocity}, "
                f"steering={steering}"
            )

            continue


        if rk4_rows.empty:

            print(
                f"Missing RK4 result for "
                f"v={velocity}, "
                f"steering={steering}"
            )

            continue


        euler = (
            euler_rows.iloc[0]
        )

        rk4 = (
            rk4_rows.iloc[0]
        )


        dx = (
            euler[
                "final_x_m"
            ]
            -
            rk4[
                "final_x_m"
            ]
        )


        dy = (
            euler[
                "final_y_m"
            ]
            -
            rk4[
                "final_y_m"
            ]
        )


        final_position_difference = (
            math.sqrt(
                dx * dx
                +
                dy * dy
            )
        )


        heading_difference = abs(
            euler[
                "final_heading_rad"
            ]
            -
            rk4[
                "final_heading_rad"
            ]
        )


        euler_radius_error = (
            euler[
                "radius_error_m"
            ]
        )


        rk4_radius_error = (
            rk4[
                "radius_error_m"
            ]
        )


        if (
            rk4_radius_error
            <
            euler_radius_error
        ):

            more_accurate_method = (
                "RK4"
            )


        elif (
            euler_radius_error
            <
            rk4_radius_error
        ):

            more_accurate_method = (
                "Euler"
            )


        else:

            more_accurate_method = (
                "Equal"
            )


        comparison_rows.append(
            {

                "initial_velocity_mps":
                    velocity,

                "steering_angle_rad":
                    steering,

                "expected_radius_m":
                    euler[
                        "expected_radius_m"
                    ],

                "euler_radius_error_m":
                    euler_radius_error,

                "rk4_radius_error_m":
                    rk4_radius_error,

                "euler_final_x_m":
                    euler[
                        "final_x_m"
                    ],

                "euler_final_y_m":
                    euler[
                        "final_y_m"
                    ],

                "rk4_final_x_m":
                    rk4[
                        "final_x_m"
                    ],

                "rk4_final_y_m":
                    rk4[
                        "final_y_m"
                    ],

                "final_position_difference_m":
                    final_position_difference,

                "heading_difference_rad":
                    heading_difference,

                "more_accurate_method":
                    more_accurate_method,
            }
        )


    return pd.DataFrame(
        comparison_rows
    )


def main():

    RESULTS_DIR.mkdir(
        exist_ok=True
    )


    euler_files = sorted(
        RESULTS_DIR.glob(
            "turn_euler_*.csv"
        )
    )


    rk4_files = sorted(
        RESULTS_DIR.glob(
            "turn_rk4_*.csv"
        )
    )


    result_files = (
        euler_files
        +
        rk4_files
    )


    print()
    print(
        "Sweep file discovery"
    )

    print(
        "--------------------"
    )

    print(
        f"Euler files found: "
        f"{len(euler_files)}"
    )

    print(
        f"RK4 files found: "
        f"{len(rk4_files)}"
    )

    print(
        f"Total files found: "
        f"{len(result_files)}"
    )


    if not result_files:

        print()
        print(
            "No Euler/RK4 sweep "
            "result files found."
        )

        print(
            "Expected filenames like:"
        )

        print(
            "turn_euler_v10_0_s0_03.csv"
        )

        print(
            "turn_rk4_v10_0_s0_03.csv"
        )

        return 1


    rows = []


    print()
    print(
        "Processing sweep files"
    )

    print(
        "----------------------"
    )


    for csv_file in result_files:

        result = (
            analyze_scenario(
                csv_file
            )
        )


        if result is not None:

            rows.append(
                result
            )


    print()
    print(
        f"Valid sweep results: "
        f"{len(rows)}"
    )


    if not rows:

        print()
        print(
            "No valid sweep scenarios found."
        )

        return 1


    summary = pd.DataFrame(
        rows
    )


    summary = (
        summary.sort_values(
            [
                "initial_velocity_mps",
                "steering_angle_rad",
                "integration_method",
            ]
        )
        .reset_index(
            drop=True
        )
    )


    summary.to_csv(
        SUMMARY_FILE,
        index=False
    )


    print()
    print(
        "=" * 120
    )

    print(
        "TURN SWEEP REPORT"
    )

    print(
        "=" * 120
    )

    print()

    print(
        summary.to_string(
            index=False
        )
    )


    passed = int(
        (
            summary[
                "result"
            ]
            ==
            "PASS"
        ).sum()
    )


    failed = int(
        (
            summary[
                "result"
            ]
            ==
            "FAIL"
        ).sum()
    )


    print()
    print(
        "Sweep validation summary"
    )

    print(
        "------------------------"
    )

    print(
        f"Total runs: "
        f"{len(summary)}"
    )

    print(
        f"Passed: "
        f"{passed}"
    )

    print(
        f"Failed: "
        f"{failed}"
    )

    comparison = (
        build_method_comparison(
            summary
        )
    )


    if comparison.empty:

        print()
        print(
            "No complete Euler/RK4 "
            "scenario pairs were found."
        )

        return 1


    comparison = (
        comparison.sort_values(
            [
                "initial_velocity_mps",
                "steering_angle_rad",
            ]
        )
        .reset_index(
            drop=True
        )
    )


    comparison.to_csv(
        COMPARISON_FILE,
        index=False
    )


    print()
    print(
        "=" * 120
    )

    print(
        "EULER VS RK4 SWEEP COMPARISON"
    )

    print(
        "=" * 120
    )

    print()

    print(
        comparison.to_string(
            index=False
        )
    )


    print()
    print(
        "Generated files"
    )

    print(
        "---------------"
    )

    print(
        SUMMARY_FILE
    )

    print(
        COMPARISON_FILE
    )


    if failed > 0:

        print()
        print(
            "OVERALL SWEEP RESULT: FAIL"
        )

        return 1


    print()
    print(
        "OVERALL SWEEP RESULT: PASS"
    )

    return 0


if __name__ == "__main__":

    sys.exit(
        main()
    )