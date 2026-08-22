from pathlib import Path
import math
import sys

import matplotlib.pyplot as plt
import pandas as pd


RESULTS_DIRECTORY = Path("results")


def is_controller_scenario(data):
    required_columns = {
        "target_velocity",
        "speed_error",
    }

    if not required_columns.issubset(
        data.columns
    ):
        return False

    if data.empty:
        return False

    target_velocity = pd.to_numeric(
        data["target_velocity"],
        errors="coerce"
    )

    velocity = pd.to_numeric(
        data["velocity"],
        errors="coerce"
    )

    speed_error = pd.to_numeric(
        data["speed_error"],
        errors="coerce"
    )

    if (
        target_velocity.isnull().all()
        or velocity.isnull().all()
        or speed_error.isnull().all()
    ):
        return False

    target_difference = (
        target_velocity
        -
        velocity
    ).abs()

    maximum_target_difference = (
        target_difference.max()
    )

    maximum_speed_error = (
        speed_error.abs().max()
    )

    tolerance = 1e-9

    return (
        maximum_target_difference
        >
        tolerance
        or
        maximum_speed_error
        >
        tolerance
    )


def validate_common(data):
    checks = {}

    velocity = pd.to_numeric(
        data["velocity"],
        errors="coerce"
    )

    checks[
        "No negative velocity"
    ] = bool(
        (velocity >= -1e-9).all()
    )

    return checks


def measured_acceleration(data):
    if len(data) < 2:
        return 0.0

    time = pd.to_numeric(
        data["time"],
        errors="coerce"
    )

    velocity = pd.to_numeric(
        data["velocity"],
        errors="coerce"
    )

    delta_time = (
        time.iloc[-1]
        -
        time.iloc[0]
    )

    if abs(delta_time) < 1e-12:
        return 0.0

    return float(
        (
            velocity.iloc[-1]
            -
            velocity.iloc[0]
        )
        /
        delta_time
    )


def commanded_acceleration(data):
    if (
        "commanded_acceleration"
        not in data.columns
    ):
        return 0.0

    acceleration = pd.to_numeric(
        data["commanded_acceleration"],
        errors="coerce"
    )

    acceleration = (
        acceleration
        .dropna()
    )

    if acceleration.empty:
        return 0.0

    return float(
        acceleration.mean()
    )


def validate_scenario(
    scenario_name,
    data
):
    checks = validate_common(
        data
    )

    command_acceleration = (
        commanded_acceleration(
            data
        )
    )

    actual_acceleration = (
        measured_acceleration(
            data
        )
    )

    acceleration_error = abs(
        command_acceleration
        -
        actual_acceleration
    )

    acceleration_tolerance = 0.15

    acceleration_check = (
        acceleration_error
        <=
        acceleration_tolerance
    )

    specific_check_found = False


    if scenario_name == "acceleration":
        specific_check_found = True

        checks[
            "Vehicle accelerated"
        ] = bool(
            data["velocity"].iloc[-1]
            >
            data["velocity"].iloc[0]
        )


    elif scenario_name == "emergency_braking":
        specific_check_found = True

        checks[
            "Vehicle slowed down"
        ] = bool(
            data["velocity"].iloc[-1]
            <
            data["velocity"].iloc[0]
        )

        checks[
            "Vehicle reached zero speed"
        ] = bool(
            abs(
                data["velocity"].iloc[-1]
            )
            <=
            1e-6
        )


    elif scenario_name == "constant_turn":
        specific_check_found = True

        checks[
            "Heading changed"
        ] = bool(
            abs(
                data["heading"].iloc[-1]
                -
                data["heading"].iloc[0]
            )
            >
            1e-6
        )

        checks[
            "Lateral displacement occurred"
        ] = bool(
            abs(
                data["y"].iloc[-1]
                -
                data["y"].iloc[0]
            )
            >
            1e-6
        )


    elif scenario_name == "lane_change":
        specific_check_found = True

        checks[
            "Lateral displacement occurred"
        ] = bool(
            abs(
                data["y"].iloc[-1]
                -
                data["y"].iloc[0]
            )
            >
            1e-6
        )


    # ==================================================
    # Print validation
    # ==================================================

    print()
    print("Validation")
    print("----------")

    for name, passed in checks.items():
        print(
            f"{name}: {passed}"
        )

    print(
        f"Commanded acceleration: "
        f"{command_acceleration:.6f} m/s^2"
    )

    print(
        f"Measured acceleration: "
        f"{actual_acceleration:.6f} m/s^2"
    )

    print(
        f"Acceleration error: "
        f"{acceleration_error:.6f} m/s^2"
    )

    print(
        f"Acceleration within tolerance: "
        f"{acceleration_check}"
    )

    if not specific_check_found:
        print(
            "No scenario-specific validation rule found."
        )

    checks[
        "Acceleration within tolerance"
    ] = acceleration_check

    return checks


def plot_trajectory(
    scenario_name,
    data
):
    output_file = (
        RESULTS_DIRECTORY
        /
        f"{scenario_name}_trajectory.png"
    )

    plt.figure(
        figsize=(7, 5)
    )

    plt.plot(
        data["x"],
        data["y"]
    )

    plt.xlabel(
        "X position [m]"
    )

    plt.ylabel(
        "Y position [m]"
    )

    plt.title(
        f"Trajectory: {scenario_name}"
    )

    plt.axis(
        "equal"
    )

    plt.grid(
        True
    )

    plt.tight_layout()

    plt.savefig(
        output_file,
        dpi=150
    )

    plt.close()

    print(
        f"Trajectory plot saved to: "
        f"{output_file}"
    )


def process_standard_file(
    file_path
):
    scenario_name = (
        file_path.stem
    )

    data = pd.read_csv(
        file_path
    )

    required_columns = {
        "time",
        "x",
        "y",
        "velocity",
        "heading",
    }

    missing_columns = (
        required_columns
        -
        set(data.columns)
    )

    if missing_columns:
        print()
        print(
            f"Skipping {file_path.name}: "
            f"missing columns "
            f"{sorted(missing_columns)}"
        )

        return False


    print()
    print("=" * 55)

    print(
        f"Scenario: "
        f"{scenario_name}"
    )

    print("=" * 55)


    print(
        f"Initial velocity: "
        f"{data['velocity'].iloc[0]:.6g} m/s"
    )

    print(
        f"Final velocity: "
        f"{data['velocity'].iloc[-1]:.6g} m/s"
    )

    print(
        f"Maximum velocity: "
        f"{data['velocity'].max():.6g} m/s"
    )

    print(
        f"Final X: "
        f"{data['x'].iloc[-1]:.6g} m"
    )

    print(
        f"Final Y: "
        f"{data['y'].iloc[-1]:.6g} m"
    )

    print(
        f"Final heading: "
        f"{data['heading'].iloc[-1]:.6g} rad"
    )

    print(
        f"Simulation duration: "
        f"{data['time'].iloc[-1]:.6g} s"
    )


    checks = validate_scenario(
        scenario_name,
        data
    )


    plot_trajectory(
        scenario_name,
        data
    )


    passed = all(
        checks.values()
    )


    print()

    print(
        "RESULT:",
        "PASS"
        if passed
        else "FAIL"
    )


    return passed


def main():
    if not RESULTS_DIRECTORY.exists():
        print(
            "ERROR: results directory does not exist."
        )

        return 1


    csv_files = sorted(
        RESULTS_DIRECTORY.glob(
            "*.csv"
        )
    )


    ignored_files = {
        "turn_sweep_summary.csv",
        "integration_method_comparison.csv",
        "timestep_sensitivity.csv",
        "cruise_control_tuning_summary.csv",
        "cruise_tuning_current.csv",
    }


    standard_files = []


    for file_path in csv_files:
        if (
            file_path.name
            in ignored_files
        ):
            continue


        try:
            data = pd.read_csv(
                file_path
            )
        except Exception as error:
            print(
                f"Skipping {file_path.name}: "
                f"{error}"
            )

            continue


        required_columns = {
            "time",
            "x",
            "y",
            "velocity",
            "heading",
        }


        if not required_columns.issubset(
            data.columns
        ):
            continue


        if is_controller_scenario(
            data
        ):
            print(
                "Controller scenario detected: "
                f"{file_path.name} "
                "(validated separately)"
            )

            continue


        standard_files.append(
            file_path
        )


    print()

    print(
        f"Found {len(standard_files)} "
        "standard simulation result files."
    )


    passed_count = 0
    failed_count = 0


    for file_path in standard_files:
        passed = process_standard_file(
            file_path
        )

        if passed:
            passed_count += 1
        else:
            failed_count += 1


    print()

    print("=" * 55)

    print(
        "STANDARD SCENARIO VALIDATION SUMMARY"
    )

    print("=" * 55)


    print(
        f"Passed: "
        f"{passed_count}"
    )

    print(
        f"Failed: "
        f"{failed_count}"
    )


    overall_pass = (
        failed_count == 0
        and
        len(standard_files) > 0
    )


    print(
        "OVERALL STANDARD SCENARIO VALIDATION:",
        "PASS"
        if overall_pass
        else "FAIL"
    )


    return (
        0
        if overall_pass
        else 1
    )


if __name__ == "__main__":
    sys.exit(
        main()
    )