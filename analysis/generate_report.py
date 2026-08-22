from pathlib import Path

import pandas as pd


RESULTS_DIR = Path("results")

REPORT_FILE = (
    RESULTS_DIR
    / "simulation_report.md"
)


def read_csv_if_exists(
    path
):
    if path.exists():
        return pd.read_csv(
            path
        )

    return None


def add_overview_section(
    lines
):
    lines.append(
        "# Vehicle Simulation Factory Report"
    )

    lines.append("")

    lines.append(
        "## Overview"
    )

    lines.append("")

    lines.append(
        "This project implements a configurable "
        "vehicle simulation and automated validation "
        "pipeline using C++, CMake and Python."
    )

    lines.append("")

    lines.append(
        "The simulation engine uses a simplified "
        "kinematic bicycle model and supports "
        "acceleration, emergency braking, constant-turn "
        "and lane-change scenarios."
    )

    lines.append("")

    lines.append(
        "Both explicit Euler and fourth-order "
        "Runge-Kutta (RK4) numerical integration "
        "methods are supported."
    )

    lines.append("")


def add_architecture_section(
    lines
):
    lines.append(
        "## Simulation Architecture"
    )

    lines.append("")

    lines.append("```text")

    lines.append(
        "JSON Scenario Configuration"
    )

    lines.append(
        "            |"
    )

    lines.append(
        "            v"
    )

    lines.append(
        "C++ Scenario Loader"
    )

    lines.append(
        "            |"
    )

    lines.append(
        "            v"
    )

    lines.append(
        "C++ Simulation Engine"
    )

    lines.append(
        "            |"
    )

    lines.append(
        "            v"
    )

    lines.append(
        "Kinematic Bicycle Model"
    )

    lines.append(
        "            |"
    )

    lines.append(
        "            v"
    )

    lines.append(
        "Euler / RK4 Integration"
    )

    lines.append(
        "            |"
    )

    lines.append(
        "            v"
    )

    lines.append(
        "CSV Simulation Logs"
    )

    lines.append(
        "            |"
    )

    lines.append(
        "            v"
    )

    lines.append(
        "Python Reprocessing"
    )

    lines.append(
        "            |"
    )

    lines.append(
        "            v"
    )

    lines.append(
        "Validation / Sweeps / Regression"
    )

    lines.append(
        "            |"
    )

    lines.append(
        "            v"
    )

    lines.append(
        "Plots + Automated Report"
    )

    lines.append("```")

    lines.append("")


def add_validation_section(
    lines
):
    lines.append(
        "## Validation Strategy"
    )

    lines.append("")

    lines.append(
        "- C++ unit tests validate individual "
        "vehicle-model behavior."
    )

    lines.append(
        "- Scenario-level validation checks "
        "acceleration, braking, turning and "
        "lane-change behavior."
    )

    lines.append(
        "- Parameter sweeps test the vehicle model "
        "across multiple speeds and steering commands."
    )

    lines.append(
        "- Regression tests compare current outputs "
        "against known-good baselines."
    )

    lines.append(
        "- Euler/RK4 comparisons evaluate numerical "
        "integration behavior."
    )

    lines.append(
        "- Timestep sensitivity analysis evaluates "
        "numerical convergence."
    )

    lines.append(
        "- Multi-configuration Euler/RK4 sweeps compare "
        "integration accuracy across the scenario matrix."
    )

    lines.append("")


def add_turn_sweep_section(
    lines
):
    lines.append(
        "## Turn Sweep Results"
    )

    lines.append("")

    sweep_file = (
        RESULTS_DIR
        / "turn_sweep_summary.csv"
    )

    sweep = read_csv_if_exists(
        sweep_file
    )


    if sweep is None:

        lines.append(
            "Turn sweep summary not found."
        )

        lines.append("")

        return


    total = len(
        sweep
    )


    if "result" in sweep.columns:

        passed = int(
            (
                sweep["result"]
                ==
                "PASS"
            ).sum()
        )

        failed = int(
            (
                sweep["result"]
                ==
                "FAIL"
            ).sum()
        )


        lines.append(
            f"- Total runs: {total}"
        )

        lines.append(
            f"- Passed: {passed}"
        )

        lines.append(
            f"- Failed: {failed}"
        )

        lines.append("")


    required_columns = [
        "integration_method",
        "initial_velocity_mps",
        "steering_angle_rad",
        "expected_radius_m",
        "measured_radius_m",
        "radius_error_m",
        "result",
    ]


    if all(
        column in sweep.columns
        for column in required_columns
    ):

        lines.append(
            "| Method | Velocity (m/s) | "
            "Steering (rad) | Expected Radius (m) | "
            "Measured Radius (m) | Error (m) | Result |"
        )

        lines.append(
            "|:---|---:|---:|---:|---:|---:|:---:|"
        )


        for _, row in sweep.iterrows():

            lines.append(
                f"| {row['integration_method']} "
                f"| {row['initial_velocity_mps']:.2f} "
                f"| {row['steering_angle_rad']:.3f} "
                f"| {row['expected_radius_m']:.2f} "
                f"| {row['measured_radius_m']:.2f} "
                f"| {row['radius_error_m']:.4f} "
                f"| {row['result']} |"
            )


    lines.append("")


def add_integration_sweep_section(
    lines
):
    lines.append(
        "## Euler vs RK4 Parameter Sweep"
    )

    lines.append("")

    comparison_file = (
        RESULTS_DIR
        / "integration_method_comparison.csv"
    )


    comparison = read_csv_if_exists(
        comparison_file
    )


    if comparison is None:

        lines.append(
            "Euler/RK4 integration comparison "
            "results were not found."
        )

        lines.append("")

        return


    required_columns = [
        "initial_velocity_mps",
        "steering_angle_rad",
        "euler_radius_error_m",
        "rk4_radius_error_m",
        "final_position_difference_m",
        "heading_difference_rad",
        "more_accurate_method",
    ]


    if not all(
        column in comparison.columns
        for column in required_columns
    ):

        lines.append(
            "Integration comparison file does not "
            "contain all required columns."
        )

        lines.append("")

        return


    lines.append(
        "Euler and RK4 were executed using identical "
        "vehicle configurations so that differences "
        "were caused only by the numerical integration "
        "method."
    )

    lines.append("")


    mean_euler_error = (
        comparison[
            "euler_radius_error_m"
        ]
        .mean()
    )


    mean_rk4_error = (
        comparison[
            "rk4_radius_error_m"
        ]
        .mean()
    )


    mean_position_difference = (
        comparison[
            "final_position_difference_m"
        ]
        .mean()
    )


    maximum_position_difference = (
        comparison[
            "final_position_difference_m"
        ]
        .max()
    )


    lines.append(
        f"- Mean Euler turning-radius error: "
        f"{mean_euler_error:.6f} m"
    )

    lines.append(
        f"- Mean RK4 turning-radius error: "
        f"{mean_rk4_error:.6f} m"
    )

    lines.append(
        f"- Mean Euler/RK4 final-position difference: "
        f"{mean_position_difference:.6f} m"
    )

    lines.append(
        f"- Maximum Euler/RK4 final-position difference: "
        f"{maximum_position_difference:.6f} m"
    )

    lines.append("")


    if (
        mean_rk4_error
        <
        mean_euler_error
    ):

        lines.append(
            "**Overall result:** RK4 produced a lower "
            "average turning-radius error than Euler."
        )


    elif (
        mean_euler_error
        <
        mean_rk4_error
    ):

        lines.append(
            "**Overall result:** Euler produced a lower "
            "average turning-radius error than RK4."
        )


    else:

        lines.append(
            "**Overall result:** Euler and RK4 produced "
            "equal average turning-radius error."
        )


    lines.append("")


    lines.append(
        "| Velocity (m/s) | Steering (rad) | "
        "Euler Radius Error (m) | "
        "RK4 Radius Error (m) | "
        "Position Difference (m) | "
        "Heading Difference (rad) | "
        "More Accurate |"
    )

    lines.append(
        "|---:|---:|---:|---:|---:|---:|:---:|"
    )


    for _, row in comparison.iterrows():

        lines.append(
            f"| {row['initial_velocity_mps']:.2f} "
            f"| {row['steering_angle_rad']:.3f} "
            f"| {row['euler_radius_error_m']:.6f} "
            f"| {row['rk4_radius_error_m']:.6f} "
            f"| {row['final_position_difference_m']:.6f} "
            f"| {row['heading_difference_rad']:.6f} "
            f"| {row['more_accurate_method']} |"
        )


    lines.append("")


    position_plot = (
        RESULTS_DIR
        / "euler_rk4_position_difference.png"
    )


    if position_plot.exists():

        lines.append(
            "### Final Position Difference"
        )

        lines.append("")

        lines.append(
            "![Euler vs RK4 Position Difference]"
            "(euler_rk4_position_difference.png)"
        )

        lines.append("")


    radius_plot = (
        RESULTS_DIR
        / "euler_rk4_radius_error.png"
    )


    if radius_plot.exists():

        lines.append(
            "### Turning-Radius Accuracy"
        )

        lines.append("")

        lines.append(
            "![Euler vs RK4 Radius Error]"
            "(euler_rk4_radius_error.png)"
        )

        lines.append("")


def add_timestep_section(
    lines
):
    lines.append(
        "## Timestep Sensitivity and Numerical Convergence"
    )

    lines.append("")


    sensitivity_file = (
        RESULTS_DIR
        / "timestep_sensitivity.csv"
    )


    df = read_csv_if_exists(
        sensitivity_file
    )


    if df is None:

        lines.append(
            "Timestep sensitivity results "
            "were not found."
        )

        lines.append("")

        return


    required_columns = [
        "dt",
        "euler_x",
        "euler_y",
        "rk4_x",
        "rk4_y",
        "position_error",
        "heading_error",
    ]


    if not all(
        column in df.columns
        for column in required_columns
    ):

        lines.append(
            "Timestep sensitivity file does not "
            "contain all required columns."
        )

        lines.append("")

        return


    lines.append(
        "The same turning scenario was simulated "
        "using progressively smaller timesteps."
    )

    lines.append("")

    lines.append(
        "| dt (s) | Euler X | Euler Y | "
        "RK4 X | RK4 Y | "
        "Position Difference (m) | "
        "Heading Difference (rad) |"
    )

    lines.append(
        "|---:|---:|---:|---:|---:|---:|---:|"
    )


    for _, row in df.iterrows():

        lines.append(
            f"| {row['dt']:.3f} "
            f"| {row['euler_x']:.5f} "
            f"| {row['euler_y']:.5f} "
            f"| {row['rk4_x']:.5f} "
            f"| {row['rk4_y']:.5f} "
            f"| {row['position_error']:.6f} "
            f"| {row['heading_error']:.6f} |"
        )


    lines.append("")


    largest_dt_row = (
        df.loc[
            df["dt"].idxmax()
        ]
    )


    smallest_dt_row = (
        df.loc[
            df["dt"].idxmin()
        ]
    )


    large_error = (
        largest_dt_row[
            "position_error"
        ]
    )


    small_error = (
        smallest_dt_row[
            "position_error"
        ]
    )


    lines.append(
        f"At dt = {largest_dt_row['dt']:.3f} s, "
        f"the Euler/RK4 position difference was "
        f"{large_error:.6f} m."
    )

    lines.append("")


    lines.append(
        f"At dt = {smallest_dt_row['dt']:.3f} s, "
        f"the difference decreased to "
        f"{small_error:.6f} m."
    )

    lines.append("")


    if (
        small_error
        <
        large_error
    ):

        lines.append(
            "**Convergence result:** decreasing the "
            "simulation timestep reduced the difference "
            "between Euler and RK4 trajectories."
        )

    else:

        lines.append(
            "**Convergence result:** the expected "
            "decreasing-error trend was not observed."
        )


    lines.append("")


    convergence_plot = (
        RESULTS_DIR
        / "timestep_convergence.png"
    )


    if convergence_plot.exists():

        lines.append(
            "### Timestep Convergence Plot"
        )

        lines.append("")

        lines.append(
            "![Timestep Convergence]"
            "(timestep_convergence.png)"
        )

        lines.append("")


def add_trajectory_section(
    lines
):
    lines.append(
        "## Scenario Trajectories"
    )

    lines.append("")


    plots = sorted(
        RESULTS_DIR.glob(
            "*_trajectory.png"
        )
    )


    if not plots:

        lines.append(
            "No trajectory plots were found."
        )

        lines.append("")

        return


    for plot in plots:

        scenario_name = (
            plot.stem
            .replace(
                "_trajectory",
                ""
            )
            .replace(
                "_",
                " "
            )
            .title()
        )


        lines.append(
            f"### {scenario_name}"
        )

        lines.append("")

        lines.append(
            f"![{scenario_name}]"
            f"({plot.name})"
        )

        lines.append("")


def add_project_summary(
    lines
):
    lines.append(
        "## Project Capabilities"
    )

    lines.append("")

    lines.append(
        "The completed framework demonstrates:"
    )

    lines.append("")

    lines.append(
        "- Scenario-driven C++ vehicle simulation."
    )

    lines.append(
        "- Kinematic bicycle vehicle dynamics."
    )

    lines.append(
        "- Euler and RK4 numerical integration."
    )

    lines.append(
        "- Configurable integration method through JSON."
    )

    lines.append(
        "- Automated Python reprocessing and validation."
    )

    lines.append(
        "- Parameter-sweep generation and execution."
    )

    lines.append(
        "- Euler-vs-RK4 comparative experiments."
    )

    lines.append(
        "- Timestep-convergence analysis."
    )

    lines.append(
        "- Regression testing against known-good baselines."
    )

    lines.append(
        "- CMake and CTest-based build/test automation."
    )

    lines.append(
        "- One-command PowerShell execution pipeline."
    )

    lines.append("")


def build_report():
    RESULTS_DIR.mkdir(
        exist_ok=True
    )


    lines = []


    add_overview_section(
        lines
    )


    add_architecture_section(
        lines
    )


    add_validation_section(
        lines
    )


    add_turn_sweep_section(
        lines
    )


    add_integration_sweep_section(
        lines
    )


    add_timestep_section(
        lines
    )


    add_trajectory_section(
        lines
    )


    add_project_summary(
        lines
    )


    REPORT_FILE.write_text(
        "\n".join(
            lines
        ),
        encoding="utf-8"
    )


    print(
        "Report generated:",
        REPORT_FILE
    )


if __name__ == "__main__":
    build_report()