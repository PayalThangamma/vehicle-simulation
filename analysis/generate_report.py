from pathlib import Path

import pandas as pd


RESULTS_DIR = Path("results")
REPORT_PATH = RESULTS_DIR / "simulation_report.md"


def safe_read_csv(path: Path):
    if not path.exists():
        return None

    try:
        return pd.read_csv(path)
    except Exception:
        return None


def yes_no(value: bool) -> str:
    return "PASS" if value else "FAIL"


def main():
    RESULTS_DIR.mkdir(
        parents=True,
        exist_ok=True,
    )

    report_lines = []

    report_lines.append(
        "# Simulation Factory Validation Report"
    )

    report_lines.append("")
    report_lines.append(
        "Automated summary of simulation, controller, numerical, "
        "and regression-validation results."
    )

    report_lines.append("")
    report_lines.append("---")
    report_lines.append("")

    cruise_path = (
        RESULTS_DIR
        / "cruise_control.csv"
    )

    cruise = safe_read_csv(
        cruise_path
    )

    cruise_pass = False

    cruise_metrics = {}

    if (
        cruise is not None
        and
        not cruise.empty
        and
        {
            "time",
            "velocity",
            "target_velocity",
            "speed_error",
        }.issubset(cruise.columns)
    ):
        target_velocity = float(
            cruise["target_velocity"].iloc[-1]
        )

        final_velocity = float(
            cruise["velocity"].iloc[-1]
        )

        maximum_velocity = float(
            cruise["velocity"].max()
        )

        final_error = abs(
            target_velocity
            -
            final_velocity
        )

        overshoot = 0.0

        if target_velocity > 0.0:
            overshoot = max(
                0.0,
                (
                    maximum_velocity
                    -
                    target_velocity
                )
                /
                target_velocity
                *
                100.0,
            )

        steady_state_window = cruise.tail(
            min(
                20,
                len(cruise),
            )
        )

        steady_state_error = float(
            steady_state_window[
                "speed_error"
            ].abs().mean()
        )

        cruise_pass = (
            final_error <= 0.5
            and
            overshoot <= 5.0
            and
            steady_state_error <= 0.5
        )

        cruise_metrics = {
            "target_velocity":
                target_velocity,

            "final_velocity":
                final_velocity,

            "maximum_velocity":
                maximum_velocity,

            "final_error":
                final_error,

            "overshoot":
                overshoot,

            "steady_state_error":
                steady_state_error,
        }

    acc_path = (
        RESULTS_DIR
        / "adaptive_cruise_control.csv"
    )

    acc = safe_read_csv(
        acc_path
    )

    acc_pass = False

    acc_metrics = {}

    if (
        acc is not None
        and
        not acc.empty
        and
        {
            "velocity",
            "commanded_acceleration",
            "lead_vehicle_velocity",
            "lead_vehicle_acceleration",
            "actual_gap",
            "desired_gap",
            "gap_error",
            "relative_velocity",
        }.issubset(acc.columns)
    ):
        minimum_gap = float(
            acc["actual_gap"].min()
        )

        final_gap = float(
            acc["actual_gap"].iloc[-1]
        )

        final_desired_gap = float(
            acc["desired_gap"].iloc[-1]
        )

        final_gap_error = float(
            acc["gap_error"].iloc[-1]
        )

        final_relative_velocity = float(
            acc["relative_velocity"].iloc[-1]
        )

        minimum_acceleration = float(
            acc[
                "commanded_acceleration"
            ].min()
        )

        maximum_acceleration = float(
            acc[
                "commanded_acceleration"
            ].max()
        )

        lead_braked = bool(
            (
                acc[
                    "lead_vehicle_acceleration"
                ]
                <
                0.0
            ).any()
        )

        ego_braked = bool(
            minimum_acceleration
            <
            -0.1
        )

        collision_avoided = (
            minimum_gap > 0.0
        )

        safe_gap = (
            minimum_gap >= 5.0
        )

        final_relative_speed_ok = (
            abs(
                final_relative_velocity
            )
            <=
            0.5
        )

        final_gap_error_ok = (
            abs(
                final_gap_error
            )
            <=
            5.0
        )

        acc_pass = all(
            [
                lead_braked,
                ego_braked,
                collision_avoided,
                safe_gap,
                final_relative_speed_ok,
                final_gap_error_ok,
            ]
        )

        acc_metrics = {
            "minimum_gap":
                minimum_gap,

            "final_gap":
                final_gap,

            "final_desired_gap":
                final_desired_gap,

            "final_gap_error":
                final_gap_error,

            "final_relative_velocity":
                final_relative_velocity,

            "minimum_acceleration":
                minimum_acceleration,

            "maximum_acceleration":
                maximum_acceleration,
        }

    sweep_path = (
        RESULTS_DIR
        / "turn_sweep_summary.csv"
    )

    sweep = safe_read_csv(
        sweep_path
    )

    sweep_pass = False

    sweep_total = 0
    sweep_passed = 0
    sweep_failed = 0

    if (
        sweep is not None
        and
        not sweep.empty
    ):
        sweep_total = len(
            sweep
        )

        if "result" in sweep.columns:
            normalized_result = (
                sweep["result"]
                .astype(str)
                .str.upper()
            )

            sweep_passed = int(
                (
                    normalized_result
                    ==
                    "PASS"
                ).sum()
            )

            sweep_failed = (
                sweep_total
                -
                sweep_passed
            )

            sweep_pass = (
                sweep_total > 0
                and
                sweep_failed == 0
            )

        elif "passed" in sweep.columns:
            passed_values = (
                sweep["passed"]
                .astype(str)
                .str.lower()
                .isin(
                    [
                        "true",
                        "1",
                        "yes",
                        "pass",
                    ]
                )
            )

            sweep_passed = int(
                passed_values.sum()
            )

            sweep_failed = (
                sweep_total
                -
                sweep_passed
            )

            sweep_pass = (
                sweep_total > 0
                and
                sweep_failed == 0
            )

    integration_path = (
        RESULTS_DIR
        / "integration_method_comparison.csv"
    )

    integration = safe_read_csv(
        integration_path
    )

    integration_metrics = {}

    if (
        integration is not None
        and
        not integration.empty
    ):
        if (
            "position_difference"
            in
            integration.columns
        ):
            integration_metrics[
                "mean_position_difference"
            ] = float(
                integration[
                    "position_difference"
                ].mean()
            )

            integration_metrics[
                "max_position_difference"
            ] = float(
                integration[
                    "position_difference"
                ].max()
            )

        possible_euler_columns = [
            "euler_radius_error",
            "Euler_radius_error",
            "euler_error",
        ]

        possible_rk4_columns = [
            "rk4_radius_error",
            "RK4_radius_error",
            "rk4_error",
        ]

        for column in possible_euler_columns:
            if column in integration.columns:
                integration_metrics[
                    "mean_euler_radius_error"
                ] = float(
                    integration[
                        column
                    ].mean()
                )

                break

        for column in possible_rk4_columns:
            if column in integration.columns:
                integration_metrics[
                    "mean_rk4_radius_error"
                ] = float(
                    integration[
                        column
                    ].mean()
                )

                break

    timestep_path = (
        RESULTS_DIR
        / "timestep_sensitivity.csv"
    )

    timestep = safe_read_csv(
        timestep_path
    )

    convergence_pass = False

    convergence_metrics = {}

    if (
        timestep is not None
        and
        not timestep.empty
        and
        {
            "dt",
            "position_difference",
        }.issubset(timestep.columns)
    ):
        timestep_sorted = (
            timestep
            .sort_values(
                "dt",
                ascending=False,
            )
            .reset_index(
                drop=True
            )
        )

        errors = (
            timestep_sorted[
                "position_difference"
            ]
            .astype(float)
            .tolist()
        )

        convergence_pass = all(
            errors[index + 1]
            <
            errors[index]
            for index in range(
                len(errors) - 1
            )
        )

        convergence_metrics = {
            "coarsest_dt":
                float(
                    timestep_sorted[
                        "dt"
                    ].iloc[0]
                ),

            "coarsest_error":
                float(
                    timestep_sorted[
                        "position_difference"
                    ].iloc[0]
                ),

            "finest_dt":
                float(
                    timestep_sorted[
                        "dt"
                    ].iloc[-1]
                ),

            "finest_error":
                float(
                    timestep_sorted[
                        "position_difference"
                    ].iloc[-1]
                ),
        }


    regression_candidates = [
        RESULTS_DIR
        / "regression_summary.csv",

        RESULTS_DIR
        / "regression_results.csv",
    ]

    regression = None

    for candidate in regression_candidates:
        regression = safe_read_csv(
            candidate
        )

        if (
            regression is not None
            and
            not regression.empty
        ):
            break

    regression_pass = False
    regression_total = 0
    regression_passed = 0

    if (
        regression is not None
        and
        not regression.empty
    ):
        regression_total = len(
            regression
        )

        if "result" in regression.columns:
            regression_passed = int(
                (
                    regression[
                        "result"
                    ]
                    .astype(str)
                    .str.upper()
                    ==
                    "PASS"
                ).sum()
            )

        elif "passed" in regression.columns:
            regression_passed = int(
                regression[
                    "passed"
                ]
                .astype(str)
                .str.lower()
                .isin(
                    [
                        "true",
                        "1",
                        "yes",
                        "pass",
                    ]
                )
                .sum()
            )

        regression_pass = (
            regression_total > 0
            and
            regression_passed
            ==
            regression_total
        )

    if regression_total == 0:
        baseline_files = list(
            Path(
                "baselines"
            ).glob(
                "*_baseline.csv"
            )
        )

        regression_total = len(
            baseline_files
        )

        regression_passed = 0

    report_lines.append(
        "## Validation Summary"
    )

    report_lines.append("")

    report_lines.append(
        "| Validation | Result |"
    )

    report_lines.append(
        "|---|---:|"
    )

    report_lines.append(
        f"| Cruise Control | {yes_no(cruise_pass)} |"
    )

    report_lines.append(
        f"| Adaptive Cruise Control | {yes_no(acc_pass)} |"
    )

    report_lines.append(
        f"| Turn Sweep | "
        f"{sweep_passed} / {sweep_total} "
        f"{yes_no(sweep_pass)} |"
    )

    if regression_total > 0:
        regression_text = (
            f"{regression_passed} / "
            f"{regression_total} "
            f"{yes_no(regression_pass)}"
        )
    else:
        regression_text = "NOT AVAILABLE"

    report_lines.append(
        f"| Regression | {regression_text} |"
    )

    report_lines.append(
        f"| Numerical Convergence | "
        f"{yes_no(convergence_pass)} |"
    )

    report_lines.append("")

    report_lines.append(
        "## Cruise Control"
    )

    report_lines.append("")

    if cruise_metrics:
        report_lines.append(
            "| Metric | Value |"
        )

        report_lines.append(
            "|---|---:|"
        )

        report_lines.append(
            "| Target velocity | "
            f"{cruise_metrics['target_velocity']:.3f} m/s |"
        )

        report_lines.append(
            "| Final velocity | "
            f"{cruise_metrics['final_velocity']:.3f} m/s |"
        )

        report_lines.append(
            "| Maximum velocity | "
            f"{cruise_metrics['maximum_velocity']:.3f} m/s |"
        )

        report_lines.append(
            "| Final speed error | "
            f"{cruise_metrics['final_error']:.3f} m/s |"
        )

        report_lines.append(
            "| Overshoot | "
            f"{cruise_metrics['overshoot']:.3f}% |"
        )

        report_lines.append(
            "| Steady-state error | "
            f"{cruise_metrics['steady_state_error']:.3f} m/s |"
        )

    else:
        report_lines.append(
            "Cruise-control results were not available."
        )

    report_lines.append("")

    report_lines.append(
        "## Adaptive Cruise Control"
    )

    report_lines.append("")

    if acc_metrics:
        report_lines.append(
            "| Metric | Value |"
        )

        report_lines.append(
            "|---|---:|"
        )

        report_lines.append(
            "| Minimum following gap | "
            f"{acc_metrics['minimum_gap']:.4f} m |"
        )

        report_lines.append(
            "| Final gap | "
            f"{acc_metrics['final_gap']:.4f} m |"
        )

        report_lines.append(
            "| Final desired gap | "
            f"{acc_metrics['final_desired_gap']:.4f} m |"
        )

        report_lines.append(
            "| Final gap error | "
            f"{acc_metrics['final_gap_error']:.4f} m |"
        )

        report_lines.append(
            "| Final relative velocity | "
            f"{acc_metrics['final_relative_velocity']:.4f} m/s |"
        )

        report_lines.append(
            "| Minimum ego acceleration | "
            f"{acc_metrics['minimum_acceleration']:.4f} m/s^2 |"
        )

        report_lines.append(
            "| Maximum ego acceleration | "
            f"{acc_metrics['maximum_acceleration']:.4f} m/s^2 |"
        )

        report_lines.append("")
        report_lines.append(
            f"Collision avoided: **"
            f"{'YES' if acc_metrics['minimum_gap'] > 0.0 else 'NO'}"
            f"**"
        )

    else:
        report_lines.append(
            "Adaptive Cruise Control results were not available."
        )

    report_lines.append("")

    report_lines.append(
        "## Numerical Integration"
    )

    report_lines.append("")

    if integration_metrics:
        report_lines.append(
            "| Metric | Value |"
        )

        report_lines.append(
            "|---|---:|"
        )

        if (
            "mean_position_difference"
            in
            integration_metrics
        ):
            report_lines.append(
                "| Mean Euler/RK4 final-position difference | "
                f"{integration_metrics['mean_position_difference']:.6f} m |"
            )

        if (
            "max_position_difference"
            in
            integration_metrics
        ):
            report_lines.append(
                "| Maximum Euler/RK4 final-position difference | "
                f"{integration_metrics['max_position_difference']:.6f} m |"
            )

        if (
            "mean_euler_radius_error"
            in
            integration_metrics
        ):
            report_lines.append(
                "| Mean Euler turning-radius error | "
                f"{integration_metrics['mean_euler_radius_error']:.6f} m |"
            )

        if (
            "mean_rk4_radius_error"
            in
            integration_metrics
        ):
            report_lines.append(
                "| Mean RK4 turning-radius error | "
                f"{integration_metrics['mean_rk4_radius_error']:.6f} m |"
            )

    else:
        report_lines.append(
            "Integration comparison results were not available."
        )

    report_lines.append("")

    report_lines.append(
        "## Timestep Convergence"
    )

    report_lines.append("")

    if convergence_metrics:
        report_lines.append(
            "| Metric | Value |"
        )

        report_lines.append(
            "|---|---:|"
        )

        report_lines.append(
            "| Coarsest timestep | "
            f"{convergence_metrics['coarsest_dt']:.3f} s |"
        )

        report_lines.append(
            "| Coarsest error | "
            f"{convergence_metrics['coarsest_error']:.6f} m |"
        )

        report_lines.append(
            "| Finest timestep | "
            f"{convergence_metrics['finest_dt']:.3f} s |"
        )

        report_lines.append(
            "| Finest error | "
            f"{convergence_metrics['finest_error']:.6f} m |"
        )

        report_lines.append("")
        report_lines.append(
            "Convergence result: "
            f"**{yes_no(convergence_pass)}**"
        )

    else:
        report_lines.append(
            "Timestep-sensitivity results were not available."
        )

    report_lines.append("")

    core_results = [
        cruise_pass,
        acc_pass,
        sweep_pass,
        convergence_pass,
    ]

    overall_pass = all(
        core_results
    )

    report_lines.append(
        "---"
    )

    report_lines.append("")

    report_lines.append(
        "## Overall Result"
    )

    report_lines.append("")

    report_lines.append(
        f"**SIMULATION FACTORY RESULT: "
        f"{yes_no(overall_pass)}**"
    )

    report_lines.append("")

    REPORT_PATH.write_text(
        "\n".join(
            report_lines
        ),
        encoding="utf-8",
    )

    print(
        "============================================"
    )

    print(
        "Simulation Factory Report"
    )

    print(
        "============================================"
    )

    print(
        f"Cruise Control:            {yes_no(cruise_pass)}"
    )

    print(
        f"Adaptive Cruise Control:   {yes_no(acc_pass)}"
    )

    print(
        f"Turn Sweep:                "
        f"{sweep_passed}/{sweep_total} "
        f"{yes_no(sweep_pass)}"
    )

    print(
        f"Numerical Convergence:     "
        f"{yes_no(convergence_pass)}"
    )

    if regression_total > 0:
        print(
            f"Regression:                "
            f"{regression_passed}/{regression_total} "
            f"{yes_no(regression_pass)}"
        )

    print(
        "============================================"
    )

    print(
        f"Overall Result:             "
        f"{yes_no(overall_pass)}"
    )

    print(
        f"Report saved to: {REPORT_PATH}"
    )

    print(
        "============================================"
    )


if __name__ == "__main__":
    main()