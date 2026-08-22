from pathlib import Path
import sys

import numpy as np
import pandas as pd


BASELINE_DIR = Path("baselines")
RESULTS_DIR = Path("results")


X_TOLERANCE = 0.05
Y_TOLERANCE = 0.05
VELOCITY_TOLERANCE = 0.05
HEADING_TOLERANCE = 0.01
TIME_TOLERANCE = 1e-9


def rmse(reference, current):
    difference = (
        reference
        -
        current
    )

    return np.sqrt(
        np.mean(
            difference ** 2
        )
    )


def compare_scenario(
    baseline_file
):
    scenario_name = (
        baseline_file.stem
        .replace(
            "_baseline",
            ""
        )
    )


    result_file = (
        RESULTS_DIR
        /
        f"{scenario_name}.csv"
    )


    print()
    print(
        "=" * 60
    )

    print(
        f"Scenario: {scenario_name}"
    )

    print(
        "=" * 60
    )


    if not result_file.exists():

        print(
            "FAIL: Current simulation result "
            f"does not exist: {result_file}"
        )

        return False, "missing"


    baseline = pd.read_csv(
        baseline_file
    )

    current = pd.read_csv(
        result_file
    )


    required_columns = [
        "time",
        "x",
        "y",
        "velocity",
        "heading",
    ]


    for column in required_columns:

        if column not in baseline.columns:

            print(
                "FAIL: Baseline is missing "
                f"column '{column}'."
            )

            return False, "failed"


        if column not in current.columns:

            print(
                "FAIL: Current result is missing "
                f"column '{column}'."
            )

            return False, "failed"


    # ==================================================
    # Sample count
    # ==================================================

    if len(baseline) != len(current):

        print(
            "FAIL: Number of simulation "
            "samples changed."
        )

        print(
            f"Baseline rows: {len(baseline)}"
        )

        print(
            f"Current rows: {len(current)}"
        )

        return False, "failed"


    # ==================================================
    # Time grid
    # ==================================================

    time_error = rmse(
        baseline["time"].to_numpy(),
        current["time"].to_numpy()
    )


    time_ok = (
        time_error
        <=
        TIME_TOLERANCE
    )


    if not time_ok:

        print(
            "FAIL: Simulation time grid changed."
        )

        print(
            f"Time RMSE: {time_error}"
        )

        return False, "failed"


    # ==================================================
    # State errors
    # ==================================================

    x_error = rmse(
        baseline["x"].to_numpy(),
        current["x"].to_numpy()
    )


    y_error = rmse(
        baseline["y"].to_numpy(),
        current["y"].to_numpy()
    )


    velocity_error = rmse(
        baseline["velocity"].to_numpy(),
        current["velocity"].to_numpy()
    )


    heading_error = rmse(
        baseline["heading"].to_numpy(),
        current["heading"].to_numpy()
    )


    print(
        f"X RMSE: {x_error}"
    )

    print(
        f"Y RMSE: {y_error}"
    )

    print(
        f"Velocity RMSE: {velocity_error}"
    )

    print(
        f"Heading RMSE: {heading_error}"
    )


    x_ok = (
        x_error
        <=
        X_TOLERANCE
    )


    y_ok = (
        y_error
        <=
        Y_TOLERANCE
    )


    velocity_ok = (
        velocity_error
        <=
        VELOCITY_TOLERANCE
    )


    heading_ok = (
        heading_error
        <=
        HEADING_TOLERANCE
    )


    print()
    print(
        "Checks"
    )

    print(
        "------"
    )


    print(
        "X trajectory:",
        x_ok
    )

    print(
        "Y trajectory:",
        y_ok
    )

    print(
        "Velocity:",
        velocity_ok
    )

    print(
        "Heading:",
        heading_ok
    )


    passed = (
        x_ok
        and
        y_ok
        and
        velocity_ok
        and
        heading_ok
    )


    if passed:

        print(
            "RESULT: PASS"
        )

        return True, "passed"


    print(
        "RESULT: FAIL"
    )

    return False, "failed"


def main():

    if not BASELINE_DIR.exists():

        print(
            "Baseline directory does not exist:"
        )

        print(
            BASELINE_DIR
        )

        return 1


    baseline_files = sorted(
        BASELINE_DIR.glob(
            "*_baseline.csv"
        )
    )


    if not baseline_files:

        print(
            "No regression baselines found."
        )

        return 1


    passed = 0
    failed = 0
    missing = 0


    for baseline_file in baseline_files:

        result, status = compare_scenario(
            baseline_file
        )


        if status == "passed":

            passed += 1

        elif status == "missing":

            missing += 1

        else:

            failed += 1


    print()
    print(
        "=" * 60
    )

    print(
        "REGRESSION SUMMARY"
    )

    print(
        "=" * 60
    )


    print(
        f"Total baselines: "
        f"{len(baseline_files)}"
    )

    print(
        f"Passed: {passed}"
    )

    print(
        f"Failed: {failed}"
    )

    print(
        f"Missing: {missing}"
    )


    if (
        failed > 0
        or
        missing > 0
    ):

        print()
        print(
            "OVERALL REGRESSION RESULT: FAIL"
        )

        return 1


    print()
    print(
        "OVERALL REGRESSION RESULT: PASS"
    )

    return 0


if __name__ == "__main__":
    sys.exit(
        main()
    )