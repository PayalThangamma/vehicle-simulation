from pathlib import Path
import json
import sys

import pandas as pd


SUMMARY_FILE = Path(
    "results/cruise_control_tuning_summary.csv"
)

SCENARIO_FILE = Path(
    "scenarios/cruise_control.json"
)


MAX_OVERSHOOT_PERCENT = 5.0
MAX_STEADY_STATE_ERROR = 0.5
MAX_FINAL_ERROR = 0.5


def main():

    if not SUMMARY_FILE.exists():
        print(
            "Missing tuning summary:",
            SUMMARY_FILE
        )
        return 1


    if not SCENARIO_FILE.exists():
        print(
            "Missing cruise-control scenario:",
            SCENARIO_FILE
        )
        return 1


    summary = pd.read_csv(
        SUMMARY_FILE
    )


    required_columns = {
        "kp",
        "ki",
        "settling_time_s",
        "overshoot_percent",
        "steady_state_error_mps",
        "final_error_mps",
        "score",
    }


    missing_columns = (
        required_columns
        -
        set(summary.columns)
    )


    if missing_columns:
        print(
            "Tuning summary is missing columns:",
            sorted(missing_columns)
        )
        return 1

    valid = summary[
        (
            summary["overshoot_percent"]
            <=
            MAX_OVERSHOOT_PERCENT
        )
        &
        (
            summary["steady_state_error_mps"]
            <=
            MAX_STEADY_STATE_ERROR
        )
        &
        (
            summary["final_error_mps"]
            <=
            MAX_FINAL_ERROR
        )
        &
        (
            summary["settling_time_s"]
            .notna()
        )
    ].copy()


    print()
    print(
        "=" * 80
    )

    print(
        "CRUISE CONTROLLER GAIN SELECTION"
    )

    print(
        "=" * 80
    )


    print(
        f"Total configurations evaluated: "
        f"{len(summary)}"
    )


    print(
        f"Configurations satisfying requirements: "
        f"{len(valid)}"
    )


    if valid.empty:

        print()
        print(
            "No controller configuration "
            "satisfies all validation requirements."
        )

        print()
        print(
            "Do not update cruise_control.json."
        )

        return 1


    valid = (
        valid
        .sort_values(
            "score"
        )
        .reset_index(
            drop=True
        )
    )


    print()
    print(
        "VALID CONFIGURATIONS"
    )

    print(
        "-" * 80
    )


    display_columns = [
        "kp",
        "ki",
        "rise_time_s",
        "settling_time_s",
        "overshoot_percent",
        "steady_state_error_mps",
        "final_error_mps",
        "score",
    ]


    available_columns = [
        column
        for column in display_columns
        if column in valid.columns
    ]


    print(
        valid[
            available_columns
        ]
        .to_string(
            index=False
        )
    )


    best = (
        valid.iloc[0]
    )


    best_kp = float(
        best["kp"]
    )


    best_ki = float(
        best["ki"]
    )


    print()
    print(
        "SELECTED CONFIGURATION"
    )

    print(
        "----------------------"
    )


    print(
        f"Kp: {best_kp}"
    )


    print(
        f"Ki: {best_ki}"
    )


    if "rise_time_s" in best:

        print(
            f"Rise time: "
            f"{best['rise_time_s']:.3f} s"
        )


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

    with SCENARIO_FILE.open(
        "r",
        encoding="utf-8"
    ) as file:

        config = json.load(
            file
        )


    old_kp = config.get(
        "cruiseKp"
    )


    old_ki = config.get(
        "cruiseKi"
    )


    config[
        "cruiseControlEnabled"
    ] = True


    config[
        "cruiseKp"
    ] = best_kp


    config[
        "cruiseKi"
    ] = best_ki


    with SCENARIO_FILE.open(
        "w",
        encoding="utf-8"
    ) as file:

        json.dump(
            config,
            file,
            indent=2
        )

        file.write(
            "\n"
        )


    print()
    print(
        "Scenario updated successfully."
    )


    print(
        f"Previous gains: "
        f"Kp={old_kp}, Ki={old_ki}"
    )


    print(
        f"New gains: "
        f"Kp={best_kp}, Ki={best_ki}"
    )


    print(
        "Updated file:",
        SCENARIO_FILE
    )


    return 0


if __name__ == "__main__":
    sys.exit(
        main()
    )