from pathlib import Path
import sys

import pandas as pd


INPUT_FILE = Path("results/adaptive_cruise_control.csv")


REQUIRED_COLUMNS = {
    "time",
    "velocity",
    "commanded_acceleration",
    "lead_vehicle_position",
    "lead_vehicle_velocity",
    "lead_vehicle_acceleration",
    "actual_gap",
    "desired_gap",
    "gap_error",
    "relative_velocity",
}


def main():
    print("============================================")
    print("Adaptive Cruise Control Validation")
    print("============================================")

    if not INPUT_FILE.exists():
        print(f"ERROR: Missing input file: {INPUT_FILE}")
        return 1

    data = pd.read_csv(INPUT_FILE)

    missing_columns = REQUIRED_COLUMNS.difference(data.columns)

    if missing_columns:
        print(
            "ERROR: Missing required columns: "
            + ", ".join(sorted(missing_columns))
        )
        return 1

    if data.empty:
        print("ERROR: ACC result file contains no data.")
        return 1


    minimum_gap = float(
        data["actual_gap"].min()
    )

    final_gap = float(
        data["actual_gap"].iloc[-1]
    )

    final_desired_gap = float(
        data["desired_gap"].iloc[-1]
    )

    final_gap_error = float(
        data["gap_error"].iloc[-1]
    )

    minimum_acceleration = float(
        data["commanded_acceleration"].min()
    )

    maximum_acceleration = float(
        data["commanded_acceleration"].max()
    )

    minimum_ego_velocity = float(
        data["velocity"].min()
    )

    final_ego_velocity = float(
        data["velocity"].iloc[-1]
    )

    final_lead_velocity = float(
        data["lead_vehicle_velocity"].iloc[-1]
    )

    final_relative_velocity = float(
        data["relative_velocity"].iloc[-1]
    )

    lead_braking_rows = data[
        data["lead_vehicle_acceleration"] < 0.0
    ]

    lead_braked = (
        not lead_braking_rows.empty
    )

    ego_braked = (
        minimum_acceleration < -0.1
    )

    collision_avoided = (
        minimum_gap > 0.0
    )

    minimum_gap_safe = (
        minimum_gap >= 5.0
    )

    ego_velocity_valid = (
        minimum_ego_velocity >= 0.0
    )

    acceleration_limits_valid = (
        minimum_acceleration >= -5.0 - 1e-9
        and
        maximum_acceleration <= 3.0 + 1e-9
    )

    final_relative_speed_small = (
        abs(final_relative_velocity) <= 0.5
    )

    final_gap_tracking_reasonable = (
        abs(final_gap_error) <= 5.0
    )

    print()
    print("Measured ACC performance")
    print("--------------------------------------------")

    print(
        f"Minimum gap:             {minimum_gap:.4f} m"
    )

    print(
        f"Final gap:               {final_gap:.4f} m"
    )

    print(
        f"Final desired gap:       {final_desired_gap:.4f} m"
    )

    print(
        f"Final gap error:         {final_gap_error:.4f} m"
    )

    print(
        f"Minimum acceleration:   {minimum_acceleration:.4f} m/s^2"
    )

    print(
        f"Maximum acceleration:    {maximum_acceleration:.4f} m/s^2"
    )

    print(
        f"Final ego velocity:      {final_ego_velocity:.4f} m/s"
    )

    print(
        f"Final lead velocity:     {final_lead_velocity:.4f} m/s"
    )

    print(
        f"Final relative velocity: {final_relative_velocity:.4f} m/s"
    )

    print()

    print("Validation criteria")
    print("--------------------------------------------")

    print(
        f"Lead vehicle braking detected:      {lead_braked}"
    )

    print(
        f"Ego braking response detected:      {ego_braked}"
    )

    print(
        f"Collision avoided:                  {collision_avoided}"
    )

    print(
        f"Minimum gap >= 5 m:                 {minimum_gap_safe}"
    )

    print(
        f"Ego velocity never negative:        {ego_velocity_valid}"
    )

    print(
        f"Acceleration limits respected:      {acceleration_limits_valid}"
    )

    print(
        f"Final relative speed <= 0.5 m/s:    {final_relative_speed_small}"
    )

    print(
        f"Final gap error <= 5 m:             {final_gap_tracking_reasonable}"
    )

    all_passed = all(
        [
            lead_braked,
            ego_braked,
            collision_avoided,
            minimum_gap_safe,
            ego_velocity_valid,
            acceleration_limits_valid,
            final_relative_speed_small,
            final_gap_tracking_reasonable,
        ]
    )

    print()
    print("============================================")

    if all_passed:
        print("ADAPTIVE CRUISE CONTROL RESULT: PASS")
        print("============================================")
        return 0

    print("ADAPTIVE CRUISE CONTROL RESULT: FAIL")
    print("============================================")

    return 1


if __name__ == "__main__":
    sys.exit(main())