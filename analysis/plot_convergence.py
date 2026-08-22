from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


INPUT_FILE = Path("results/timestep_sensitivity.csv")
OUTPUT_FILE = Path("results/timestep_convergence.png")


def main():

    if not INPUT_FILE.exists():
        print(f"ERROR: Missing input file: {INPUT_FILE}")
        return 1

    data = pd.read_csv(INPUT_FILE)

    required_columns = {
        "dt",
        "steps",
        "simulated_time",
        "position_difference",
        "heading_difference",
    }

    missing_columns = required_columns - set(data.columns)

    if missing_columns:
        print(
            "ERROR: Missing required columns in "
            f"{INPUT_FILE}: {sorted(missing_columns)}"
        )
        return 1

    numeric_columns = [
        "dt",
        "steps",
        "simulated_time",
        "position_difference",
        "heading_difference",
    ]

    for column in numeric_columns:
        data[column] = pd.to_numeric(
            data[column],
            errors="coerce"
        )

    if data[numeric_columns].isnull().any().any():
        print(
            "ERROR: timestep_sensitivity.csv contains "
            "invalid numeric values."
        )
        return 1

    data = data.sort_values("dt").reset_index(drop=True)

    print()
    print("=" * 80)
    print("TIMESTEP CONVERGENCE ANALYSIS")
    print("=" * 80)

    print(
        data[
            [
                "dt",
                "steps",
                "simulated_time",
                "position_difference",
                "heading_difference",
            ]
        ].to_string(index=False)
    )

    expected_time = float(
        data["simulated_time"].iloc[0]
    )

    maximum_time_difference = (
        data["simulated_time"] - expected_time
    ).abs().max()

    time_consistent = (
        maximum_time_difference < 1e-9
    )

    print()
    print(
        f"Expected simulated time: "
        f"{expected_time:.6f} s"
    )

    print(
        "Consistent simulated time:",
        time_consistent
    )

    if not time_consistent:
        print(
            "WARNING: Not every timestep simulation "
            "ended at the same physical time."
        )


    dt_values = data["dt"]

    position_difference = data[
        "position_difference"
    ]


    OUTPUT_FILE.parent.mkdir(
        parents=True,
        exist_ok=True
    )

    plt.figure(figsize=(8, 5))

    plt.plot(
        dt_values,
        position_difference,
        marker="o"
    )

    plt.xlabel("Timestep dt [s]")

    plt.ylabel(
        "Euler-RK4 final position difference [m]"
    )

    plt.title(
        "Numerical Integration Convergence"
    )

    plt.grid(True)

    plt.tight_layout()

    plt.savefig(
        OUTPUT_FILE,
        dpi=150
    )

    plt.close()

    print()
    print(
        "Convergence plot saved to:",
        OUTPUT_FILE
    )

    finest_row = data.iloc[0]
    coarsest_row = data.iloc[-1]

    finest_dt = float(
        finest_row["dt"]
    )

    coarsest_dt = float(
        coarsest_row["dt"]
    )

    finest_error = float(
        finest_row["position_difference"]
    )

    coarsest_error = float(
        coarsest_row["position_difference"]
    )

    print()
    print("=" * 80)
    print("CONVERGENCE SUMMARY")
    print("=" * 80)

    print(
        f"Coarsest timestep: "
        f"{coarsest_dt:.6f} s"
    )

    print(
        f"Coarsest Euler-RK4 difference: "
        f"{coarsest_error:.6f} m"
    )

    print()

    print(
        f"Finest timestep: "
        f"{finest_dt:.6f} s"
    )

    print(
        f"Finest Euler-RK4 difference: "
        f"{finest_error:.6f} m"
    )

    if finest_error < coarsest_error:
        print()
        print(
            "CONVERGENCE CHECK: PASS"
        )

        print(
            "Euler approaches the RK4 solution "
            "as the timestep becomes smaller."
        )
    else:
        print()
        print(
            "CONVERGENCE CHECK: WARNING"
        )

        print(
            "The Euler-RK4 difference did not decrease "
            "between the coarsest and finest timestep."
        )

    print()
    print("=" * 80)
    print("ERROR REDUCTION")
    print("=" * 80)

    descending = data.sort_values(
        "dt",
        ascending=False
    ).reset_index(drop=True)

    previous_error = None

    for _, row in descending.iterrows():
        current_dt = float(
            row["dt"]
        )

        current_error = float(
            row["position_difference"]
        )

        if previous_error is None:
            print(
                f"dt={current_dt:.6f} s | "
                f"error={current_error:.6f} m"
            )
        else:
            if previous_error > 0.0:
                ratio = (
                    current_error
                    /
                    previous_error
                )

                print(
                    f"dt={current_dt:.6f} s | "
                    f"error={current_error:.6f} m | "
                    f"relative error={ratio:.6f}"
                )
            else:
                print(
                    f"dt={current_dt:.6f} s | "
                    f"error={current_error:.6f} m"
                )

        previous_error = current_error

    print()
    print(
        "Timestep convergence analysis completed."
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())