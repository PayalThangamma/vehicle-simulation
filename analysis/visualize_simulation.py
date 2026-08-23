import argparse
import math
from pathlib import Path

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.patches import Polygon


REQUIRED_COLUMNS = {
    "time",
    "x",
    "y",
    "velocity",
    "heading",
    "commanded_acceleration",
    "steering_angle",
    "target_velocity",
    "speed_error",
}


def load_simulation(csv_path: Path) -> pd.DataFrame:
    data = pd.read_csv(csv_path)

    missing = REQUIRED_COLUMNS.difference(data.columns)

    if missing:
        missing_text = ", ".join(sorted(missing))
        raise ValueError(
            f"CSV is missing required columns: {missing_text}"
        )

    if data.empty:
        raise ValueError("Simulation CSV contains no rows.")

    return data


def create_vehicle_polygon(
    x: float,
    y: float,
    heading: float,
    length: float = 4.5,
    width: float = 2.0,
):
    half_length = length / 2.0
    half_width = width / 2.0

    local_points = [
        (half_length, 0.0),
        (-half_length, half_width),
        (-half_length, -half_width),
    ]

    cos_heading = math.cos(heading)
    sin_heading = math.sin(heading)

    world_points = []

    for local_x, local_y in local_points:
        world_x = (
            x
            + local_x * cos_heading
            - local_y * sin_heading
        )

        world_y = (
            y
            + local_x * sin_heading
            + local_y * cos_heading
        )

        world_points.append(
            (world_x, world_y)
        )

    return world_points


def calculate_axis_limits(values, margin_ratio=0.15, minimum_margin=5.0):
    minimum = float(values.min())
    maximum = float(values.max())

    span = maximum - minimum

    margin = max(
        span * margin_ratio,
        minimum_margin,
    )

    return (
        minimum - margin,
        maximum + margin,
    )


def visualize(
    data: pd.DataFrame,
    csv_path: Path,
    save_gif: bool,
):
    times = data["time"]
    x_values = data["x"]
    y_values = data["y"]
    velocities = data["velocity"]
    headings = data["heading"]

    accelerations = data["commanded_acceleration"]
    steering_angles = data["steering_angle"]
    target_velocities = data["target_velocity"]
    speed_errors = data["speed_error"]

    x_limits = calculate_axis_limits(x_values)
    y_limits = calculate_axis_limits(y_values)

    figure = plt.figure(
        figsize=(14, 8)
    )

    grid = figure.add_gridspec(
        2,
        2,
        width_ratios=[2.0, 1.0],
        height_ratios=[2.0, 1.0],
    )

    trajectory_axis = figure.add_subplot(
        grid[:, 0]
    )

    telemetry_axis = figure.add_subplot(
        grid[0, 1]
    )

    speed_axis = figure.add_subplot(
        grid[1, 1]
    )

    figure.suptitle(
        f"Vehicle Simulation Replay\n{csv_path.name}",
        fontsize=16,
    )

    # ============================================================
    # Trajectory view
    # ============================================================

    trajectory_axis.set_title(
        "Vehicle Trajectory"
    )

    trajectory_axis.set_xlabel(
        "X position [m]"
    )

    trajectory_axis.set_ylabel(
        "Y position [m]"
    )

    trajectory_axis.set_xlim(
        x_limits
    )

    trajectory_axis.set_ylim(
        y_limits
    )

    trajectory_axis.grid(True)

    trajectory_axis.set_aspect(
        "equal",
        adjustable="box",
    )

    trajectory_line, = trajectory_axis.plot(
        [],
        [],
        linewidth=2,
        label="Driven trajectory",
    )

    current_position, = trajectory_axis.plot(
        [],
        [],
        marker="o",
        markersize=5,
        linestyle="None",
    )

    vehicle_polygon = Polygon(
        create_vehicle_polygon(
            float(x_values.iloc[0]),
            float(y_values.iloc[0]),
            float(headings.iloc[0]),
        ),
        closed=True,
    )

    trajectory_axis.add_patch(
        vehicle_polygon
    )

    trajectory_axis.legend(
        loc="best"
    )

    # ============================================================
    # Telemetry view
    # ============================================================

    telemetry_axis.axis(
        "off"
    )

    telemetry_axis.set_title(
        "Live Telemetry"
    )

    telemetry_text = telemetry_axis.text(
        0.02,
        0.95,
        "",
        transform=telemetry_axis.transAxes,
        verticalalignment="top",
        family="monospace",
        fontsize=11,
    )

    # ============================================================
    # Speed tracking plot
    # ============================================================

    speed_axis.set_title(
        "Velocity Tracking"
    )

    speed_axis.set_xlabel(
        "Time [s]"
    )

    speed_axis.set_ylabel(
        "Velocity [m/s]"
    )

    speed_axis.set_xlim(
        float(times.min()),
        float(times.max()),
    )

    minimum_speed = min(
        float(velocities.min()),
        float(target_velocities.min()),
    )

    maximum_speed = max(
        float(velocities.max()),
        float(target_velocities.max()),
    )

    speed_margin = max(
        (maximum_speed - minimum_speed) * 0.15,
        1.0,
    )

    speed_axis.set_ylim(
        minimum_speed - speed_margin,
        maximum_speed + speed_margin,
    )

    speed_axis.grid(True)

    actual_speed_line, = speed_axis.plot(
        [],
        [],
        linewidth=2,
        label="Actual velocity",
    )

    target_speed_line, = speed_axis.plot(
        [],
        [],
        linestyle="--",
        linewidth=2,
        label="Target velocity",
    )

    current_speed_marker, = speed_axis.plot(
        [],
        [],
        marker="o",
        linestyle="None",
    )

    speed_axis.legend(
        loc="best"
    )

    # ============================================================
    # Animation update
    # ============================================================

    def update(frame_index):
        current_time = float(
            times.iloc[frame_index]
        )

        current_x = float(
            x_values.iloc[frame_index]
        )

        current_y = float(
            y_values.iloc[frame_index]
        )

        current_velocity = float(
            velocities.iloc[frame_index]
        )

        current_heading = float(
            headings.iloc[frame_index]
        )

        current_acceleration = float(
            accelerations.iloc[frame_index]
        )

        current_steering = float(
            steering_angles.iloc[frame_index]
        )

        current_target = float(
            target_velocities.iloc[frame_index]
        )

        current_error = float(
            speed_errors.iloc[frame_index]
        )

        trajectory_line.set_data(
            x_values.iloc[: frame_index + 1],
            y_values.iloc[: frame_index + 1],
        )

        current_position.set_data(
            [current_x],
            [current_y],
        )

        vehicle_polygon.set_xy(
            create_vehicle_polygon(
                current_x,
                current_y,
                current_heading,
            )
        )

        actual_speed_line.set_data(
            times.iloc[: frame_index + 1],
            velocities.iloc[: frame_index + 1],
        )

        target_speed_line.set_data(
            times.iloc[: frame_index + 1],
            target_velocities.iloc[: frame_index + 1],
        )

        current_speed_marker.set_data(
            [current_time],
            [current_velocity],
        )

        telemetry_text.set_text(
            "\n".join(
                [
                    f"Time           : {current_time:8.2f} s",
                    f"X position     : {current_x:8.2f} m",
                    f"Y position     : {current_y:8.2f} m",
                    f"Velocity       : {current_velocity:8.2f} m/s",
                    f"Target velocity: {current_target:8.2f} m/s",
                    f"Speed error    : {current_error:8.2f} m/s",
                    f"Acceleration   : {current_acceleration:8.2f} m/s^2",
                    f"Steering angle : {current_steering:8.4f} rad",
                    f"Heading        : {current_heading:8.4f} rad",
                ]
            )
        )

        return (
            trajectory_line,
            current_position,
            vehicle_polygon,
            actual_speed_line,
            target_speed_line,
            current_speed_marker,
            telemetry_text,
        )

    # ============================================================
    # Determine animation timing
    # ============================================================

    if len(times) >= 2:
        dt = float(
            times.iloc[1] - times.iloc[0]
        )

        interval_ms = max(
            int(dt * 1000),
            20,
        )
    else:
        interval_ms = 100

    replay = animation.FuncAnimation(
        figure,
        update,
        frames=len(data),
        interval=interval_ms,
        blit=False,
        repeat=True,
    )

    figure.tight_layout()

    # ============================================================
    # Optional GIF export
    # ============================================================

    if save_gif:
        output_directory = Path(
            "docs"
        )

        output_directory.mkdir(
            parents=True,
            exist_ok=True,
        )

        output_path = (
            output_directory
            / f"{csv_path.stem}_demo.gif"
        )

        print(
            f"Saving GIF to: {output_path}"
        )

        replay.save(
            output_path,
            writer="pillow",
            fps=20,
        )

        print(
            "GIF export complete."
        )

    plt.show()


def parse_arguments():
    parser = argparse.ArgumentParser(
        description=(
            "Replay vehicle-simulation CSV output "
            "as an animated visualization."
        )
    )

    parser.add_argument(
        "csv",
        type=Path,
        help="Path to a simulation CSV file.",
    )

    parser.add_argument(
        "--save-gif",
        action="store_true",
        help=(
            "Export the animation to docs/<scenario>_demo.gif."
        ),
    )

    return parser.parse_args()


def main():
    arguments = parse_arguments()

    csv_path = arguments.csv

    if not csv_path.exists():
        raise FileNotFoundError(
            f"Simulation CSV not found: {csv_path}"
        )

    print(
        "============================================"
    )

    print(
        "Vehicle Simulation Visualizer"
    )

    print(
        "============================================"
    )

    print(
        f"Input: {csv_path}"
    )

    data = load_simulation(
        csv_path
    )

    print(
        f"Samples: {len(data)}"
    )

    print(
        f"Duration: {data['time'].iloc[-1]:.3f} s"
    )

    visualize(
        data=data,
        csv_path=csv_path,
        save_gif=arguments.save_gif,
    )


if __name__ == "__main__":
    main()