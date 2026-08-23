import argparse
import math
from pathlib import Path

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.patches import Polygon


BASE_REQUIRED_COLUMNS = {
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


ACC_COLUMNS = {
    "lead_vehicle_position",
    "lead_vehicle_velocity",
    "lead_vehicle_acceleration",
    "actual_gap",
    "desired_gap",
    "gap_error",
    "relative_velocity",
}


def load_simulation(csv_path: Path) -> pd.DataFrame:
    data = pd.read_csv(csv_path)

    missing = BASE_REQUIRED_COLUMNS.difference(data.columns)

    if missing:
        raise ValueError(
            "CSV is missing required columns: "
            + ", ".join(sorted(missing))
        )

    if data.empty:
        raise ValueError("Simulation CSV contains no rows.")

    return data


def is_acc_scenario(data: pd.DataFrame) -> bool:
    if not ACC_COLUMNS.issubset(data.columns):
        return False

    max_gap = float(
        data["actual_gap"].abs().max()
    )

    max_lead_velocity = float(
        data["lead_vehicle_velocity"].abs().max()
    )

    return (
        max_gap > 1e-9
        or
        max_lead_velocity > 1e-9
    )


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


def calculate_axis_limits(
    values,
    margin_ratio=0.15,
    minimum_margin=5.0,
):
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
    acc_enabled = is_acc_scenario(data)

    times = data["time"]
    x_values = data["x"]
    y_values = data["y"]
    velocities = data["velocity"]
    headings = data["heading"]

    accelerations = data[
        "commanded_acceleration"
    ]

    steering_angles = data[
        "steering_angle"
    ]

    target_velocities = data[
        "target_velocity"
    ]

    speed_errors = data[
        "speed_error"
    ]

    if acc_enabled:
        lead_positions = data[
            "lead_vehicle_position"
        ]

        lead_velocities = data[
            "lead_vehicle_velocity"
        ]

        lead_accelerations = data[
            "lead_vehicle_acceleration"
        ]

        actual_gaps = data[
            "actual_gap"
        ]

        desired_gaps = data[
            "desired_gap"
        ]

        gap_errors = data[
            "gap_error"
        ]

        relative_velocities = data[
            "relative_velocity"
        ]

        combined_x_values = pd.concat(
            [
                x_values,
                lead_positions,
            ],
            ignore_index=True,
        )

        x_limits = calculate_axis_limits(
            combined_x_values
        )
    else:
        x_limits = calculate_axis_limits(
            x_values
        )

    y_limits = calculate_axis_limits(
        y_values,
        minimum_margin=4.0,
    )

    figure = plt.figure(
        figsize=(15, 9)
    )

    grid = figure.add_gridspec(
        3,
        2,
        width_ratios=[2.0, 1.2],
        height_ratios=[1.5, 1.0, 1.0],
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

    acceleration_axis = figure.add_subplot(
        grid[2, 1]
    )

    figure.suptitle(
        f"Vehicle Simulation Replay\n{csv_path.name}",
        fontsize=16,
    )

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

    trajectory_axis.grid(
        True
    )

    trajectory_axis.set_aspect(
        "equal",
        adjustable="box",
    )

    ego_trajectory_line, = (
        trajectory_axis.plot(
            [],
            [],
            linewidth=2,
            label="Ego trajectory",
        )
    )

    ego_position_marker, = (
        trajectory_axis.plot(
            [],
            [],
            marker="o",
            markersize=5,
            linestyle="None",
        )
    )

    ego_polygon = Polygon(
        create_vehicle_polygon(
            float(x_values.iloc[0]),
            float(y_values.iloc[0]),
            float(headings.iloc[0]),
        ),
        closed=True,
    )

    trajectory_axis.add_patch(
        ego_polygon
    )

    lead_polygon = None
    lead_position_marker = None
    gap_line = None

    if acc_enabled:
        lead_polygon = Polygon(
            create_vehicle_polygon(
                float(
                    lead_positions.iloc[0]
                ),
                0.0,
                0.0,
            ),
            closed=True,
        )

        trajectory_axis.add_patch(
            lead_polygon
        )

        lead_position_marker, = (
            trajectory_axis.plot(
                [],
                [],
                marker="s",
                markersize=5,
                linestyle="None",
                label="Lead vehicle",
            )
        )

        gap_line, = trajectory_axis.plot(
            [],
            [],
            linestyle="--",
            linewidth=1.5,
            label="Following gap",
        )

    trajectory_axis.legend(
        loc="best"
    )

    telemetry_axis.axis(
        "off"
    )

    telemetry_axis.set_title(
        "Live Telemetry"
    )

    telemetry_text = telemetry_axis.text(
        0.02,
        0.96,
        "",
        transform=telemetry_axis.transAxes,
        verticalalignment="top",
        family="monospace",
        fontsize=10.5,
    )

    speed_axis.set_title(
        "Velocity"
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

    speed_series = [
        velocities,
        target_velocities,
    ]

    if acc_enabled:
        speed_series.append(
            lead_velocities
        )

    minimum_speed = min(
        float(series.min())
        for series in speed_series
    )

    maximum_speed = max(
        float(series.max())
        for series in speed_series
    )

    speed_margin = max(
        (
            maximum_speed
            -
            minimum_speed
        )
        *
        0.15,
        1.0,
    )

    speed_axis.set_ylim(
        minimum_speed - speed_margin,
        maximum_speed + speed_margin,
    )

    speed_axis.grid(
        True
    )

    ego_speed_line, = speed_axis.plot(
        [],
        [],
        linewidth=2,
        label="Ego velocity",
    )

    target_speed_line, = speed_axis.plot(
        [],
        [],
        linestyle="--",
        linewidth=1.5,
        label="Target velocity",
    )

    lead_speed_line = None

    if acc_enabled:
        lead_speed_line, = speed_axis.plot(
            [],
            [],
            linewidth=2,
            label="Lead velocity",
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

    acceleration_axis.set_title(
        "Acceleration Command"
    )

    acceleration_axis.set_xlabel(
        "Time [s]"
    )

    acceleration_axis.set_ylabel(
        "Acceleration [m/s²]"
    )

    acceleration_axis.set_xlim(
        float(times.min()),
        float(times.max()),
    )

    minimum_acceleration = float(
        accelerations.min()
    )

    maximum_acceleration = float(
        accelerations.max()
    )

    if acc_enabled:
        minimum_acceleration = min(
            minimum_acceleration,
            float(
                lead_accelerations.min()
            ),
        )

        maximum_acceleration = max(
            maximum_acceleration,
            float(
                lead_accelerations.max()
            ),
        )

    acceleration_margin = max(
        (
            maximum_acceleration
            -
            minimum_acceleration
        )
        *
        0.20,
        0.5,
    )

    acceleration_axis.set_ylim(
        minimum_acceleration
        -
        acceleration_margin,
        maximum_acceleration
        +
        acceleration_margin,
    )

    acceleration_axis.grid(
        True
    )

    ego_acceleration_line, = (
        acceleration_axis.plot(
            [],
            [],
            linewidth=2,
            label="Ego command",
        )
    )

    lead_acceleration_line = None

    if acc_enabled:
        lead_acceleration_line, = (
            acceleration_axis.plot(
                [],
                [],
                linestyle="--",
                linewidth=1.5,
                label="Lead acceleration",
            )
        )

    acceleration_axis.legend(
        loc="best"
    )

    def update(
        frame_index
    ):
        current_time = float(
            times.iloc[
                frame_index
            ]
        )

        current_x = float(
            x_values.iloc[
                frame_index
            ]
        )

        current_y = float(
            y_values.iloc[
                frame_index
            ]
        )

        current_velocity = float(
            velocities.iloc[
                frame_index
            ]
        )

        current_heading = float(
            headings.iloc[
                frame_index
            ]
        )

        current_acceleration = float(
            accelerations.iloc[
                frame_index
            ]
        )

        current_steering = float(
            steering_angles.iloc[
                frame_index
            ]
        )

        current_target = float(
            target_velocities.iloc[
                frame_index
            ]
        )

        current_speed_error = float(
            speed_errors.iloc[
                frame_index
            ]
        )

        ego_trajectory_line.set_data(
            x_values.iloc[
                : frame_index + 1
            ],
            y_values.iloc[
                : frame_index + 1
            ],
        )

        ego_position_marker.set_data(
            [current_x],
            [current_y],
        )

        ego_polygon.set_xy(
            create_vehicle_polygon(
                current_x,
                current_y,
                current_heading,
            )
        )

        ego_speed_line.set_data(
            times.iloc[
                : frame_index + 1
            ],
            velocities.iloc[
                : frame_index + 1
            ],
        )

        target_speed_line.set_data(
            times.iloc[
                : frame_index + 1
            ],
            target_velocities.iloc[
                : frame_index + 1
            ],
        )

        current_speed_marker.set_data(
            [current_time],
            [current_velocity],
        )

        ego_acceleration_line.set_data(
            times.iloc[
                : frame_index + 1
            ],
            accelerations.iloc[
                : frame_index + 1
            ],
        )

        if acc_enabled:
            current_lead_position = float(
                lead_positions.iloc[
                    frame_index
                ]
            )

            current_lead_velocity = float(
                lead_velocities.iloc[
                    frame_index
                ]
            )

            current_lead_acceleration = float(
                lead_accelerations.iloc[
                    frame_index
                ]
            )

            current_actual_gap = float(
                actual_gaps.iloc[
                    frame_index
                ]
            )

            current_desired_gap = float(
                desired_gaps.iloc[
                    frame_index
                ]
            )

            current_gap_error = float(
                gap_errors.iloc[
                    frame_index
                ]
            )

            current_relative_velocity = float(
                relative_velocities.iloc[
                    frame_index
                ]
            )

            lead_polygon.set_xy(
                create_vehicle_polygon(
                    current_lead_position,
                    0.0,
                    0.0,
                )
            )

            lead_position_marker.set_data(
                [current_lead_position],
                [0.0],
            )

            gap_line.set_data(
                [
                    current_x,
                    current_lead_position,
                ],
                [
                    current_y,
                    0.0,
                ],
            )

            lead_speed_line.set_data(
                times.iloc[
                    : frame_index + 1
                ],
                lead_velocities.iloc[
                    : frame_index + 1
                ],
            )

            lead_acceleration_line.set_data(
                times.iloc[
                    : frame_index + 1
                ],
                lead_accelerations.iloc[
                    : frame_index + 1
                ],
            )

            telemetry_lines = [
                "MODE: ADAPTIVE CRUISE CONTROL",
                "",
                f"Time              : {current_time:8.2f} s",
                "",
                f"Ego velocity      : {current_velocity:8.2f} m/s",
                f"Lead velocity     : {current_lead_velocity:8.2f} m/s",
                f"Relative velocity : {current_relative_velocity:8.2f} m/s",
                "",
                f"Actual gap        : {current_actual_gap:8.2f} m",
                f"Desired gap       : {current_desired_gap:8.2f} m",
                f"Gap error         : {current_gap_error:8.2f} m",
                "",
                f"Ego acceleration  : {current_acceleration:8.2f} m/s²",
                f"Lead acceleration : {current_lead_acceleration:8.2f} m/s²",
            ]

            if (
                current_lead_acceleration
                <
                -1e-6
            ):
                telemetry_lines.append(
                    ""
                )

                telemetry_lines.append(
                    "LEAD VEHICLE BRAKING"
                )

            telemetry_text.set_text(
                "\n".join(
                    telemetry_lines
                )
            )

        else:
            telemetry_text.set_text(
                "\n".join(
                    [
                        "MODE: VEHICLE SIMULATION",
                        "",
                        f"Time           : {current_time:8.2f} s",
                        f"X position     : {current_x:8.2f} m",
                        f"Y position     : {current_y:8.2f} m",
                        f"Velocity       : {current_velocity:8.2f} m/s",
                        f"Target velocity: {current_target:8.2f} m/s",
                        f"Speed error    : {current_speed_error:8.2f} m/s",
                        f"Acceleration   : {current_acceleration:8.2f} m/s²",
                        f"Steering angle : {current_steering:8.4f} rad",
                        f"Heading        : {current_heading:8.4f} rad",
                    ]
                )
            )

        artists = [
            ego_trajectory_line,
            ego_position_marker,
            ego_polygon,
            ego_speed_line,
            target_speed_line,
            current_speed_marker,
            ego_acceleration_line,
            telemetry_text,
        ]

        if acc_enabled:
            artists.extend(
                [
                    lead_polygon,
                    lead_position_marker,
                    gap_line,
                    lead_speed_line,
                    lead_acceleration_line,
                ]
            )

        return tuple(
            artists
        )

    if len(times) >= 2:
        dt = float(
            times.iloc[1]
            -
            times.iloc[0]
        )

        interval_ms = max(
            int(
                dt
                *
                1000
            ),
            20,
        )
    else:
        interval_ms = 100

    replay = animation.FuncAnimation(
        figure,
        update,
        frames=len(
            data
        ),
        interval=interval_ms,
        blit=False,
        repeat=True,
    )

    figure.tight_layout()

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
            /
            f"{csv_path.stem}_demo.gif"
        )

        if len(times) >= 2:
            simulation_dt = float(
                times.iloc[1]
                -
                times.iloc[0]
            )

            gif_fps = max(
                1,
                min(
                    30,
                    round(
                        1.0
                        /
                        simulation_dt
                    ),
                ),
            )
        else:
            gif_fps = 10

        print(
            f"Saving GIF to: {output_path}"
        )

        print(
            f"GIF frame rate: {gif_fps} FPS"
        )

        replay.save(
            output_path,
            writer="pillow",
            fps=gif_fps,
        )

        print(
            "GIF export complete."
        )

    plt.show()


def parse_arguments():
    parser = argparse.ArgumentParser(
        description=(
            "Replay vehicle simulation CSV output "
            "as an animated visualization."
        )
    )

    parser.add_argument(
        "csv",
        type=Path,
        help=(
            "Path to a simulation CSV file."
        ),
    )

    parser.add_argument(
        "--save-gif",
        action="store_true",
        help=(
            "Export the animation into docs/."
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

    acc_enabled = is_acc_scenario(
        data
    )

    print(
        f"Samples: {len(data)}"
    )

    print(
        f"Duration: {data['time'].iloc[-1]:.3f} s"
    )

    print(
        "ACC data detected: "
        +
        (
            "Yes"
            if acc_enabled
            else "No"
        )
    )

    visualize(
        data=data,
        csv_path=csv_path,
        save_gif=arguments.save_gif,
    )


if __name__ == "__main__":
    main()