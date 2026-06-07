import serial
import re
import math
from collections import deque
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.gridspec import GridSpec
from mpl_toolkits.mplot3d import Axes3D
import threading


class KalmanVisualizer:
    def __init__(self, port="/dev/ttyACM0", baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.max_history = 200

        self.history = {
            "pos": deque(maxlen=self.max_history),
            "vel": deque(maxlen=self.max_history),
            "euler": deque(maxlen=self.max_history),
            "ori": deque(maxlen=self.max_history),
            "ang_vel": deque(maxlen=self.max_history),
            "acc_bias": deque(maxlen=self.max_history),
            "gyro_bias": deque(maxlen=self.max_history),
            "time_per_kalman": deque(maxlen=self.max_history),
        }

        self.setup_plots()

    def connect(self):
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"Connected to {self.port}")
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False

    def quaternion_to_euler(self, quat):
        """Convert quaternion [qx, qy, qz, qw] to Euler angles (degrees)"""
        qx, qy, qz, qw = quat

        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (qw * qy - qz * qx)
        pitch = math.asin(min(1, max(-1, sinp)))

        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]

    def quaternion_to_rotation_matrix(self, quat):
        """Convert quaternion to 3x3 rotation matrix"""
        qx, qy, qz, qw = quat

        R = np.array(
            [
                [
                    1 - 2 * (qy**2 + qz**2),
                    2 * (qx * qy - qz * qw),
                    2 * (qx * qz + qy * qw),
                ],
                [
                    2 * (qx * qy + qz * qw),
                    1 - 2 * (qx**2 + qz**2),
                    2 * (qy * qz - qx * qw),
                ],
                [
                    2 * (qx * qz - qy * qw),
                    2 * (qy * qz + qx * qw),
                    1 - 2 * (qx**2 + qy**2),
                ],
            ]
        )

        return R

    def setup_plots(self):
        """Setup matplotlib dashboard"""
        self.fig = plt.figure(figsize=(18, 10))
        self.fig.suptitle("Kalman Filter State Monitor", fontsize=16, fontweight="bold")

        gs = GridSpec(3, 4, figure=self.fig, hspace=0.3, wspace=0.3)

        self.ax_3d = self.fig.add_subplot(gs[0:2, 0], projection="3d")
        self.ax_pos = self.fig.add_subplot(gs[0, 1:3])
        self.ax_vel = self.fig.add_subplot(gs[1, 1:3])
        self.ax_euler = self.fig.add_subplot(gs[2, 1:3])

        self.ax_ang_vel = self.fig.add_subplot(gs[0, 3])
        self.ax_acc_bias = self.fig.add_subplot(gs[1, 3])
        self.ax_gyro_bias = self.fig.add_subplot(gs[2, 3])

        self.ax_stats = self.fig.add_axes([0.02, 0.02, 0.15, 0.15])
        self.ax_stats.axis("off")
        self.stats_text = self.ax_stats.text(
            0,
            1,
            "Waiting for data...",
            transform=self.ax_stats.transAxes,
            fontsize=9,
            verticalalignment="top",
            fontfamily="monospace",
        )

        self.ax_3d.set_xlabel("X")
        self.ax_3d.set_ylabel("Y")
        self.ax_3d.set_zlabel("Z")
        self.ax_3d.set_title("3D Orientation")
        self.ax_3d.set_xlim([-1.5, 1.5])
        self.ax_3d.set_ylim([-1.5, 1.5])
        self.ax_3d.set_zlim([-1.5, 1.5])

    def draw_3d_frame(self, ax, rotation_matrix):
        """Draw coordinate frame on 3D axis"""
        ax.clear()

        origin = np.array([0, 0, 0])

        x_axis = np.array([1, 0, 0])
        y_axis = np.array([0, 1, 0])
        z_axis = np.array([0, 0, 1])

        x_rotated = rotation_matrix @ x_axis
        y_rotated = rotation_matrix @ y_axis
        z_rotated = rotation_matrix @ z_axis

        ax.quiver(
            0,
            0,
            0,
            x_rotated[0],
            x_rotated[1],
            x_rotated[2],
            color="r",
            arrow_length_ratio=0.2,
            linewidth=2.5,
            label="X",
        )
        ax.quiver(
            0,
            0,
            0,
            y_rotated[0],
            y_rotated[1],
            y_rotated[2],
            color="g",
            arrow_length_ratio=0.2,
            linewidth=2.5,
            label="Y",
        )
        ax.quiver(
            0,
            0,
            0,
            z_rotated[0],
            z_rotated[1],
            z_rotated[2],
            color="b",
            arrow_length_ratio=0.2,
            linewidth=2.5,
            label="Z",
        )

        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_xlim([-1.5, 1.5])
        ax.set_ylim([-1.5, 1.5])
        ax.set_zlim([-1.5, 1.5])
        ax.set_title("3D Orientation")
        ax.legend(fontsize=10)

    def parse_state(self, lines):
        """Parse STATE block from list of lines"""
        state = {}

        for line in lines:
            line = line.strip()
            if not line or line.startswith("STATE"):
                continue

            if "=" in line:
                key, value = line.split("=", 1)
                key = key.strip()
                value = value.strip()

                try:
                    if "[" in value and "]" in value:
                        nums = re.findall(r"-?\d+\.?\d*", value)
                        state[key] = [float(n) for n in nums]
                    else:
                        state[key] = float(value)
                except ValueError:
                    pass

        return state

    def update_data(self, state, time_per_kalman):
        """Add new state to history"""
        if "pos" in state:
            self.history["pos"].append(state["pos"])
        if "vel" in state:
            self.history["vel"].append(state["vel"])
        if "ori" in state:
            self.history["ori"].append(state["ori"])
            euler = self.quaternion_to_euler(state["ori"])
            self.history["euler"].append(euler)
        if "ang_vel" in state:
            self.history["ang_vel"].append(state["ang_vel"])
        if "acc_bias" in state:
            self.history["acc_bias"].append(state["acc_bias"])
        if "gyro_bias" in state:
            self.history["gyro_bias"].append(state["gyro_bias"])

        self.history["time_per_kalman"].append(time_per_kalman * 1e6)

    def update_plots(self, frame):
        """Update all plots"""
        pos_data = list(self.history["pos"])
        vel_data = list(self.history["vel"])
        euler_data = list(self.history["euler"])
        ang_vel_data = list(self.history["ang_vel"])
        acc_bias_data = list(self.history["acc_bias"])
        gyro_bias_data = list(self.history["gyro_bias"])
        ori_data = list(self.history["ori"])

        if ori_data:
            latest_quat = ori_data[-1]
            R = self.quaternion_to_rotation_matrix(latest_quat)
            self.draw_3d_frame(self.ax_3d, R)

        self.ax_pos.clear()
        self.ax_vel.clear()
        self.ax_euler.clear()
        self.ax_ang_vel.clear()
        self.ax_acc_bias.clear()
        self.ax_gyro_bias.clear()

        if pos_data and len(pos_data) > 1:
            pos_array = list(zip(*pos_data))
            self.ax_pos.plot(pos_array[0], "r-", alpha=0.7, linewidth=1.5)
            self.ax_pos.plot(pos_array[1], "g-", alpha=0.7, linewidth=1.5)
            self.ax_pos.plot(pos_array[2], "b-", alpha=0.7, linewidth=1.5)
            self.ax_pos.set_ylabel("Position (m)")
            self.ax_pos.set_title("Position [X(red) Y(green) Z(blue)]")
            self.ax_pos.grid(True, alpha=0.3)

        if vel_data and len(vel_data) > 1:
            vel_array = list(zip(*vel_data))
            self.ax_vel.plot(vel_array[0], "r-", alpha=0.7, linewidth=1.5)
            self.ax_vel.plot(vel_array[1], "g-", alpha=0.7, linewidth=1.5)
            self.ax_vel.plot(vel_array[2], "b-", alpha=0.7, linewidth=1.5)
            self.ax_vel.set_ylabel("Velocity (m/s)")
            self.ax_vel.set_title("Velocity [X(red) Y(green) Z(blue)]")
            self.ax_vel.grid(True, alpha=0.3)

        if euler_data and len(euler_data) > 1:
            euler_array = list(zip(*euler_data))
            self.ax_euler.plot(euler_array[0], "r-", alpha=0.7, linewidth=1.5)
            self.ax_euler.plot(euler_array[1], "g-", alpha=0.7, linewidth=1.5)
            self.ax_euler.plot(euler_array[2], "b-", alpha=0.7, linewidth=1.5)
            self.ax_euler.set_ylabel("Angle (degrees)")
            self.ax_euler.set_xlabel("Samples")
            self.ax_euler.set_title("Orientation [Roll(red) Pitch(green) Yaw(blue)]")
            self.ax_euler.grid(True, alpha=0.3)

        if ang_vel_data and len(ang_vel_data) > 1:
            ang_vel_array = list(zip(*ang_vel_data))
            self.ax_ang_vel.plot(ang_vel_array[0], "r-", alpha=0.7, linewidth=1)
            self.ax_ang_vel.plot(ang_vel_array[1], "g-", alpha=0.7, linewidth=1)
            self.ax_ang_vel.plot(ang_vel_array[2], "b-", alpha=0.7, linewidth=1)
            self.ax_ang_vel.set_ylabel("Ang Vel (rad/s)")
            self.ax_ang_vel.set_title("Angular Velocity")
            self.ax_ang_vel.grid(True, alpha=0.3)

        if acc_bias_data and len(acc_bias_data) > 1:
            acc_bias_array = list(zip(*acc_bias_data))
            self.ax_acc_bias.plot(acc_bias_array[0], "r-", alpha=0.7, linewidth=1)
            self.ax_acc_bias.plot(acc_bias_array[1], "g-", alpha=0.7, linewidth=1)
            self.ax_acc_bias.plot(acc_bias_array[2], "b-", alpha=0.7, linewidth=1)
            self.ax_acc_bias.set_ylabel("Accel Bias (m/s²)")
            self.ax_acc_bias.set_title("Accel Bias")
            self.ax_acc_bias.grid(True, alpha=0.3)

        if gyro_bias_data and len(gyro_bias_data) > 1:
            gyro_bias_array = list(zip(*gyro_bias_data))
            self.ax_gyro_bias.plot(gyro_bias_array[0], "r-", alpha=0.7, linewidth=1)
            self.ax_gyro_bias.plot(gyro_bias_array[1], "g-", alpha=0.7, linewidth=1)
            self.ax_gyro_bias.plot(gyro_bias_array[2], "b-", alpha=0.7, linewidth=1)
            self.ax_gyro_bias.set_ylabel("Gyro Bias (rad/s)")
            self.ax_gyro_bias.set_xlabel("Samples")
            self.ax_gyro_bias.set_title("Gyro Bias")
            self.ax_gyro_bias.grid(True, alpha=0.3)

        for ax in [
            self.ax_pos,
            self.ax_vel,
            self.ax_euler,
            self.ax_ang_vel,
            self.ax_acc_bias,
            self.ax_gyro_bias,
        ]:
            ax.set_xlim(0, self.max_history)

        if len(self.history["time_per_kalman"]) > 0:
            avg_time = sum(self.history["time_per_kalman"]) / len(
                self.history["time_per_kalman"]
            )
            max_time = max(self.history["time_per_kalman"])
            min_time = min(self.history["time_per_kalman"])

            if euler_data:
                latest_euler = euler_data[-1]
                stats_str = f"Samples: {len(euler_data)}\n"
                stats_str += f"Avg: {avg_time:.3f} µs\n"
                stats_str += f"Max: {max_time:.3f} µs\n"
                stats_str += f"Min: {min_time:.3f} µs\n"
                stats_str += f"\nRoll:  {latest_euler[0]:7.2f}°\n"
                stats_str += f"Pitch: {latest_euler[1]:7.2f}°\n"
                stats_str += f"Yaw:   {latest_euler[2]:7.2f}°"

                self.stats_text.set_text(stats_str)

    def run(self):
        """Start visualization"""
        if not self.connect():
            return

        buffer = ""
        state_lines = []
        loop_count = None
        time_per_kalman = None
        in_state_block = False

        def animate(frame):
            nonlocal buffer, state_lines, loop_count, time_per_kalman, in_state_block

            try:
                if self.serial.in_waiting:
                    data = self.serial.read(self.serial.in_waiting)
                    buffer += data.decode("utf-8", errors="ignore")

                    lines = buffer.split("\n")
                    buffer = lines[-1]

                    for line in lines[:-1]:
                        line_stripped = line.strip()

                        if not line_stripped:
                            continue

                        if line_stripped.startswith("loops:"):
                            parts = re.findall(r"loops:\s*(\d+)", line_stripped)
                            if parts:
                                loop_count = int(parts[0])
                            parts = re.findall(
                                r"time_per_kalman:\s*([\d.]+)", line_stripped
                            )
                            if parts:
                                time_per_kalman = float(parts[0])

                            if (
                                in_state_block
                                and state_lines
                                and loop_count
                                and time_per_kalman
                            ):
                                state = self.parse_state(state_lines)
                                if state and "pos" in state:
                                    self.update_data(state, time_per_kalman)

                            state_lines = []
                            in_state_block = False

                        elif line_stripped.startswith("STATE:"):
                            in_state_block = True
                            state_lines = []

                        elif in_state_block and "=" in line_stripped:
                            state_lines.append(line_stripped)

            except Exception as e:
                print(f"Serial error: {e}")

            self.update_plots(frame)

        ani = animation.FuncAnimation(
            self.fig, animate, interval=100, blit=False, cache_frame_data=False
        )

        try:
            plt.show()
        finally:
            if self.serial:
                self.serial.close()


if __name__ == "__main__":
    viz = KalmanVisualizer()
    viz.run()
