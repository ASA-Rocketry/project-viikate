import argparse
import math
import re
from collections import deque

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import serial


class AttitudeMonitor:
    def __init__(self, port, baudrate):
        self.serial = serial.Serial(port, baudrate, timeout=0)
        self.buffer = ""
        self.roll = deque(maxlen=200)
        self.pitch = deque(maxlen=200)

        self.figure = plt.figure(figsize=(10, 5))
        self.figure.suptitle("Canard attitude")
        self.attitude_axis = self.figure.add_subplot(1, 2, 1)
        self.model_axis = self.figure.add_subplot(1, 2, 2, projection="3d")
        self.roll_line, = self.attitude_axis.plot([], [], label="Roll")
        self.pitch_line, = self.attitude_axis.plot([], [], label="Pitch")
        self.attitude_axis.set(
            xlabel="Samples",
            ylabel="Angle (degrees)",
            ylim=(-90, 90),
        )
        self.attitude_axis.grid(True)
        self.attitude_axis.legend()

    def read_serial(self):
        if not self.serial.in_waiting:
            return

        self.buffer += self.serial.read(self.serial.in_waiting).decode(
            "utf-8", errors="ignore"
        )
        lines = self.buffer.splitlines(keepends=True)
        self.buffer = ""
        if lines and not lines[-1].endswith(("\n", "\r")):
            self.buffer = lines.pop()

        for line in lines:
            match = re.search(
                r"ATTITUDE roll=(-?[\d.]+) pitch=(-?[\d.]+)",
                line,
            )
            if match:
                self.roll.append(float(match.group(1)))
                self.pitch.append(float(match.group(2)))

    def draw_model(self):
        self.model_axis.clear()
        self.model_axis.set(
            xlim=(-1, 1),
            ylim=(-1, 1),
            zlim=(-1, 1),
            xlabel="X",
            ylabel="Y",
            zlabel="Z",
            title="Canard axis",
        )
        if not self.roll:
            return

        roll = math.radians(self.roll[-1])
        pitch = math.radians(self.pitch[-1])
        axis_x = math.sin(pitch)
        axis_y = -math.sin(roll) * math.cos(pitch)
        axis_z = math.cos(roll) * math.cos(pitch)
        self.model_axis.quiver(0, 0, 0, axis_x, axis_y, axis_z, color="b")

    def update(self, _frame):
        self.read_serial()
        samples = range(len(self.roll))
        self.roll_line.set_data(samples, self.roll)
        self.pitch_line.set_data(samples, self.pitch)
        self.attitude_axis.set_xlim(0, max(200, len(self.roll)))
        self.draw_model()
        return self.roll_line, self.pitch_line

    def run(self):
        self.animation = animation.FuncAnimation(
            self.figure,
            self.update,
            interval=100,
            blit=False,
            cache_frame_data=False,
        )
        try:
            plt.show()
        finally:
            self.serial.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot canard attitude telemetry.")
    parser.add_argument("--port", default="/dev/ttyACM0", help="serial device path")
    parser.add_argument("--baudrate", type=int, default=115200)
    args = parser.parse_args()
    AttitudeMonitor(args.port, args.baudrate).run()
