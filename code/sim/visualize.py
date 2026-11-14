#!/usr/bin/env python3
"""Animate a Viikate software-in-the-loop flight.

Run a scenario like `sil.py run` and open an interactive window with the
trajectory in 3D and the logged state over time:

    python3 code/sim/visualize.py --rocket viikate_v1 --stop-at 20

Or visualize a run saved earlier:

    python3 code/sim/sil.py run --rocket viikate_v1 --stop-at 20 \
        --csv flight.csv
    python3 code/sim/visualize.py --input flight.csv

The 3D view uses a launch-fixed North/East/Up frame. The rocket is drawn
with its true attitude from the logged quaternion, but its size is
exaggerated when the trajectory is large, so it stays visible. The state
plots show altitude, downrange, speed, attitude and fin angles.

Keys: space pauses, left/right step, home/end jump. To render a video
instead of opening a window:

    python3 code/sim/visualize.py --rocket viikate_v1 --stop-at 20 \
        --save flight.mp4
"""

import argparse
import csv
import math
import sys
from pathlib import Path

import numpy as np

SIM_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SIM_DIR))

import sil  # noqa: E402

try:
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.widgets import Button, Slider
except ImportError as exc:  # pragma: no cover - depends on the environment
    raise SystemExit(
        "visualize.py needs matplotlib; run it inside `nix develop`"
    ) from exc

DEFAULT_LENGTH_M = 1.27
DEFAULT_FRONTAL_AREA_M2 = 0.00442
DEFAULT_CANARD_FWD_M = 0.4689
DEFAULT_CANARD_RADIUS_M = 0.0596975

REQUIRED_COLUMNS = (
    "t_ms",
    "pos_n_m",
    "pos_e_m",
    "pos_d_m",
    "vel_n_mps",
    "vel_e_mps",
    "vel_d_mps",
    "q_w",
    "q_x",
    "q_y",
    "q_z",
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
    *(f"cmd{i}_deg" for i in range(4)),
    *(f"fin{i}_deg" for i in range(4)),
)

# The plots use North/East/Up. The log's position is NED.
NED_TO_PLOT = np.diag([1.0, 1.0, -1.0])
FIN_COLORS = ("tab:blue", "tab:orange", "tab:green", "tab:red")
ATTITUDE_COLORS = {
    "roll": "tab:blue",
    "pitch": "tab:orange",
    "yaw": "tab:green",
    "tilt": "0.25",
}


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Animate a Viikate SIL flight and its logged state."
    )
    parser.add_argument(
        "--input",
        type=Path,
        help="SIL CSV from `sil.py run --csv`; do not run the simulation",
    )
    parser.add_argument(
        "--csv",
        type=Path,
        help="save the rows of this run before visualizing them",
    )
    parser.add_argument(
        "--plant", choices=sil.PLANTS, default="rocket", help="plant model"
    )
    parser.add_argument(
        "--rocket",
        help="design name or .rocket path (rocket plant only; also sets "
        "the drawing geometry with --input)",
    )
    parser.add_argument(
        "--stop-at", type=float, default=20.0, help="simulated seconds"
    )
    parser.add_argument(
        "--plant-param",
        type=sil._key_value,
        action="append",
        default=[],
        help="scenario parameter name=value (repeatable)",
    )
    parser.add_argument(
        "--servo-signs",
        type=sil._signs,
        help="four servo mounting signs, e.g. 1,1,-1,-1",
    )
    parser.add_argument(
        "--no-build", action="store_true", help="use an existing zephyr.exe"
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="simulated seconds per wall-clock second (default: 1)",
    )
    parser.add_argument(
        "--fps", type=int, default=30, help="animation frames per second"
    )
    parser.add_argument(
        "--save",
        type=Path,
        help="write an .mp4 or .gif instead of opening a window",
    )
    parser.add_argument(
        "--dpi", type=int, default=110, help="pixel density when saving"
    )
    args = parser.parse_args(argv)
    if args.input is not None and args.csv is not None:
        parser.error("--input and --csv cannot be used together")
    if args.speed <= 0:
        parser.error("--speed must be positive")
    if args.fps <= 0:
        parser.error("--fps must be positive")
    if args.stop_at <= 0:
        parser.error("--stop-at must be positive")
    return args


def load_csv(path):
    """Read the columns written by SimResult.write_csv()."""
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        rows = [{k: float(v) for k, v in row.items()} for row in reader]
    if not rows:
        raise SystemExit(f"{path}: no rows")
    missing = set(REQUIRED_COLUMNS) - set(rows[0])
    if missing:
        raise SystemExit(
            f"{path}: not a SIL log, missing columns: "
            + ", ".join(sorted(missing))
        )
    return rows


def read_design(path):
    """Read the few geometry values the drawing needs from a design file."""
    values = {}
    if path is not None and path.is_file():
        for line in path.read_text().splitlines():
            line = line.split("#", 1)[0].strip()
            key, sep, value = line.partition("=")
            if sep:
                values[key.strip()] = value.strip()

    def number(key, default):
        try:
            return float(values[key])
        except (KeyError, ValueError):
            return default

    area = number("frontal_area_m2", DEFAULT_FRONTAL_AREA_M2)
    body_radius = math.sqrt(max(area, 0.0) / math.pi)
    canard_radius = number(
        "canard_lift_radius_m", DEFAULT_CANARD_RADIUS_M
    )
    return {
        "length_m": number("length_m", DEFAULT_LENGTH_M),
        "body_radius_m": max(body_radius, 0.005),
        "canard_fwd_m": number("canard_fwd_of_cg_m", DEFAULT_CANARD_FWD_M),
        "canard_radius_m": max(canard_radius, body_radius + 0.01),
    }


def design_geometry(args, result):
    path = None
    if args.rocket:
        path = sil.rocket_path(args.rocket)
    elif result is not None and result.rocket:
        candidate = sil.ROCKETS_DIR / f"{result.rocket}.rocket"
        if candidate.is_file():
            path = candidate
    else:
        candidate = sil.ROCKETS_DIR / "viikate_v1.rocket"
        if candidate.is_file():
            path = candidate
    return read_design(path)


def quat_to_matrix(q):
    """Body-to-NED rotation matrix from a scalar-first quaternion."""
    w, x, y, z = q
    return np.array(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ]
    )


def prepare(rows):
    q = np.array(
        [[row["q_w"], row["q_x"], row["q_y"], row["q_z"]] for row in rows]
    )
    q /= np.linalg.norm(q, axis=1, keepdims=True)

    t = np.array([row["t_ms"] for row in rows]) / 1000.0
    north = np.array([row["pos_n_m"] for row in rows])
    east = np.array([row["pos_e_m"] for row in rows])
    down = np.array([row["pos_d_m"] for row in rows])
    vel_n = np.array([row["vel_n_mps"] for row in rows])
    vel_e = np.array([row["vel_e_mps"] for row in rows])
    vel_d = np.array([row["vel_d_mps"] for row in rows])

    # Body x in NED is the first column of the body-to-NED matrix.
    body_x_ned = np.column_stack(
        [
            1.0 - 2.0 * (q[:, 2] ** 2 + q[:, 3] ** 2),
            2.0 * (q[:, 1] * q[:, 2] + q[:, 3] * q[:, 0]),
            2.0 * (q[:, 1] * q[:, 3] - q[:, 2] * q[:, 0]),
        ]
    )
    tilt = np.degrees(np.arccos(np.clip(-body_x_ned[:, 2], -1.0, 1.0)))

    def unwrapped(name):
        return np.degrees(np.unwrap(np.radians([row[name] for row in rows])))

    return {
        "t": t,
        "north": north,
        "east": east,
        "up": -down,
        "vel_n": vel_n,
        "vel_e": vel_e,
        "vel_up": -vel_d,
        "speed": np.sqrt(vel_n**2 + vel_e**2 + vel_d**2),
        "horizontal_speed": np.hypot(vel_n, vel_e),
        "downrange": np.hypot(north, east),
        "altitude": -down,
        "roll": unwrapped("roll_deg"),
        "pitch": unwrapped("pitch_deg"),
        "yaw": unwrapped("yaw_deg"),
        "tilt": tilt,
        "cmd": np.array(
            [[row[f"cmd{i}_deg"] for row in rows] for i in range(4)]
        ),
        "fin": np.array(
            [[row[f"fin{i}_deg"] for row in rows] for i in range(4)]
        ),
        "quat": q,
    }


class Rocket:
    """The rocket drawn at the current pose, with exaggerated geometry."""

    def __init__(self, ax, geometry):
        self.geometry = geometry
        self.body, = ax.plot(
            [], [], [], color="#1f4e79", lw=5, solid_capstyle="round",
            zorder=10
        )
        self.canards = [
            ax.plot(
                [], [], [], color="tab:orange", lw=2.2, solid_capstyle="round",
                zorder=11
            )[0]
            for _ in range(4)
        ]
        self.nose, = ax.plot(
            [], [], [], marker="o", ms=4, color="tab:red", zorder=12
        )

    def update(self, position, rotation, scale):
        """Place the rocket at position (plot frame) with rotation
        body-to-plot and scale relative to the design's true size."""
        length = self.geometry["length_m"] * scale
        body_x = rotation[:, 0]
        body_y = rotation[:, 1]
        body_z = rotation[:, 2]
        nose = position + body_x * (0.5 * length)
        tail = position - body_x * (0.5 * length)
        self.body.set_data_3d(
            [tail[0], nose[0]], [tail[1], nose[1]], [tail[2], nose[2]]
        )
        self.nose.set_data_3d([nose[0]], [nose[1]], [nose[2]])

        root_radius = self.geometry["body_radius_m"] * scale
        tip_radius = self.geometry["canard_radius_m"] * scale
        if scale > 1.05:
            # Keep the canards visible when the scene is hundreds of metres
            # across; the body attitude itself is still exact.
            root_radius = min(root_radius, 0.02 * length)
            tip_radius = max(tip_radius, 0.14 * length)
        chord = 0.08 * length
        x = self.geometry["canard_fwd_m"] * scale
        for i in range(4):
            angle = i * 0.5 * math.pi
            radial = math.cos(angle) * body_y + math.sin(angle) * body_z
            root = position + body_x * x + radial * root_radius
            tip = position + body_x * x + radial * tip_radius
            back = tip - body_x * chord
            self.canards[i].set_data_3d(
                [root[0], tip[0], back[0]],
                [root[1], tip[1], back[1]],
                [root[2], tip[2], back[2]],
            )


def padded_range(values, minimum):
    lo = min(float(np.min(values)), 0.0)
    hi = max(float(np.max(values)), 0.0)
    if hi - lo < minimum:
        middle = 0.5 * (lo + hi)
        lo, hi = middle - 0.5 * minimum, middle + 0.5 * minimum
    pad = 0.08 * (hi - lo)
    return lo - pad, hi + pad


def setup_scene(ax, data, geometry):
    north = data["north"]
    east = data["east"]
    up = data["up"]
    length = geometry["length_m"]
    span = max(
        float(np.ptp(north)),
        float(np.ptp(east)),
        float(np.ptp(up)),
        2.0 * length,
    )
    scale = max(1.0, 0.04 * span / length)

    minimum = max(2.0 * length, 1.0)
    nlim = padded_range(north, minimum)
    elim = padded_range(east, minimum)
    ulim = padded_range(up, minimum)
    ax.set_xlim(nlim)
    ax.set_ylim(elim)
    ax.set_zlim(ulim)
    ax.set_box_aspect(
        (nlim[1] - nlim[0], elim[1] - elim[0], ulim[1] - ulim[0])
    )

    # Ground plane and a sparse grid so motion is readable.
    corners_n = np.array([[nlim[0], nlim[1]], [nlim[0], nlim[1]]])
    corners_e = np.array([[elim[0], elim[0]], [elim[1], elim[1]]])
    ax.plot_surface(
        corners_n,
        corners_e,
        np.zeros_like(corners_n),
        color="0.55",
        alpha=0.10,
        linewidth=0,
        shade=False,
    )
    for value in np.linspace(nlim[0], nlim[1], 5):
        ax.plot(
            [value, value], elim, [0.0, 0.0],
            color="0.65", lw=0.5, alpha=0.5
        )
    for value in np.linspace(elim[0], elim[1], 5):
        ax.plot(
            nlim, [value, value], [0.0, 0.0],
            color="0.65", lw=0.5, alpha=0.5
        )

    ax.scatter(
        [0.0], [0.0], [0.0], marker="s", s=55, color="tab:red",
        depthshade=False, zorder=8,
    )
    ax.plot(
        north, east, up, color="0.55", lw=0.9, alpha=0.55, zorder=2,
    )
    trail, = ax.plot([], [], [], color="tab:red", lw=1.6, zorder=3)
    rocket = Rocket(ax, geometry)

    ax.set_xlabel("North (m)")
    ax.set_ylabel("East (m)")
    ax.set_zlabel("Up (m)")
    ax.view_init(elev=24, azim=-58)
    ax.grid(True, alpha=0.25)
    ax.set_title("3D trajectory", pad=10)
    note = "red square: pad"
    if scale > 1.05:
        note += f"   rocket drawn {scale:.1f}x actual size"
    ax.text2D(
        0.02, 0.02, note, transform=ax.transAxes, fontsize=8, color="0.35",
        va="bottom",
    )
    return trail, rocket, scale


def main(argv=None):
    args = parse_args(argv)

    if args.save is not None:
        plt.switch_backend("Agg")
        if args.save.suffix.lower() not in (".mp4", ".gif"):
            raise SystemExit("--save must end in .mp4 or .gif")
    elif plt.get_backend().lower() == "agg":
        raise SystemExit(
            "visualize.py: no interactive display; use --save out.mp4"
        )

    if args.input is not None:
        rows = load_csv(args.input)
        result = None
        label = args.input.name
    else:
        exe = (
            sil.exe_path(args.plant)
            if args.no_build
            else sil.build(args.plant)
        )
        result = sil.run(
            exe,
            args.stop_at,
            dict(args.plant_param),
            args.servo_signs,
            args.rocket,
        )
        rows = result.rows
        if args.csv is not None:
            result.write_csv(args.csv)
        label = result.rocket or args.plant
        if result.end_reason:
            label += f" ({result.end_reason})"

    if not rows:
        raise SystemExit("no rows to visualize")
    data = prepare(rows)
    geometry = design_geometry(args, result)
    t = data["t"]
    n = len(t)
    dt = float(np.median(np.diff(t))) if n > 1 else 0.01
    dt = max(dt, 1e-6)

    fig = plt.figure(figsize=(16, 9))
    gs = fig.add_gridspec(
        2,
        3,
        width_ratios=(2.25, 1.0, 1.0),
        left=0.035,
        right=0.985,
        top=0.93,
        bottom=0.15,
        wspace=0.28,
        hspace=0.30,
    )
    ax3d = fig.add_subplot(gs[:, 0], projection="3d")
    ax_alt = fig.add_subplot(gs[0, 1])
    ax_speed = fig.add_subplot(gs[0, 2], sharex=ax_alt)
    ax_att = fig.add_subplot(gs[1, 1], sharex=ax_alt)
    ax_fin = fig.add_subplot(gs[1, 2], sharex=ax_alt)

    trail, rocket, scale = setup_scene(ax3d, data, geometry)
    status = ax3d.text2D(
        0.02,
        0.98,
        "",
        transform=ax3d.transAxes,
        va="top",
        family="monospace",
        fontsize=9,
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.78,
                  edgecolor="0.8"),
    )

    tlim = (min(0.0, float(t[0])), max(float(t[-1]), 0.05))

    def add_line(ax, values, label, **kwargs):
        line, = ax.plot(t, values, label=label, **kwargs)
        return line

    add_line(ax_alt, data["altitude"], "altitude", color="tab:blue")
    add_line(ax_alt, data["downrange"], "downrange", color="tab:orange")
    ax_alt.set_ylabel("m")
    ax_alt.legend(loc="best", fontsize=8)
    ax_alt.tick_params(labelbottom=False)

    add_line(ax_speed, data["speed"], "total", color="tab:blue")
    add_line(ax_speed, data["vel_up"], "vertical", color="tab:orange")
    ax_speed.set_ylabel("m/s")
    ax_speed.legend(loc="best", fontsize=8)
    ax_speed.tick_params(labelbottom=False)

    for name in ("roll", "pitch", "yaw", "tilt"):
        add_line(
            ax_att,
            data[name],
            name,
            color=ATTITUDE_COLORS[name],
            lw=1.4,
        )
    ax_att.axhline(0.0, color="0.6", lw=0.6)
    ax_att.set_ylabel("deg")
    ax_att.legend(loc="best", fontsize=8, ncol=2)
    ax_att.set_xlabel("time (s)")

    for i in range(4):
        add_line(
            ax_fin,
            data["fin"][i],
            f"fin {i}",
            color=FIN_COLORS[i],
            lw=1.4,
        )
    for i in range(4):
        add_line(
            ax_fin,
            data["cmd"][i],
            f"cmd {i}",
            color=FIN_COLORS[i],
            lw=1.0,
            ls="--",
            alpha=0.65,
        )
    ax_fin.axhline(0.0, color="0.6", lw=0.6)
    ax_fin.set_ylabel("deg")
    ax_fin.legend(loc="best", fontsize=6.5, ncol=2)
    ax_fin.set_xlabel("time (s)")

    cursors = []
    for ax in (ax_alt, ax_speed, ax_att, ax_fin):
        ax.set_xlim(tlim)
        ax.margins(x=0)
        ax.grid(True, alpha=0.3)
        cursors.append(
            ax.axvline(t[0], color="0.2", lw=0.8, alpha=0.8)
        )

    fig.suptitle(f"Viikate SIL — {label}", fontsize=13)

    current = 0
    frame_pos = 0.0
    playing = True
    updating_slider = False
    saving = args.save is not None

    def show_index(index):
        nonlocal current
        index = int(np.clip(index, 0, n - 1))
        current = index
        trail.set_data_3d(
            data["north"][: index + 1],
            data["east"][: index + 1],
            data["up"][: index + 1],
        )
        rotation = NED_TO_PLOT @ quat_to_matrix(data["quat"][index])
        rocket.update(
            np.array(
                [
                    data["north"][index],
                    data["east"][index],
                    data["up"][index],
                ]
            ),
            rotation,
            scale,
        )
        for cursor in cursors:
            cursor.set_xdata([t[index], t[index]])
        status.set_text(
            f"t     {t[index]:7.2f} s\n"
            f"alt   {data['altitude'][index]:7.1f} m\n"
            f"speed {data['speed'][index]:7.1f} m/s\n"
            f"v_up  {data['vel_up'][index]:7.1f} m/s"
        )
        fig.canvas.draw_idle()

    ax_slider = fig.add_axes((0.19, 0.045, 0.60, 0.025))
    slider = Slider(
        ax_slider,
        "Time (s)",
        tlim[0],
        tlim[1],
        valinit=t[0],
        valfmt="%.2f s",
    )
    ax_button = fig.add_axes((0.045, 0.032, 0.10, 0.045))
    button = Button(ax_button, "Pause")

    def on_slider(value):
        nonlocal frame_pos
        index = int(np.searchsorted(t, value))
        index = int(np.clip(index, 0, n - 1))
        if not updating_slider:
            frame_pos = float(index)
        show_index(index)

    slider.on_changed(on_slider)

    def toggle(_event=None):
        nonlocal playing, frame_pos
        if playing:
            playing = False
            button.label.set_text("Play")
        else:
            if frame_pos >= n - 1 - 0.5:
                frame_pos = 0.0
            playing = True
            button.label.set_text("Pause")
        fig.canvas.draw_idle()

    button.on_clicked(toggle)

    def on_key(event):
        nonlocal frame_pos
        if event.key == " ":
            toggle()
        elif event.key in ("left", "right"):
            if playing:
                toggle()
            step = max(1, int(round(args.speed / (args.fps * dt))))
            delta = -step if event.key == "left" else step
            index = int(np.clip(current + delta, 0, n - 1))
            frame_pos = float(index)
            slider.set_val(t[index])
        elif event.key == "home":
            frame_pos = 0.0
            slider.set_val(t[0])
        elif event.key == "end":
            frame_pos = float(n - 1)
            slider.set_val(t[-1])

    fig.canvas.mpl_connect("key_press_event", on_key)

    step = max(args.speed / (args.fps * dt), 0.01)
    interval = max(int(round(1000.0 / args.fps)), 1)
    n_display = max(1, int(math.ceil((n - 1) / step)) + 1)

    def animate(_frame):
        nonlocal frame_pos, playing, updating_slider
        if not playing:
            return
        frame_pos = min(frame_pos + step, n - 1)
        index = int(round(frame_pos))
        if frame_pos >= n - 1:
            playing = False
            button.label.set_text("Play")
        if saving:
            show_index(index)
        else:
            updating_slider = True
            try:
                slider.set_val(t[index])
            finally:
                updating_slider = False
        return ()

    show_index(0)
    if saving:
        animation = FuncAnimation(
            fig,
            animate,
            frames=n_display,
            interval=interval,
            blit=False,
            cache_frame_data=False,
        )
        animation.save(args.save, fps=args.fps, dpi=args.dpi)
        plt.close(fig)
    else:
        animation = FuncAnimation(
            fig,
            animate,
            interval=interval,
            blit=False,
            cache_frame_data=False,
        )
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
