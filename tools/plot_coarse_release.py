"""Plot matched 1 kHz diagnostic captures. Requires matplotlib."""
import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("before", type=Path)
    parser.add_argument("after", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    fig, axes = plt.subplots(2, 2, figsize=(12, 7), sharex=True, constrained_layout=True)
    for column, direction in enumerate(("negative", "positive")):
        for directory, label, color in ((args.before, "Before", "#b94738"),
                                        (args.after, "After", "#147d73")):
            capture = json.loads((directory / f"strong-{direction}-dense.json").read_text())
            assert capture["status"] == 2 and capture["options"]["dense"] == 1
            samples = capture["samples"]
            release = 350 + capture["options"]["pulse_ms"]
            anchor = next(row["angle"] for row in samples if row["ms"] >= release)
            time = [row["ms"] - release for row in samples]
            angle = [(row["angle"] - anchor) * 180 / 3.141592653589793 for row in samples]
            axes[0, column].plot(time, angle, label=label, color=color, linewidth=1.6)
            axes[1, column].plot(time, [row["velocity"] for row in samples],
                                 label=label, color=color, linewidth=1.6)
        axes[0, column].set_title("Into detents" if column == 0 else "Outside endstop")
        axes[1, column].set_xlabel("Time after release (ms)")
        for axis in axes[:, column]:
            axis.axvline(0, color="#666666", linestyle="--", linewidth=0.8)
            axis.axhline(0, color="#bbbbbb", linewidth=0.7)
            axis.grid(alpha=0.2)
            axis.legend(loc="best")
    axes[0, 0].set_ylabel("Displacement since release (degrees)")
    axes[1, 0].set_ylabel("Measured fast velocity (rad/s)")
    fig.suptitle("Coarse strong: measured release response, 1 kHz capture\n"
                 "200 ms drive pulse, +/-25 rad/s target; current limited to 150 mA during drive")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=160)
    plt.close(fig)


if __name__ == "__main__":
    main()
