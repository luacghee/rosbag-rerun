"""Regenerate the shared sample odometry layout using the installed Rerun SDK."""

from pathlib import Path

import rerun.blueprint as rrb


def main() -> None:
    output = Path(__file__).resolve().parents[2] / "rerun/blueprints/odometry.rbl"
    blueprint = rrb.Blueprint(
        rrb.Spatial3DView(name="Odometry", origin="/world"),
        rrb.TimePanel(state="expanded", timeline="bag_time"),
    )
    blueprint.save("rosbag_rerun", output)
    print(f"Saved {output}")


if __name__ == "__main__":
    main()
