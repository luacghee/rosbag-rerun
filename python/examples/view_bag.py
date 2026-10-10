"""Visualize common ROS bag messages in Rerun or save them to an .rrd file.

Run from the repository root after installing the Python package:
    python python/examples/view_bag.py data/local/recording.bag
    python python/examples/view_bag.py data/local/recording.bag --blueprint rerun/blueprints/odometry.rbl
    python python/examples/view_bag.py data/local/recording.bag --save rerun/recordings/recording.rrd
"""

from __future__ import annotations

import argparse
from pathlib import Path

from rosbag_rerun import ROSBagReader, iter_playback
from rosbag_rerun.viewer import RerunBagViewer, SUPPORTED_MESSAGE_TYPES


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="ROS 1 .bag file or ROS 2 bag directory")
    parser.add_argument("--topic", action="append", help="Topic to include; repeat for more")
    parser.add_argument("--pace", action="store_true", help="Wait between messages at recorded pace")
    parser.add_argument("--limit", type=int, default=0, help="Messages to read; 0 means all")
    parser.add_argument("--save", type=Path, help="Save an .rrd file instead of opening the viewer")
    parser.add_argument("--blueprint", type=Path, help="Load an .rbl layout into the viewer")
    parser.add_argument("--coordinates", choices=("z-up", "y-down"), default="z-up")
    args = parser.parse_args()
    if args.limit < 0:
        parser.error("--limit must be nonnegative")
    if args.blueprint is not None and not args.blueprint.expanduser().is_file():
        parser.error(f"blueprint file not found: {args.blueprint}")

    with ROSBagReader(args.bag) as reader:
        available = {
            topic for topic, msg_type in reader.topics_and_types.items()
            if msg_type in SUPPORTED_MESSAGE_TYPES
        }
        topics = available if args.topic is None else available.intersection(args.topic)
        if not topics:
            parser.error("no supported pose or image topics selected")
        viewer = RerunBagViewer(
            save_path=args.save,
            blueprint_path=args.blueprint,
            coordinates=args.coordinates,
        )
        read_count = logged_count = 0
        for record in iter_playback(
            reader.iter_messages(topics=sorted(topics)),
            rate=1.0 if args.pace else 0.0,
        ):
            read_count += 1
            logged_count += viewer.log(record)
            if args.limit and read_count >= args.limit:
                break
    print(f"Logged {logged_count} of {read_count} messages to Rerun")


if __name__ == "__main__":
    main()
