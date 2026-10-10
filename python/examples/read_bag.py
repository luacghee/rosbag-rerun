"""Print messages from a bag, optionally at their recorded pace.

Run from the repository root after installing the Python package:
    python python/examples/read_bag.py data/local/recording.bag --limit 5
    python python/examples/read_bag.py data/local/recording.bag --pace
"""

from __future__ import annotations

import argparse
from pathlib import Path

from rosbag_rerun import ROSBagReader, iter_playback


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="ROS 1 .bag file or ROS 2 bag directory")
    parser.add_argument("--topic", action="append", help="Topic to include; repeat for more")
    parser.add_argument("--limit", type=int, default=10, help="Messages to print; 0 means all")
    parser.add_argument("--pace", action="store_true", help="Wait between messages at recorded pace")
    args = parser.parse_args()
    if args.limit < 0:
        parser.error("--limit must be nonnegative")

    with ROSBagReader(args.bag) as reader:
        print("Topics:")
        for topic, msg_type in reader.topics_and_types.items():
            print(f"  {topic}: {msg_type}")

        messages = reader.iter_messages(topics=args.topic)
        if args.pace:
            messages = iter_playback(messages)

        for count, record in enumerate(messages, start=1):
            print(f"{record.bag_timestamp_ns}  {record.topic}  {record.msg_type}")
            if args.limit and count >= args.limit:
                break


if __name__ == "__main__":
    main()
