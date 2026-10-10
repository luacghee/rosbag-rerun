"""Print synchronized groups from a bag, optionally at recorded pace.

Each --group is NAME REFERENCE_TOPIC [OTHER_TOPIC ...]. Repeat it to define
independent groups; a topic may appear in more than one group.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

from rosbag_rerun import ROSBagReader, SyncGroup, iter_playback


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="ROS 1 .bag file or ROS 2 bag directory")
    parser.add_argument(
        "--group",
        action="append",
        nargs="+",
        required=True,
        metavar="ITEM",
        help="NAME REFERENCE_TOPIC [OTHER_TOPIC ...]; repeat for more groups",
    )
    parser.add_argument("--tolerance-ms", type=float, default=30.0)
    parser.add_argument("--limit", type=int, default=10, help="Bundles to print; 0 means all")
    parser.add_argument("--pace", action="store_true", help="Wait between bundles at recorded pace")
    args = parser.parse_args()
    if args.limit < 0:
        parser.error("--limit must be nonnegative")
    if not math.isfinite(args.tolerance_ms) or args.tolerance_ms < 0:
        parser.error("--tolerance-ms must be finite and nonnegative")

    tolerance_ns = round(args.tolerance_ms * 1_000_000)
    groups = []
    for values in args.group:
        if len(values) < 2:
            parser.error("each --group needs a name and reference topic")
        groups.append(
            SyncGroup(
                name=values[0],
                topics=tuple(values[1:]),
                reference_topic=values[1],
                tolerance_ns=tolerance_ns,
            )
        )

    with ROSBagReader(args.bag) as reader:
        bundles = reader.iter_synchronized(groups)
        if args.pace:
            bundles = iter_playback(bundles)

        for count, bundle in enumerate(bundles, start=1):
            offsets = ", ".join(
                f"{topic}: {(record.bag_timestamp_ns - bundle.timestamp_ns) / 1_000_000:+.1f} ms"
                for topic, record in bundle.messages.items()
            )
            print(f"{bundle.timestamp_ns}  {bundle.group}  [{offsets}]")
            if args.limit and count >= args.limit:
                break


if __name__ == "__main__":
    main()
