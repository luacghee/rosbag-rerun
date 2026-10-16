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
    parser.add_argument("--timestamp-source", choices=("bag", "header"), default="bag",
                        help="Clock used to match messages (default: bag)")
    parser.add_argument("--max-out-of-order-ms", type=float, default=200.0,
                        help="Header-time reordering window (default: 200 ms)")
    parser.add_argument("--limit", type=int, default=10, help="Bundles to print; 0 means all")
    parser.add_argument("--pace", action="store_true", help="Wait between bundles at recorded pace")
    args = parser.parse_args()
    if args.limit < 0:
        parser.error("--limit must be nonnegative")
    if not math.isfinite(args.tolerance_ms) or args.tolerance_ms < 0:
        parser.error("--tolerance-ms must be finite and nonnegative")

    if not math.isfinite(args.max_out_of_order_ms) or args.max_out_of_order_ms < 0:
        parser.error("--max-out-of-order-ms must be finite and nonnegative")
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
                timestamp_source=args.timestamp_source,
                max_out_of_order_ns=round(args.max_out_of_order_ms * 1_000_000),
            )
        )

    with ROSBagReader(args.bag) as reader:
        bundles = reader.iter_synchronized(groups)
        if args.pace:
            bundles = iter_playback(bundles)

        for count, bundle in enumerate(bundles, start=1):
            def selected_time_ns(record):
                if args.timestamp_source == "bag":
                    return record.bag_timestamp_ns
                stamp = record.message.header.stamp
                return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

            offsets = ", ".join(
                f"{topic}: {(selected_time_ns(record) - bundle.timestamp_ns) / 1_000_000:+.1f} ms"
                for topic, record in bundle.messages.items()
            )
            print(f"{bundle.timestamp_ns}  {bundle.group}  [{offsets}]")
            if args.limit and count >= args.limit:
                break


if __name__ == "__main__":
    main()
