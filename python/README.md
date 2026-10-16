# Python reader

Read ROS 1 and ROS 2 bags in Python without installing ROS. The current public
API is a generic iterator built on [Ternaris rosbags](https://ternaris.gitlab.io/rosbags/).
Rerun message converters can consume this iterator without changing bag access.

All commands below run from the repository root.

## Install

```bash
conda env create -f python/environment.yml
conda activate rosbag-rerun
python -m pip install -e ./python
```

This installs the reader and Rerun viewer dependencies. `-e` means changes to
the Python source take effect without reinstalling the package. The other
dependencies are installed automatically from `pyproject.toml`.

Python 3.12 is the default version (`.python-version`) and the minimum version
accepted by the Python package.

Run the Python tests from the repository root in that Conda environment:

```bash
python -m unittest discover -s python/tests -v
```

## Example scripts

Run these from the repository root after installing the package:
Put your own bag at `data/local/recording.bag`, or replace that path in the
commands below. Bag files are not included in the repository.

```bash
python python/examples/read_bag.py data/local/recording.bag --limit 5
python python/examples/read_bag.py data/local/recording.bag --pace --limit 5
```

For a bag containing `/camera`, `/imu`, and `/pose`, define independent
synchronization groups by repeating `--group`. The first topic in each group
is its reference topic:

```bash
python python/examples/sync_bag.py path/to/recording.bag \
  --group camera_imu /camera /imu \
  --group camera_pose /camera /pose \
  --tolerance-ms 10 --timestamp-source header --limit 5
```

Use `--timestamp-source bag` for recorded timestamps. The example also accepts
`--max-out-of-order-ms` when header timestamps arrive out of order.

Both scripts process messages as fast as they can by default. Add `--pace` to
wait according to the bag timestamps, or `--limit 0` to process the entire bag.
The scripts print record details as a starting point for a Rerun converter.

## Rerun viewer

The viewer example converts decoded odometry, stamped poses, raw 8-bit images,
and JPEG/PNG compressed images. It uses bag timestamps on the `bag_time` Rerun
timeline. Unsupported message types and image encodings are skipped.

```bash
python python/examples/view_bag.py data/local/recording.bag
python python/examples/view_bag.py data/local/recording.bag \
  --blueprint rerun/blueprints/odometry.rbl
python python/examples/view_bag.py data/local/recording.bag --pace
python python/examples/view_bag.py data/local/recording.bag \
  --blueprint rerun/blueprints/odometry.rbl --save rerun/recordings/vio.rrd
```

By default, the script sends data to a Rerun window as fast as it can read it.
Use `--pace` to follow recorded intervals. `--save` writes an `.rrd` file
instead of opening a window. Repeat `--topic` to select topics, or use
`--limit` for a short preview. The viewer defaults to Z-up coordinates; pass
`--coordinates y-down` for optical-camera coordinates.
The script reads one message and logs it before requesting the next message;
it does not load the bag into memory first.

`--blueprint` loads an `.rbl` layout into the live viewer, or includes it in
the exported `.rrd` when used with `--save`. The included
`rerun/blueprints/odometry.rbl` shows the `/world` pose hierarchy and expands
the `bag_time` timeline. Its application ID matches the script's
`rosbag_rerun` ID. Regenerate it after editing the layout with
`python python/examples/make_odometry_blueprint.py`.

The adapter is available in Python as `rosbag_rerun.viewer.RerunBagViewer`.
It logs pose coordinates as provided by each message; it does not resolve ROS
TF frames. Point clouds and other message types are not converted yet.
The earlier `rerun_viewer.py` is kept as a reference for specialized SLAM
visualizations.

## Read messages

```python
from rosbag_rerun import ROSBagReader

with ROSBagReader("data/local/recording.bag") as bag:
    print(bag.topics_and_types)
    for record in bag.iter_messages(topics=["/camera/image"]):
        # Dispatch record.msg_type and record.message to a Rerun converter.
        print(record.bag_timestamp_ns, record.topic, record.msg_type)
```

`iter_messages()` is lazy and accepts `topics`, `start_ns`, `stop_ns`, and
`deserialize=False` for raw bytes.
Deserialization errors include the topic, message type, and timestamp.
For bags lacking embedded message definitions, pass a suitable `typestore`
to `ROSBagReader`.

## Synchronize topics

```python
from rosbag_rerun import ROSBagReader, SyncGroup

groups = [
    SyncGroup(
        name="camera_imu",
        topics=("/camera", "/imu"),
        reference_topic="/camera",
        tolerance_ns=10_000_000,
        timestamp_source="header",  # use "bag" for recorded timestamps
    ),
    SyncGroup(
        name="camera_pose",
        topics=("/camera", "/pose"),
        reference_topic="/camera",
        tolerance_ns=20_000_000,
    ),
]

with ROSBagReader("data/local/recording.bag") as bag:
    for bundle in bag.iter_synchronized(groups):
        print(bundle.group, bundle.timestamp_ns, bundle.messages)
```

Each set matches the nearest available message from its other topics within
`tolerance_ns` of the reference message's selected timestamp. `timestamp_source`
defaults to `"bag"`; choose `"header"` for sensor capture times. Header matching
requires deserialized messages with `header.stamp` and buffers up to
`max_out_of_order_ns` (200 ms by default) of header-time reordering. Increase
that bound if the bag delivers older headers later; input must still be in
nondecreasing bag-time order. A topic may belong to
multiple groups; each group consumes messages independently. A message is used
at most once per topic within a group. Incomplete groups are skipped. Bundles
are emitted after the tolerance window has passed, or at end of bag, and are
ordered by reference timestamp. `iter_synchronized` also accepts any
chronologically ordered `BagMessage` iterable if the source is not a bag reader.

## Optional recorded-speed playback

`iter_playback` adds wall-clock waits between records. Use it when feeding a
live viewer at the bag's original pace. For conversion to a Rerun recording,
iterate directly and use the stored timestamps to control its timeline.

```python
from rosbag_rerun import ROSBagReader, iter_playback

with ROSBagReader("data/local/recording.bag") as bag:
    for record in iter_playback(bag.iter_messages(), rate=1.0):
        print(record.topic)
```

`rate=2.0` halves the waits; `rate=0` adds no waits.

## Data

Keep large or private bags in `data/` (or pass any path directly). The
directory is present in the repository, but its contents are ignored by Git.
The tests generate their own tiny bag and do not need local data.
