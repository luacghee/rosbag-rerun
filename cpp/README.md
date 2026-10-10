# C++ (future work)

There is no C++ implementation yet. The intended scope is a ROS-independent
bag reader with an iterator for messages, synchronization groups, and optional
recorded-pace playback. It should expose message data for Rerun converters.

The [Python implementation](../python/README.md) defines the current behavior.
Shared Rerun layouts and recording output belong under [`rerun/`](../rerun/README.md).
