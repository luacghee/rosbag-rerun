# Shared Rerun files

These files are shared by the Python viewer and future C++ and Rust viewers.

- `blueprints/` holds reusable `.rbl` display layouts. The sample odometry
  layout uses application ID `rosbag_rerun` and shows poses under `/world`.
- `recordings/` is the suggested location for generated `.rrd` recordings.
  Recordings are ignored by Git; keep only redistributable examples if needed.

From the repository root, regenerate the sample layout with:

```bash
python python/examples/make_odometry_blueprint.py
```
