import colorsys
import numpy as np
import os
import rerun as rr
import rerun.blueprint as rrb



class RerunSingleRobotViewer:
    RGBD_SETUP = "rgbd"
    RGBD_IMU_SETUP = "rgbd_imu"

    def __init__(self, name, verbose=False, vslam_setup="rgbd"):
        if vslam_setup not in {self.RGBD_SETUP, self.RGBD_IMU_SETUP}:
            raise ValueError(
                f"Unsupported viewer SLAM setup {vslam_setup!r}; expected "
                f"{self.RGBD_SETUP!r} or {self.RGBD_IMU_SETUP!r}"
            )
        self.verbose = verbose
        if not self.verbose:
            # Hide Rust-side warnings/info from the viewer stack
            os.environ["RUST_LOG"] = "error"
            
        self.name = name
        self.vslam_setup = vslam_setup
        if self.vslam_setup == self.RGBD_IMU_SETUP:
            # ORB-SLAM3 inertial output is gravity aligned: X/Y form the
            # ground plane and +Z is up.
            self.view_coordinates = rr.ViewCoordinates.RIGHT_HAND_Z_UP
            self.eye_up = (0.0, 0.0, 1.0)
            self.ground_plane = rr.components.Plane3D.XY
            self.default_eye_offset_direction = np.array([0.0, -1.0, 0.6])
        else:
            # RGB-D starts in the optical-camera convention: +X right, +Y
            # down, and +Z forward.
            self.view_coordinates = rr.ViewCoordinates.RIGHT_HAND_Y_DOWN
            self.eye_up = (0.0, -1.0, 0.0)
            self.ground_plane = rr.components.Plane3D.ZX
            self.default_eye_offset_direction = np.array([0.0, -0.6, -1.0])
        self.load_cmap()
        self.image_contents = []

    def launch(self, images=True, spatial=True, text=False, spatial_bounds=None):
        self._setup_blueprint(
            images=images,
            spatial=spatial,
            text=text,
            spatial_bounds=spatial_bounds,
        )
        # self.log_world_origin()

    def _setup_blueprint(self, images, spatial, text, spatial_bounds=None):
        # Setup rerun visualizer
        rr.init(application_id=self.name, strict=True, spawn=True)
        # Coordinate metadata changes Rerun's orientation and grid
        # interpretation without modifying the logged SLAM poses.
        rr.log("3D", self.view_coordinates, static=True)

        viewer_contents = []
        if images:
            viewer_contents.append(rrb.Horizontal(contents=self.image_contents))
        if spatial:
            viewer_contents.append(
                self.setup_spatial_view(
                    name="3D",
                    origin="3D/",
                    spatial_bounds=spatial_bounds,
                )
            )
        if text:
            viewer_contents.append(rrb.TextLogView(name="Playback", origin="Playback/"))

        row_shares = []
        if images:
            row_shares.append(1)
        if spatial:
            row_shares.append(3)
        if text:
            row_shares.append(1)

        if len(viewer_contents) == 1:
            viewer_layout = viewer_contents[0]
        else:
            viewer_layout = rrb.Vertical(row_shares=row_shares, contents=viewer_contents)

        rr.send_blueprint(
            rrb.Blueprint(
                rrb.SelectionPanel(state="expanded"),
                rrb.BlueprintPanel(state="expanded"),
                rrb.TimePanel(state="collapsed"),
                viewer_layout,
        ), make_active=True)

    def add_image_topics(self, image_topics):
        print("[VIEWER] Adding image topics...")
        for name, topic in image_topics.items():
            self.image_contents.append(rrb.Spatial2DView(name=name, origin=topic))
            print(f"[VIEWER] Added '{topic}' image topic to viewer")

    def setup_spatial_view(self, name, origin, spatial_bounds=None):
        # Let Rerun derive the initial position and look target from the scene
        # bounds. The explicit up vector and grid match the selected SLAM mode.
        eye_controls_kwargs = {
            "kind": rrb.Eye3DKind.Orbital,
            "eye_up": self.eye_up,
            "spin_speed": 0.0,
        }
        if spatial_bounds is not None:
            bounds_min, bounds_max = (np.asarray(bound, dtype=float) for bound in spatial_bounds)
            center = (bounds_min + bounds_max) / 2.0
            scene_diagonal = max(float(np.linalg.norm(bounds_max - bounds_min)), 1.0)
            camera_distance = 2.0 * scene_diagonal
            eye_controls_kwargs.update(
                position=(
                    center + self.default_eye_offset_direction * camera_distance
                ),
                look_target=center,
            )
        eye_controls = rrb.EyeControls3D(**eye_controls_kwargs)
        line_grid = rrb.archetypes.LineGrid3D(
            plane=self.ground_plane.with_distance(0)
        )
        overrides = {
            # "/0rigin/": [rrb.EntityBehavior(visible=True)],
            # "/groundtruth/": [rrb.EntityBehavior(visible=False)],
            # "/nvblox/": [rrb.EntityBehavior(visible=True)],
            # "/object/": [rrb.EntityBehavior(visible=True)],
            # "/object/filtered_voxels": [rrb.EntityBehavior(visible=True)],
            # "/object/superquadrics": [rrb.EntityBehavior(visible=True)],
            # "/object/watertight_mesh": [rrb.EntityBehavior(visible=False)],
            # "/odom/": [rrb.EntityBehavior(visible=False)],
            # "/slam/": [rrb.EntityBehavior(visible=True)]
        }
        return rrb.Spatial3DView(
                    name=name,
                    origin=origin,
                    eye_controls=eye_controls,
                    line_grid=line_grid,
                    overrides=overrides
                )

    def setup_spatial2(self, name, origin):
        return rrb.Spatial3DView(
                    name="Association", 
                    origin="assoc/",
                    eye_controls=rrb.EyeControls3D(
                        kind=rrb.Eye3DKind.FirstPerson,
                        speed=20.0,
                    ),
                    line_grid=rrb.archetypes.LineGrid3D(plane=rr.components.Plane3D.XY.with_distance(0)),
                    overrides={
                        "/test/": [rrb.EntityBehavior(visible=True)],
                    }
                )

    def set_frame_idx_and_timestamp(self, frame_id, timestamp):
        rr.set_time("frame_id", sequence=frame_id)
        rr.set_time("timestamp", timestamp=self._timestamp_to_datetime64(timestamp))

    def set_timestamp(self, timestamp):
        rr.set_time("timestamp", timestamp=self._timestamp_to_datetime64(timestamp))

    @staticmethod
    def _timestamp_to_datetime64(timestamp):
        """Convert integer-like nanosecond timestamps to NumPy datetimes."""
        if isinstance(timestamp, np.datetime64):
            return timestamp.astype("datetime64[ns]")
        return np.datetime64(int(timestamp), "ns")

    def color_from_id(self, identifier):
        """Generate a color from an identifier."""
        return [
            (identifier * 17) % 256,
            (identifier * 31) % 256,
            (identifier * 47) % 256
        ]

    def log_world_origin(self, static=True):
        rr.log(
            "3D/0rigin",
            rr.Arrows3D(
                vectors=[[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]] # RGB for XYZ axes
            ),
            static=static            
        )

    def load_cmap(self, num_color=100):
        """Build a high-contrast palette whose neighboring IDs differ in hue."""
        golden_ratio = 0.618033988749895
        self.colors = []
        for i in range(num_color):
            hue = (i * golden_ratio) % 1.0
            rgb = colorsys.hsv_to_rgb(hue, 0.75, 1.0)
            self.colors.append([*rgb, 1.0])

    def clear_log(self, name, recursive=True):
        rr.log(name, rr.Clear(recursive=recursive))

    def clearlog_components(self, name, recursive=True):
        rr.log(name, rr.Clear(recursive=recursive))

    def log_text(self, name, text_log):
        rr.log(name, rr.TextLog(text_log))

    def log_pose(self, name, pose):
        if pose is None:
            if self.verbose:
                print(f"[VIEWER] Skipped logging pose: name='{name}' (pose=None)")
            return        
        rr.log(
            f"{name}/pose",
            rr.Transform3D(translation = pose.translation, quaternion = pose.rotation),
            rr.Arrows3D(vectors = np.eye(3) * 0.2, colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]]), # RGB for XYZ axes
        )

    def log_pose_and_sphere(self, name, pose, radius=0.1, colors=[255, 0 ,0]):
        if pose is None:
            if self.verbose:
              print(f"[VIEWER] Skipped logging pose: name='{name}' (pose=None)")
            return              
        rr.log(
            f"{name}/pose",
            rr.Transform3D(translation=pose.translation, quaternion=pose.rotation),
            rr.Arrows3D(vectors = np.eye(3) * 0.2, colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]]), # RGB for XYZ axes
            rr.Points3D(positions=[0, 0, 0], colors=colors, radii=radius)
        )

    def log_pose_and_ellipsoids(self, name, pose, radius=0.1, robot_id=0, static=True):
        colors = self.colors[robot_id]
        if pose is None:
            if self.verbose:
              print(f"[VIEWER] Skipped logging pose: name='{name}' (pose=None)")
            return        
        rr.log(
            f"{name}/pose",
            rr.Transform3D(translation = pose.translation, quaternion = pose.rotation),
            rr.Arrows3D(vectors = np.eye(3) * 0.2, colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]]), # RGB for XYZ axes
            rr.Ellipsoids3D(radii=[radius], colors=colors),
            static=static
        )

    def log_trajectory(self, name, trajectory, radius=0.01, colors=[255, 0 ,0], static=True):
        if trajectory is None:
            if self.verbose:
              print(f"[VIEWER] Skipped logging trajectory: name='{name}' (trajectory=None)")
            return        
        elif len(trajectory) == 0:
            if self.verbose:
              print(f"[VIEWER] Skipped logging trajectory: name='{name}' (trajectory=[])")
            return        
        trajectory_translations = [pose.translation for pose in trajectory]
        rr.log(
            f"{name}/trajectory", 
            rr.LineStrips3D(trajectory_translations, radii=radius, colors=colors), 
            static=static
        )

    def log_dashed_trajectory(self, name, trajectory, radius=0.01, colors=[255, 0 ,0], interval=5, static=True):
        if trajectory is None:
            if self.verbose:
              print(f"[VIEWER] Skipped logging dashed trajectory: name='{name}' (dashed trajectory=None)")
            return
        elif len(trajectory) == 0:
            if self.verbose:
              print(f"[VIEWER] Skipped logging dashed trajectory: name='{name}' (dashed trajectory=[])")
            return

        trajectory_translations = [pose.translation for pose in trajectory]

        # Stack the start and end points to form pairs (segments).
        trajectory_translations = np.array(trajectory_translations)
        start_indices = np.arange(0, len(trajectory_translations) - 1, interval)
        start_points = trajectory_translations[start_indices]
        end_points = trajectory_translations[start_indices + 1]

        dashed_trajectory_translations = np.stack([start_points, end_points], axis=1)

        rr.log(
            f"{name}/trajectory", 
            rr.LineStrips3D(dashed_trajectory_translations, radii=radius, colors=colors), 
            static=static
        )

    def log_posed_trajectory(
            self, 
            name, 
            traj, 
            robot_id=0,
            pose_radius=0.01, 
            traj_radius=0.01,
            traj_type="solid",
            dash_interval=5,
            static=False,
            label=None,
            label_offset=(0.0, -2.0, 0.0),
        ):
        if len(traj) == 0:
            if self.verbose:
              print(f"[VIEWER] Skipped logging posed trajectory: name='{name}' (trajectory=[])")
            return

        self.log_pose_and_ellipsoids(name, traj[-1], pose_radius, robot_id, static)
        if traj_type == "solid":
            self.log_trajectory(name, traj, traj_radius, self.colors[robot_id], static)
        elif traj_type == "dashed":
            self.log_dashed_trajectory(name, traj, traj_radius, self.colors[robot_id], dash_interval, static)

        if label is not None:
            label_position = traj[-1].translation + np.asarray(label_offset, dtype=float)
            self.log_3d_label(
                f"{name}/label",
                label_position,
                label,
                color=self.colors[robot_id],
                static=static,
            )

    def log_3d_label(self, name, position, label, color=None, radius=0.1, static=False):
        """Log a text label anchored to a 3D world position."""
        if color is None:
            color = [255, 255, 255]
        rr.log(
            name,
            rr.Points3D(
                positions=[np.asarray(position, dtype=float)],
                colors=[color],
                radii=[radius],
                labels=[label],
                show_labels=True,
            ),
            static=static,
        )

    def log_image(self, name, input_image, compress=False, static=True):
        if compress == True:
            rr.log(name, rr.Image(input_image).compress(jpeg_quality=80), static=static)
        elif compress == False:
            rr.log(name, rr.Image(input_image), static=static)

    def log_images(self, input_images, compress=False, static=True):
        for topic, input_image in input_images.items():
            self.log_image(topic, input_image, compress=compress, static=static)

    def log_image_pts(self, name, image_points, radius=5, colors=[0, 255, 0]):
        rr.log(name, rr.Points2D(image_points, radii=radius, colors=colors))

    def log_mesh3D(self, name, mesh, static=True):
        rr.log(
            name,
            rr.Mesh3D(
                vertex_positions=mesh.vertices,
                vertex_colors=mesh.vertex_colors,
                triangle_indices=mesh.triangles
            ),
            static=static
        )

    def log_meshes3D(self, name, meshes):
        for k, mesh in enumerate(meshes):
            self.log_mesh3D(f"{name}_{k}", mesh)

    # TODO
    def log_mesh3D_trimesh(self, name, mesh_trimesh):
        import open3d as o3d

        vertices_trimesh = np.asarray(mesh_trimesh.vertices)
        faces_trimesh = np.asarray(mesh_trimesh.faces)
        mesh_o3d = o3d.geometry.TriangleMesh()
        mesh_o3d.vertices = o3d.utility.Vector3dVector(vertices_trimesh)
        mesh_o3d.triangles = o3d.utility.Vector3iVector(faces_trimesh)
        mesh_color = np.tile(np.array((255, 255, 0)), (len(vertices_trimesh), 1))
        mesh_o3d.vertex_colors = o3d.utility.Vector3dVector(mesh_color)

        rr.log(
            name, 
            rr.Mesh3D(
                vertex_positions=mesh_o3d.vertices,
                vertex_colors=mesh_o3d.vertex_colors,
                triangle_indices=mesh_o3d.triangles
            )
        )

    def log_points3D(self, name, points3D, radius=0.05, colors=[0, 255, 0]):
        rr.log(name, rr.Points3D(points3D, radii=radius, colors=colors))
