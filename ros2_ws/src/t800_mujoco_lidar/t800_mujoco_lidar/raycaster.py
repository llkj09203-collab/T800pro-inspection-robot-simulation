"""MuJoCo CPU multi-ray implementation."""

import numpy as np

from .model import build_ray_directions, world_points_to_lidar


class Raycaster:
    def __init__(self, model_path, mount_camera, horizontal_rays, vertical_rays,
                 horizontal_fov_deg, vertical_up_deg, vertical_down_deg,
                 min_range_m, max_range_m, angular_stride):
        import mujoco

        self.mj = mujoco
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        self.mount = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_CAMERA, mount_camera)
        self.base = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "LINK_BASE")
        if self.mount < 0 or self.base < 0:
            raise ValueError("model lacks the lidar mount camera or LINK_BASE")
        self.rays, self.rings = build_ray_directions(
            horizontal_rays, vertical_rays, horizontal_fov_deg,
            vertical_up_deg, vertical_down_deg, angular_stride)
        self.min_range = float(min_range_m)
        self.max_range = float(max_range_m)
        if not 0.0 < self.min_range < self.max_range:
            raise ValueError("range limits are invalid")
        self.geomgroup = np.zeros(6, dtype=np.uint8)
        self.geomgroup[0] = 1
        self.geomgroup[5] = 1

    def _apply_state(self, state):
        model, data = self.model, self.data
        if state["ngeom"] != model.ngeom:
            raise ValueError("MuJoCo scene and lidar model do not match")
        for name, size in (("qpos", model.nq), ("qvel", model.nv),
                           ("mocap_pos", 3 * model.nmocap),
                           ("mocap_quat", 4 * model.nmocap)):
            value = np.asarray(state[name], dtype=np.float64)
            if value.shape != (size,) or not np.isfinite(value).all():
                raise ValueError(f"invalid {name} state")
            getattr(data, name)[:] = value.reshape(getattr(data, name).shape)
        geoms = np.asarray(state["geoms"], dtype=np.float64)
        expected = set(np.flatnonzero(model.geom_group == 5).tolist())
        if not expected:
            if geoms.size:
                raise ValueError("unexpected obstacle state in obstacle-free scene")
        else:
            if geoms.shape != (len(expected), 11) or not np.isfinite(geoms).all():
                raise ValueError("invalid obstacle state")
            if set(geoms[:, 0].tolist()) != expected:
                raise ValueError("obstacle IDs do not match the scene")
            for geom in geoms:
                index = int(geom[0])
                model.geom_pos[index] = geom[1:4]
                model.geom_quat[index] = geom[4:8]
                model.geom_size[index] = geom[8:11]
        data.time = float(state["sim_time"])
        self.mj.mj_forward(model, data)
        if (not np.allclose(data.cam_xpos[self.mount], state["origin"], atol=1e-5, rtol=0)
                or not np.allclose(data.cam_xmat[self.mount], state["sensor_matrix"],
                                   atol=1e-5, rtol=0)):
            raise ValueError("lidar mount pose differs from the native snapshot")

    def capture(self, state):
        self._apply_state(state)
        origin = self.data.cam_xpos[self.mount].copy()
        camera_matrix = self.data.cam_xmat[self.mount].copy().reshape(3, 3)
        world_dirs = np.ascontiguousarray(self.rays @ camera_matrix.T, dtype=np.float64)
        count = len(world_dirs)
        distances = np.full(count, -1.0, dtype=np.float64)
        geom_ids = np.full(count, -1, dtype=np.int32)
        args = [self.model, self.data, np.ascontiguousarray(origin), world_dirs.ravel(),
                self.geomgroup, True, self.base, geom_ids, distances]
        if "normal:" in (self.mj.mj_multiRay.__doc__ or ""):
            args.append(None)
        self.mj.mj_multiRay(*args, count, self.max_range)
        valid = (distances > self.min_range) & (distances < self.max_range)
        world_points = origin + distances[valid, None] * world_dirs[valid]
        # Preserve each hit's original angular-ray index. Debug visualization
        # samples these stable IDs instead of positions in the changing list of
        # valid returns, preventing the displayed beams from jumping around.
        ray_ids = np.flatnonzero(valid).astype(np.uint32, copy=False)
        return (world_points_to_lidar(world_points, origin, camera_matrix),
                self.rings[valid], ray_ids)
