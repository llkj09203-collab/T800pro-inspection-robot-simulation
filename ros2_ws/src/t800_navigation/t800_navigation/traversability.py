"""Robot-aware 2.5D elevation and traversability grid construction."""

from dataclasses import dataclass
from collections import deque
import math

import numpy as np


@dataclass(frozen=True)
class TraversabilityConfig:
    resolution: float = 0.10
    max_step_height: float = 0.18
    max_slope_deg: float = 30.0
    max_roughness: float = 0.06
    robot_clearance_height: float = 0.75
    ground_band: float = 0.05
    min_points_per_cell: int = 2
    padding_cells: int = 2
    ground_quantile: float = 0.15
    ground_seed_bin: float = 0.05
    ground_seed_tolerance: float = 0.12
    inflation_radius: float = 0.0


@dataclass
class TraversabilityGrid:
    origin_x: float
    origin_y: float
    resolution: float
    ground: np.ndarray
    ceiling: np.ndarray
    mean: np.ndarray
    roughness: np.ndarray
    slope_deg: np.ndarray
    count: np.ndarray
    cost: np.ndarray

    @property
    def height(self):
        return int(self.cost.shape[0])

    @property
    def width(self):
        return int(self.cost.shape[1])


def _validate(points, config):
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    points = points[np.isfinite(points).all(axis=1)]
    if config.resolution <= 0.0:
        raise ValueError("resolution must be positive")
    if not 0.0 < config.max_step_height < config.robot_clearance_height:
        raise ValueError("step height must be positive and below robot clearance")
    if not 0.0 < config.max_slope_deg < 90.0:
        raise ValueError("max slope must be between zero and 90 degrees")
    if not 0.0 <= config.ground_quantile < 0.5:
        raise ValueError("ground quantile must be in [0, 0.5)")
    if config.ground_seed_bin <= 0.0 or config.ground_seed_tolerance < 0.0:
        raise ValueError("ground seed parameters must be non-negative")
    if config.inflation_radius < 0.0:
        raise ValueError("inflation radius must be non-negative")
    return points


def _cell_lower_quantiles(flat, z, cells, quantile):
    """Return a robust lower surface per cell without interpolating sparse pairs."""
    order = np.lexsort((z, flat))
    sorted_flat = flat[order]
    sorted_z = z[order]
    starts = np.searchsorted(sorted_flat, np.arange(cells), side="left")
    ends = np.searchsorted(sorted_flat, np.arange(cells), side="right")
    sizes = ends - starts
    candidate = np.full(cells, np.nan, dtype=np.float64)
    occupied = sizes > 0
    ranks = np.floor(quantile * np.maximum(sizes - 1, 0)).astype(np.int64)
    indices = starts[occupied] + ranks[occupied]
    candidate[occupied] = sorted_z[indices]
    return candidate


def _reachable_ground(candidate, valid, width, height, config):
    """Grow ground from the dominant low surface using step-continuity."""
    values = candidate[valid]
    accepted = np.zeros(width * height, dtype=bool)
    if values.size == 0:
        return accepted

    bins = np.floor(values / config.ground_seed_bin).astype(np.int64)
    unique, counts = np.unique(bins, return_counts=True)
    # Prefer the lower bin if two horizontal surfaces have the same support.
    dominant_bin = unique[np.flatnonzero(counts == counts.max())[0]]
    reference = (dominant_bin + 0.5) * config.ground_seed_bin
    seeds = np.flatnonzero(valid & (np.abs(candidate - reference) <=
                                    config.ground_seed_tolerance))
    if seeds.size == 0:
        seeds = np.array([np.flatnonzero(valid)[np.argmin(values)]])

    queue = deque(int(index) for index in seeds)
    accepted[seeds] = True
    while queue:
        current = queue.popleft()
        y, x = divmod(current, width)
        for ny, nx in ((y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1)):
            if ny < 0 or ny >= height or nx < 0 or nx >= width:
                continue
            neighbour = ny * width + nx
            if accepted[neighbour] or not valid[neighbour]:
                continue
            if abs(candidate[neighbour] - candidate[current]) <= config.max_step_height:
                accepted[neighbour] = True
                queue.append(neighbour)
    return accepted


def _inflate(mask, radius_cells):
    if radius_cells <= 0:
        return mask
    height, width = mask.shape
    result = mask.copy()
    ys, xs = np.where(mask)
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if dx * dx + dy * dy > radius_cells * radius_cells:
                continue
            target_y = ys + dy
            target_x = xs + dx
            valid = ((target_y >= 0) & (target_y < height) &
                     (target_x >= 0) & (target_x < width))
            result[target_y[valid], target_x[valid]] = True
    return result


def build_traversability_grid(points, config=TraversabilityConfig()):
    """Compress a global XYZ cloud into robot-specific 2.5D grid layers.

    Returns None for an empty cloud.  Occupancy values follow nav_msgs/OccupancyGrid:
    -1 unknown, 0..64 traversable cost, and 100 non-traversable.
    """
    points = _validate(points, config)
    if len(points) == 0:
        return None

    padding = config.padding_cells * config.resolution
    origin_x = math.floor((points[:, 0].min() - padding) / config.resolution) * config.resolution
    origin_y = math.floor((points[:, 1].min() - padding) / config.resolution) * config.resolution
    width = int(math.floor((points[:, 0].max() - origin_x + padding) / config.resolution)) + 1
    height = int(math.floor((points[:, 1].max() - origin_y + padding) / config.resolution)) + 1

    ix = np.floor((points[:, 0] - origin_x) / config.resolution).astype(np.int64)
    iy = np.floor((points[:, 1] - origin_y) / config.resolution).astype(np.int64)
    flat = iy * width + ix
    cells = width * height

    count = np.zeros(cells, dtype=np.int32)
    z_max = np.full(cells, -np.inf, dtype=np.float64)
    np.add.at(count, flat, 1)
    np.maximum.at(z_max, flat, points[:, 2])

    sampled = count >= config.min_points_per_cell
    ground_candidate = _cell_lower_quantiles(
        flat, points[:, 2], cells, config.ground_quantile)
    known = _reachable_ground(
        ground_candidate, sampled, width, height, config)
    mean = np.full(cells, np.nan, dtype=np.float64)
    roughness = np.full(cells, np.nan, dtype=np.float64)

    # Roughness describes the local ground surface, not the entire vertical
    # column (which may contain walls, furniture, or overhead returns).
    ground_delta = points[:, 2] - ground_candidate[flat]
    ground_hit = ((ground_delta >= -config.ground_band) &
                  (ground_delta <= config.ground_band) & known[flat])
    ground_count = np.zeros(cells, dtype=np.int32)
    ground_sum = np.zeros(cells, dtype=np.float64)
    ground_sq_sum = np.zeros(cells, dtype=np.float64)
    np.add.at(ground_count, flat[ground_hit], 1)
    np.add.at(ground_sum, flat[ground_hit], points[ground_hit, 2])
    np.add.at(ground_sq_sum, flat[ground_hit], points[ground_hit, 2] ** 2)
    ground_valid = known & (ground_count > 0)
    mean[ground_valid] = ground_sum[ground_valid] / ground_count[ground_valid]
    variance = np.maximum(
        0.0, ground_sq_sum[ground_valid] / ground_count[ground_valid] -
        mean[ground_valid] ** 2)
    roughness[ground_valid] = np.sqrt(variance)

    # A return blocks the body only when it lies above the step-over band and
    # below the robot's required vertical clearance.  Returns higher than the
    # body envelope are retained in the height layers but do not block travel.
    relative_height = points[:, 2] - ground_candidate[flat]
    body_hit = ((relative_height > config.max_step_height) &
                (relative_height < config.robot_clearance_height))
    body_blocked = np.zeros(cells, dtype=np.uint8)
    np.maximum.at(body_blocked, flat, body_hit.astype(np.uint8))

    ground_flat = np.full(cells, np.nan, dtype=np.float64)
    ground_flat[known] = ground_candidate[known]
    ground = ground_flat.reshape(height, width)
    z_max[~sampled] = np.nan
    ceiling = z_max.reshape(height, width)
    known_2d = known.reshape(height, width)
    slope_deg = np.full((height, width), np.nan, dtype=np.float64)
    # Use only valid adjacent ground cells.  Missing neighbours remain unknown
    # instead of being silently treated as flat terrain.
    slope_samples = np.zeros((height, width), dtype=np.float64)
    slope_valid = np.zeros((height, width), dtype=bool)
    for dy, dx in ((0, 1), (1, 0)):
        if height <= dy or width <= dx:
            continue
        lhs = (slice(0, height - dy), slice(0, width - dx))
        rhs = (slice(dy, height), slice(dx, width))
        a = ground[lhs]
        b = ground[rhs]
        valid = known_2d[lhs] & known_2d[rhs]
        slope = np.degrees(np.arctan2(np.abs(b - a), config.resolution))
        target = slope_samples[lhs]
        target[valid] = np.maximum(target[valid], slope[valid])
        slope_valid[lhs] |= valid
        shifted = slope_samples[rhs]
        shifted[valid] = np.maximum(shifted[valid], slope[valid])
        slope_valid[rhs] |= valid
    slope_deg[slope_valid] = slope_samples[slope_valid]

    cost = np.full(cells, -1, dtype=np.int8)
    height_span = z_max - ground_candidate
    step_cost = np.clip(height_span / config.max_step_height * 50.0, 0.0, 50.0)
    rough_cost = np.clip(roughness / config.max_roughness * 50.0, 0.0, 50.0)
    slope_flat = slope_deg.reshape(-1)
    slope_cost = np.clip(slope_flat / config.max_slope_deg * 60.0, 0.0, 60.0)
    cost_layers = np.stack((step_cost, rough_cost, slope_cost))
    traversable_cost = np.max(np.nan_to_num(cost_layers, nan=0.0), axis=0)
    cost[known] = np.rint(traversable_cost[known]).astype(np.int8)
    # Sampled cells rejected by the connected-ground model are obstacle
    # surfaces rather than invented elevated ground.
    blocked = ((sampled & ~known) | body_blocked.astype(bool) |
               (roughness > config.max_roughness) |
               (slope_flat > config.max_slope_deg))
    cost[blocked] = 100
    if config.inflation_radius > 0.0:
        radius = int(math.ceil(config.inflation_radius / config.resolution))
        inflated = _inflate((cost == 100).reshape(height, width), radius).reshape(-1)
        cost[inflated & (cost >= 0)] = 100

    return TraversabilityGrid(
        origin_x=origin_x,
        origin_y=origin_y,
        resolution=config.resolution,
        ground=ground.astype(np.float32),
        ceiling=ceiling.astype(np.float32),
        mean=mean.reshape(height, width).astype(np.float32),
        roughness=roughness.reshape(height, width).astype(np.float32),
        slope_deg=slope_deg.astype(np.float32),
        count=count.reshape(height, width),
        cost=cost.reshape(height, width),
    )
