import numpy as np

from t800_navigation.traversability import (
    TraversabilityConfig, build_traversability_grid,
)


def cell_points(x, y, heights):
    return [[x, y, z] for z in heights]


def test_low_step_is_traversable_but_wall_is_blocked():
    points = np.array(
        cell_points(0.05, 0.05, [0.00, 0.04, 0.10]) +
        cell_points(0.25, 0.05, [0.00, 0.30, 0.60])
    )
    grid = build_traversability_grid(points, TraversabilityConfig(
        resolution=0.2, max_step_height=0.18, max_roughness=1.0,
        max_slope_deg=89.0, min_points_per_cell=2, padding_cells=0))
    assert 0 <= grid.cost[0, 0] < 100
    assert grid.cost[0, 1] == 100


def test_overhead_return_above_clearance_does_not_block():
    points = np.array(cell_points(0.05, 0.05, [0.0, 1.0]))
    grid = build_traversability_grid(points, TraversabilityConfig(
        resolution=0.2, max_step_height=0.18, robot_clearance_height=0.75,
        max_roughness=2.0, max_slope_deg=89.0,
        min_points_per_cell=2, padding_cells=0))
    assert grid.cost[0, 0] < 100
    assert np.isclose(grid.ceiling[0, 0], 1.0)


def test_steep_adjacent_ground_cells_are_blocked():
    points = np.array(
        cell_points(0.05, 0.05, [0.0, 0.0]) +
        cell_points(0.25, 0.05, [0.2, 0.2])
    )
    grid = build_traversability_grid(points, TraversabilityConfig(
        resolution=0.2, max_step_height=0.3, max_roughness=1.0,
        max_slope_deg=30.0, min_points_per_cell=2, padding_cells=0))
    assert np.all(grid.cost[0, :2] == 100)


def test_empty_cloud_has_no_grid():
    assert build_traversability_grid(np.empty((0, 3))) is None


def test_isolated_wall_returns_are_not_promoted_to_ground():
    floor = []
    for x in (0.05, 0.25, 0.45):
        floor += cell_points(x, 0.05, [0.0, 0.01, 0.02])
    wall_only = cell_points(0.85, 0.05, [0.8, 1.0, 1.2])
    grid = build_traversability_grid(np.array(floor + wall_only), TraversabilityConfig(
        resolution=0.2, max_roughness=1.0, max_slope_deg=89.0,
        min_points_per_cell=2, padding_cells=0))
    assert np.isfinite(grid.ground[0, 0])
    assert np.isnan(grid.ground[0, 4])
    assert grid.cost[0, 4] == 100


def test_vertical_obstacle_does_not_inflate_ground_roughness():
    points = np.array(cell_points(0.05, 0.05, [0.0, 0.01, 0.3, 0.6]))
    grid = build_traversability_grid(points, TraversabilityConfig(
        resolution=0.2, max_roughness=0.06, max_slope_deg=89.0,
        min_points_per_cell=2, padding_cells=0))
    assert grid.roughness[0, 0] < 0.01
    assert grid.cost[0, 0] == 100


def test_obstacle_inflation_uses_metric_radius():
    points = np.array(
        cell_points(0.05, 0.05, [0.0, 0.0]) +
        cell_points(0.25, 0.05, [0.0, 0.3]) +
        cell_points(0.45, 0.05, [0.0, 0.0]))
    grid = build_traversability_grid(points, TraversabilityConfig(
        resolution=0.2, max_roughness=1.0, max_slope_deg=89.0,
        min_points_per_cell=2, padding_cells=0, inflation_radius=0.2))
    assert np.all(grid.cost[0, :3] == 100)
