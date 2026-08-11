#!/usr/bin/env python3
"""Convert a downsampled PCD map into a PCT planner tomogram pickle.

This is a CPU-only, numpy port of the tomography pipeline in
src/PCT_planner/tomography/scripts/tomogram.py + kernels.py. The original
implementation requires a CUDA GPU through CuPy; this script runs anywhere
with numpy (+ open3d to read the PCD, numba optional for faster inflation).

The exported pickle has exactly the schema expected by
src/PCT_planner/planner/scripts/planner_wrapper.py::

    {
        "data": float16 array of shape (5, n_slice, dim_x, dim_y),
        "resolution": grid size in metres,
        "center": [cx, cy],
        "slice_h0": height of the first slice,
        "slice_dh": vertical slice thickness in metres,
    }

Layers of ``data`` (see TomogramPlanner.loadTomogram):

    0: inflated traversability cost
    1: traversability gradient along x
    2: traversability gradient along y
    3: elevation (ground height) map
    4: clearance (ceiling height) map

Example::

    python3 pcd_to_tomogram.py --pcd /path/to/map.pcd

Default output is written next to the PCD (``<pcd_stem>.pickle``).
"""

import argparse
import os
import pickle
import sys
import time

import numpy as np

# Scene defaults copied from tomography/config/scene.py (SceneTrav) so the
# generated tomogram matches what the planner expects.
DEFAULT_TRAV = {
    "kernel_size": 7,
    "interval_min": 0.50,
    "interval_free": 0.65,
    "slope_max": 0.40,
    "step_max": 0.17,
    "standable_ratio": 0.20,
    "cost_barrier": 50.0,
    "safe_margin": 0.4,
    "inflation": 0.2,
}

try:  # optional: much faster inflation on CPU
    from numba import njit

    @njit(cache=True)
    def _inflate_slice(trav_cost, score_table, half_k):
        n_row, n_col = trav_cost.shape
        out = np.zeros_like(trav_cost)
        for i in range(n_row):
            for j in range(n_col):
                best = 0.0
                for dy in range(-half_k, half_k + 1):
                    ii = i + dy
                    if ii < 0 or ii >= n_row:
                        continue
                    for dx in range(-half_k, half_k + 1):
                        jj = j + dx
                        if jj < 0 or jj >= n_col:
                            continue
                        weight = score_table[dy + half_k, dx + half_k]
                        if weight <= 0.0:
                            continue
                        value = trav_cost[ii, jj] * weight
                        if value > best:
                            best = value
                out[i, j] = best
        return out

    HAS_NUMBA = True
except ImportError:
    HAS_NUMBA = False


def log(message):
    print(f"[INFO] {message}", flush=True)


def read_points(pcd_path):
    try:
        import open3d as o3d
    except ImportError as error:
        raise RuntimeError(
            "open3d is required to read PCD files; install it with "
            "'pip install open3d'"
        ) from error
    pcd = o3d.io.read_point_cloud(pcd_path)
    points = np.asarray(pcd.points, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] < 3:
        raise ValueError(f"unexpected point cloud shape: {points.shape}")
    points = points[:, :3].copy()
    points = points[~np.isnan(points).any(axis=1)]
    if points.shape[0] == 0:
        raise ValueError("point cloud is empty after removing NaN points")
    return points


def build_layers(points, center, resolution, dim_x, dim_y, n_slice,
                 slice_h0, slice_dh):
    """Replicate the CUDA tomography kernel with numpy.

    For every point and every slice:
      pz <= slice_h  -> layers_g = max(layers_g, pz)
      pz >  slice_h  -> layers_c = min(layers_c, pz)
    """
    px = points[:, 0]
    py = points[:, 1]
    pz = points[:, 2]

    # Same mapping as the CUDA kernel: round((p - center) / res) + n_row / 2.
    ix = (
        np.floor((px - center[0]) / resolution + 0.5).astype(np.int64)
        + dim_x // 2
    )
    iy = (
        np.floor((py - center[1]) / resolution + 0.5).astype(np.int64)
        + dim_y // 2
    )
    valid = (ix >= 0) & (ix < dim_x) & (iy >= 0) & (iy < dim_y)
    if not valid.any():
        raise ValueError("no point falls inside the computed map bounds")

    ix = ix[valid]
    iy = iy[valid]
    pz = pz[valid]
    lin = ix * dim_y + iy  # row-major cell index inside one slice

    # k = first slice whose bottom is >= pz. Points with k <= s write
    # layers_g[s], points with k > s write layers_c[s].
    k = np.ceil((pz - slice_h0) / slice_dh).astype(np.int64)
    np.clip(k, 0, n_slice, out=k)

    # Sort once by cell index; per-slice subsets keep cells grouped so
    # reduceat can aggregate max/min z per cell without re-sorting.
    order = np.argsort(lin, kind="stable")
    lin_s = lin[order]
    k_s = k[order]
    pz_s = pz[order]

    layers_g = np.full((n_slice, dim_x, dim_y), -1e6, dtype=np.float32)
    layers_c = np.full((n_slice, dim_x, dim_y), 1e6, dtype=np.float32)
    g_flat = layers_g.reshape(n_slice, -1)
    c_flat = layers_c.reshape(n_slice, -1)

    for s in range(n_slice):
        below = k_s <= s
        if below.any():
            cells = lin_s[below]
            heights = pz_s[below]
            starts = np.concatenate(
                ([0], np.flatnonzero(cells[1:] != cells[:-1]) + 1))
            g_flat[s, cells[starts]] = np.maximum.reduceat(heights, starts)

        above = k_s > s
        if above.any():
            cells = lin_s[above]
            heights = pz_s[above]
            starts = np.concatenate(
                ([0], np.flatnonzero(cells[1:] != cells[:-1]) + 1))
            c_flat[s, cells[starts]] = np.minimum.reduceat(heights, starts)

    return layers_g, layers_c


def box_sum(values, half_k):
    """Sum of a (2*half_k+1)^2 window around every cell (zero padding)."""
    pad = half_k
    padded = np.pad(values, pad, mode="constant", constant_values=0.0)
    height, width = padded.shape
    cum = np.zeros((height + 1, width + 1), dtype=np.float32)
    cum[1:, 1:] = np.cumsum(np.cumsum(padded, axis=0), axis=1)
    size = 2 * half_k + 1
    return (
        cum[size:, size:]
        - cum[:-size, size:]
        - cum[size:, :-size]
        + cum[:-size, :-size]
    )


def traversability_slice(layers_g_s, layers_c_s, trav):
    """Traversability cost for one slice (port of the CUDA trav kernel)."""
    g = layers_g_s
    c = layers_c_s
    interval = c - g

    # Gradient magnitude fields (port of the CUDA computation in point2map).
    diff_x_sq = np.maximum(
        (g[1:-1, :] - g[:-2, :]) ** 2,
        (g[1:-1, :] - g[2:, :]) ** 2,
    )
    diff_y_sq = np.maximum(
        (g[:, 1:-1] - g[:, :-2]) ** 2,
        (g[:, 1:-1] - g[:, 2:]) ** 2,
    )
    grad_mag_sq = np.zeros_like(g)
    grad_mag_max = np.zeros_like(g)
    grad_mag_sq[1:-1, 1:-1] = diff_x_sq[:, 1:-1] + diff_y_sq[1:-1, :]
    grad_mag_max[1:-1, 1:-1] = np.maximum(
        diff_x_sq[:, 1:-1], diff_y_sq[1:-1, :])

    barrier = trav["cost_barrier"]
    interval_min = trav["interval_min"]
    interval_free = trav["interval_free"]
    step_stand_sq = trav["step_stand_sq"]
    step_cross_sq = trav["step_cross_sq"]
    half_k = trav["half_kernel_size"]
    standable_th = trav["standable_th"]

    cost = np.zeros_like(interval)

    low = interval < interval_min
    cost[low] = barrier
    normal = ~low
    cost[normal] += np.maximum(0.0, 20.0 * (interval_free - interval[normal]))

    stand = grad_mag_sq <= step_stand_sq
    cost[stand] += 15.0 * grad_mag_sq[stand] / step_stand_sq

    steep = ~stand
    crossable = steep & (grad_mag_max <= step_cross_sq)
    standable_count = box_sum(
        (grad_mag_sq < step_stand_sq).astype(np.float32), half_k)
    too_sparse = crossable & (standable_count < standable_th)
    cost[too_sparse] = barrier
    dense = crossable & ~too_sparse
    cost[dense] += 20.0 * grad_mag_max[dense] / step_cross_sq
    cost[steep & ~crossable] = barrier
    return cost


def inflate_slice(trav_cost, score_table, half_k):
    """Inflate one traversability slice (port of the CUDA inflation kernel)."""
    if HAS_NUMBA:
        return _inflate_slice(trav_cost, score_table, half_k)

    n_row, n_col = trav_cost.shape
    padded = np.pad(trav_cost, half_k, mode="constant", constant_values=0.0)
    out = np.zeros_like(trav_cost)
    for dy in range(-half_k, half_k + 1):
        for dx in range(-half_k, half_k + 1):
            weight = score_table[dy + half_k, dx + half_k]
            if weight <= 0.0:
                continue
            window = padded[
                half_k + dy: half_k + dy + n_row,
                half_k + dx: half_k + dx + n_col,
            ]
            np.maximum(out, window * weight, out=out)
    return out


def build_inflated(layers_g, layers_c, trav):
    """Traversability + inflation for every slice."""
    n_slice = layers_g.shape[0]
    resolution = trav["resolution"]

    half_inf = int((trav["safe_margin"] + trav["inflation"]) / resolution)
    size = 2 * half_inf + 1
    yy, xx = np.mgrid[0:size, 0:size]
    dist = np.sqrt(
        (resolution * (yy - half_inf)) ** 2
        + (resolution * (xx - half_inf)) ** 2
    )
    score_table = np.clip(
        1.0 - (dist - trav["inflation"])
        / (trav["safe_margin"] + resolution),
        0.0, 1.0,
    ).astype(np.float32)

    inflated = np.empty_like(layers_g)
    t0 = time.time()
    for s in range(n_slice):
        cost = traversability_slice(layers_g[s], layers_c[s], trav)
        inflated[s] = inflate_slice(cost, score_table, half_inf)
        if (s + 1) % 10 == 0 or s + 1 == n_slice:
            log(
                f"traversability+inflation: slice {s + 1}/{n_slice} "
                f"({time.time() - t0:.1f}s)"
            )
    return inflated


def simplify_layers(layers_g, inflated, cost_barrier):
    """Keep only slices where the scene is unique (port of layer simplification)."""
    n_slice = layers_g.shape[0]
    idx_simp = [0]
    if n_slice > 1:
        l_idx, m_idx = 0, 1
        diff_h = layers_g[1:] - layers_g[:-1]
        while m_idx < n_slice - 2:
            mask_l_g = layers_g[m_idx] - layers_g[l_idx] > 0
            mask_l_t = inflated[l_idx] > inflated[m_idx]
            mask_u_g = diff_h[m_idx] > 0
            mask_t = inflated[m_idx] < cost_barrier
            unique = (mask_l_g | mask_l_t) & mask_u_g & mask_t
            if unique.any():
                idx_simp.append(m_idx)
                l_idx = m_idx
            m_idx += 1
        idx_simp.append(m_idx)
    return np.asarray(idx_simp, dtype=np.int64)


def simplify_layers_floors(layers_g, inflated, cost_barrier, slice_dh):
    """Select one representative slice per traversable floor level.

    The upstream CUDA simplification keeps slices where elevation is rising
    inside traversable cells, which suits ramped scenes but collapses flat
    floor-plate buildings into only the lowest and highest slices. This mode
    instead keeps a new slice whenever the median elevation of the traversable
    region changes by at least 0.75 * slice_dh, i.e. one slice per floor level.
    """
    n_slice = layers_g.shape[0]
    idx_simp = []
    last_elev = None
    for s in range(n_slice):
        traversable = (layers_g[s] > -1e6) & (inflated[s] < cost_barrier)
        if not traversable.any():
            continue
        elev = float(np.median(layers_g[s][traversable]))
        if last_elev is None or abs(elev - last_elev) >= 0.75 * slice_dh:
            idx_simp.append(s)
            last_elev = elev
    if not idx_simp:
        idx_simp = [0]
    return np.asarray(idx_simp, dtype=np.int64)


def build_tomogram(points, resolution, ground_h, slice_dh, trav):
    points_min = points.min(axis=0).copy()
    points_max = points.max(axis=0)
    if ground_h is None:
        ground_h = float(points_min[2])
    points_min[2] = ground_h

    dim_x = int(np.ceil((points_max[0] - points_min[0]) / resolution)) + 4
    dim_y = int(np.ceil((points_max[1] - points_min[1]) / resolution)) + 4
    n_slice = max(1, int(np.ceil((points_max[2] - points_min[2]) / slice_dh)))
    center = ((points_max[:2] + points_min[:2]) / 2).astype(np.float32)
    slice_h0 = float(points_min[2] + slice_dh)

    log(
        f"map: center=({center[0]:.3f}, {center[1]:.3f}) "
        f"grid={dim_x}x{dim_y} slices={n_slice} "
        f"resolution={resolution}m slice_dh={slice_dh}m ground_h={ground_h:.3f}m"
    )
    data_mb = 5 * n_slice * dim_x * dim_y * 2 / 2 ** 20
    log(f"estimated output data size: ~{data_mb:.0f} MB (float16)")

    t0 = time.time()
    layers_g, layers_c = build_layers(
        points, center, resolution, dim_x, dim_y, n_slice, slice_h0, slice_dh)
    log(f"tomography done in {time.time() - t0:.1f}s")

    trav = dict(trav)
    trav["resolution"] = resolution
    trav["half_kernel_size"] = trav["kernel_size"] // 2
    step_stand = 1.2 * resolution * np.tan(trav["slope_max"])
    trav["step_stand_sq"] = float(step_stand ** 2)
    trav["step_cross_sq"] = float(trav["step_max"] ** 2)
    trav["standable_th"] = int(
        trav["standable_ratio"] * (2 * trav["half_kernel_size"] + 1) ** 2
    ) - 1

    inflated = build_inflated(layers_g, layers_c, trav)
    log(f"traversability+inflation total: {time.time() - t0:.1f}s")

    if trav["simplify"] == "floors":
        idx_simp = simplify_layers_floors(
            layers_g, inflated, trav["cost_barrier"], slice_dh)
    else:
        idx_simp = simplify_layers(layers_g, inflated, trav["cost_barrier"])
    log(f"layer simplification: {n_slice} -> {idx_simp.size} slices")
    for s in idx_simp:
        valid = layers_g[s] > -1e6
        traversable = valid & (inflated[s] < trav["cost_barrier"])
        log(
            f"  kept slice {s}: "
            f"elevation_valid={valid.mean() * 100:.1f}% "
            f"traversable={traversable.sum() / max(valid.sum(), 1) * 100:.1f}% "
            f"of valid cells"
        )

    layers_t = inflated[idx_simp]
    layers_g_s = layers_g[idx_simp]
    layers_c_s = layers_c[idx_simp]

    trav_gx = np.zeros_like(layers_g_s)
    trav_gx[:, 1:-1, :] = layers_t[:, 2:, :] - layers_t[:, :-2, :]
    trav_gy = np.zeros_like(layers_g_s)
    trav_gy[:, :, 1:-1] = layers_t[:, :, 2:] - layers_t[:, :, :-2]

    layers_g_out = np.where(layers_g_s > -1e6, layers_g_s, np.nan).astype(
        np.float16)
    layers_c_out = np.where(layers_c_s < 1e6, layers_c_s, np.nan).astype(
        np.float16)

    data = np.stack([
        layers_t.astype(np.float16),
        trav_gx.astype(np.float16),
        trav_gy.astype(np.float16),
        layers_g_out,
        layers_c_out,
    ])
    return {
        "data": data,
        "resolution": float(resolution),
        "center": center,
        "slice_h0": slice_h0,
        "slice_dh": float(slice_dh),
    }


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Convert a PCD map into a PCT planner tomogram pickle.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--pcd", required=True, help="input PCD file (downsampled map)")
    parser.add_argument(
        "--output", default="",
        help="output pickle path (default: <pcd_dir>/<pcd_stem>.pickle)")
    parser.add_argument(
        "--resolution", type=float, default=0.05,
        help="grid resolution in metres")
    parser.add_argument(
        "--ground-h", type=float, default=None,
        help="z of the ground layer in metres (default: min z of the cloud)")
    parser.add_argument(
        "--slice-dh", type=float, default=0.5,
        help="vertical slice thickness in metres")
    parser.add_argument(
        "--simplify", choices=("floors", "original"), default="floors",
        help="layer selection: 'floors' keeps one representative slice per "
             "traversable floor level (recommended for multi-floor buildings); "
             "'original' replicates the upstream CUDA heuristic")
    parser.add_argument(
        "--crop-percentile", type=float, default=0.0, metavar="P",
        help="crop sparse x/y outliers: keep the [P, 100-P] percentile box "
             "of the cloud (e.g. 1 drops the outer 1%% of points in x and y); "
             "0 disables cropping")
    for name, default in DEFAULT_TRAV.items():
        parser.add_argument(
            f"--{name.replace('_', '-')}", type=float, default=default,
            help=f"traversability parameter (default: {default})")
    return parser.parse_args(argv)


def main(argv=None):
    log(f"python={sys.version.split()[0]}")
    log(f"numpy={np.__version__}")
    args = parse_args(argv)
    if not os.path.isfile(args.pcd):
        sys.exit(f"PCD file does not exist: {args.pcd}")

    trav = {name: getattr(args, name) for name in DEFAULT_TRAV}
    trav["kernel_size"] = int(trav["kernel_size"])
    trav["simplify"] = args.simplify

    output = args.output or (
        os.path.splitext(args.pcd)[0] + ".pickle")
    log(f"reading PCD: {args.pcd}")
    points = read_points(args.pcd)
    log(
        f"points={points.shape[0]} "
        f"x=[{points[:, 0].min():.3f}, {points[:, 0].max():.3f}] "
        f"y=[{points[:, 1].min():.3f}, {points[:, 1].max():.3f}] "
        f"z=[{points[:, 2].min():.3f}, {points[:, 2].max():.3f}]"
    )
    if args.crop_percentile > 0.0:
        lo = args.crop_percentile
        hi = 100.0 - args.crop_percentile
        x_lo, x_hi = np.percentile(points[:, 0], [lo, hi])
        y_lo, y_hi = np.percentile(points[:, 1], [lo, hi])
        mask = (
            (points[:, 0] >= x_lo) & (points[:, 0] <= x_hi)
            & (points[:, 1] >= y_lo) & (points[:, 1] <= y_hi)
        )
        log(
            f"crop percentile {args.crop_percentile}: "
            f"x=[{x_lo:.3f}, {x_hi:.3f}] y=[{y_lo:.3f}, {y_hi:.3f}] "
            f"kept {mask.sum()}/{points.shape[0]} points"
        )
        points = points[mask]
        if points.shape[0] == 0:
            sys.exit("crop removed all points; lower --crop-percentile")

    t0 = time.time()
    data_dict = build_tomogram(
        points, args.resolution, args.ground_h, args.slice_dh, trav)
    log(f"tomogram built in {time.time() - t0:.1f}s")

    t0 = time.time()
    with open(output, "wb") as handle:
        pickle.dump(data_dict, handle, protocol=4)
    size_mb = os.path.getsize(output) / 2 ** 20
    log(f"saved pickle ({size_mb:.1f} MB): {output} in {time.time() - t0:.1f}s")

    # Self check: reload and print the schema so it can be compared with
    # planner_wrapper.loadTomogram expectations.
    with open(output, "rb") as handle:
        check = pickle.load(handle)
    log(
        f"verify: data.shape={check['data'].shape} dtype={check['data'].dtype} "
        f"resolution={check['resolution']} center={check['center']} "
        f"slice_h0={check['slice_h0']} slice_dh={check['slice_dh']}"
    )
    return output


if __name__ == "__main__":
    main()
