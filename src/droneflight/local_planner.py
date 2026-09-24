"""Onboard stem mapping + receding-horizon planning — avoidance without a prior map.

WHY THIS EXISTS (2026-09-23)
Two results bracket the forest problem, and this module sits between them:

* **Reactive avoidance is dead in flight** (TEST-PLAN N2): a per-step "steer to
  the free bearing nearest the goal" policy flew into 34-41 trunks, and every
  proposed fix measured null or worse. It sees the tree, turns, and hits it.
* **Routing between mapped stems works** (2026-09-14): the same forest, flown in
  plain mission mode along a path planned *through the gaps*, gave **0 strikes,
  330/330**. But that needed a prior stem map — a lidar survey — and the user has
  since ruled that out ("assume no lidar unless absolutely necessary"), on top of
  the earlier rule that a pre-flight map is never ground truth in a changed
  battlespace.

So this does the stem routing **onboard, from the aircraft's own depth sensor**:
every trunk the sensor has seen is remembered, and a short path through the gaps
is planned against that memory and re-planned as more comes into view. What made
the prior-map flight work — committing to a path with clearance margin, instead of
picking a bearing each step — is kept; the survey is not needed.

WHY IT IS NOT JUST A BETTER REACTIVE LAYER
Three differences, each aimed at something N2 measured:

1. **Memory.** A 90 deg sensor forgets a trunk the moment it leaves the FOV, and a
   reactive dodge turns the sensor away from the trunk it is dodging. Here a trunk
   seen once stays an obstacle until it is behind the aircraft.
2. **A path, not a bearing.** A bearing says nothing about the next trunk but one;
   the plan is checked end to end against everything known, with a clearance cost
   that prefers the middle of a gap rather than its edge (N2's defect was parking
   the heading on the edge of a shadow).
3. **Speed from the path.** The commanded speed is limited by the path's curvature
   and by how tight the next gap is, so the aircraft slows *for the turn it is
   about to make* rather than reacting to the one it is in.

🔑 THE FRAME PROPERTY THAT MAKES IT GPS-DENIED-SAFE
Observations are placed in the map using the aircraft's **believed** pose, and the
plan is followed using the same believed pose. A navigation error ``e`` therefore
shifts every remembered trunk by ``e`` *and* the aircraft by ``e`` — the relative
geometry, which is all that obstacle clearance depends on, is unchanged. So
avoidance here is **immune to slowly-varying absolute navigation error**, unlike
prior-map routing, which needs the map and the pose to agree to better than the
clearance (the 2026-09-15 localiser finding: ≲0.5 m). What absolute error still
costs is *where on the route* the aircraft is — corridor keeping — which is the
navigation stack's job, not this module's.
⚠️ The immunity is to error that changes slowly compared with how long a trunk
stays in memory. A *jump* (a VTRN fix snapping the estimate by metres) moves the
aircraft in the believed frame without moving the remembered trunks, so the
memory is **cleared** on a jump larger than ``jump_reset_m`` rather than trusted.

⚠️ WHAT IS STILL ORACLE
The scan it consumes comes from `perception.sense` — perfect ranges to known
cylinders, no noise, no dropout, no false returns. That isolates *"can the
aircraft plan through trees it can see"* from *"can a camera see them"* (Track B).
Occlusion, however, **is** real: rays stop at the first trunk, so the memory only
ever contains what line of sight allowed.

Pure Python like the rest of the flight stack (no numpy), so it runs in the
10 Hz offboard loop and in the kinematic harness unchanged.
"""

from __future__ import annotations

import heapq
import itertools
import math
from dataclasses import dataclass, field

from droneflight.perception import DepthScan
from droneflight.px4_profile import OFFBOARD_DELAY_S, OFFBOARD_TAU_S

Point = tuple[float, float]

_SQRT2 = math.sqrt(2.0)
_NEIGHBOURS: tuple[tuple[int, int, float], ...] = (
    (1, 0, 1.0),
    (-1, 0, 1.0),
    (0, 1, 1.0),
    (0, -1, 1.0),
    (1, 1, _SQRT2),
    (1, -1, _SQRT2),
    (-1, 1, _SQRT2),
    (-1, -1, _SQRT2),
)


@dataclass(frozen=True)
class LocalPlannerConfig:
    """Airframe, clearance and planning parameters."""

    drone_radius_m: float = 0.45
    """Half-width including props — the same 0.45 m `AvoidanceConfig` uses."""

    obstacle_radius_m: float = 0.15
    """Assumed trunk radius. A depth return lies on the trunk's *near surface*; it is
    pushed this far further along the ray so the remembered point sits near the
    axis, and the obstacle is inflated by it. Under-estimating it costs clearance;
    a fat trunk (radius r) is also seen by more rays, which partly compensates."""

    hard_margin_m: float = 0.35
    """Air gap below which a cell is nearly forbidden (heavily penalised, not
    infinite, so the aircraft can always plan its way *out* of a squeeze)."""

    preferred_gap_m: float = 1.5
    """Air gap the clearance cost pushes toward. Planning against a stem map
    (2026-09-14) showed PX4 tracking eats ~0.7 m of planned clearance, so the
    cost keeps pulling well past the hard margin rather than grazing it."""

    clearance_weight: float = 3.0
    """How strongly the path prefers the middle of a gap. 0 = shortest path."""

    cell_m: float = 0.3
    horizon_m: float = 22.0
    """How far ahead along the route the local goal sits. Must be inside sensor
    range, or the plan runs into space it has never looked at."""

    window_margin_m: float = 6.0
    """Extra room around the start->goal box, so the plan can swing round a gap."""

    replan_period_s: float = 0.4
    max_expansions: int = 40_000
    memory_radius_m: float = 45.0
    """Remembered points further than this from the aircraft are forgotten."""

    correction_threshold_m: float = 0.0
    """Per-step displacement that velocity does not explain, above which the
    memory is shifted by it. **0** — every unexplained displacement is by
    definition an estimate correction. It was 0.05 m (chosen to sit above a
    0.01 m velocity-noise floor), but the EV bridge bleeds a fix in at about
    ``0.03 x fix`` per step at 3 m/s, so a fix under ~1.7 m was never detected and
    the remembered trunks ended up off by the whole of it. Over six 400/ha
    GPS-denied harness seeds: 0.05 -> 2 grazes, 0 -> 1; 200/ha clean 4/4."""

    jump_reset_m: float = 1.5
    """If the believed position jumps by more than this in one step (a navigation
    fix), clear the memory: every remembered point was placed with the old
    error, and is now in the wrong place relative to the aircraft."""

    cruise_speed_ms: float = 6.0
    min_speed_ms: float = 0.8
    max_lateral_accel_ms2: float = 2.0
    """Speed is capped so the upcoming path's curvature needs no more than this.
    Kept under PX4's `MPC_ACC_HOR` (3.0) so tracking keeps up with the turn."""

    max_brake_accel_ms2: float = 2.0
    tight_gap_speed_ms: float = 2.0
    """Speed through a gap at exactly the hard margin, rising linearly to cruise
    at the preferred gap."""

    pursuit_m: float = 2.5
    """Pure-pursuit lookahead along the local path — the ceiling; see ``pursuit_s``."""

    pursuit_s: float = 1.2
    min_pursuit_m: float = 0.8
    """Lookahead scales with speed (``pursuit_s`` of travel, floored here). A fixed
    2.5 m chord cuts inside every bend by the same amount at any speed, so near a
    trunk no speed was predicted clear and the aircraft stopped for good (800/ha:
    stationary for 700 s with a path 0.18 m clear). Slow now also means tight."""

    speed_rise_ms2: float = 1.5
    """How fast the commanded speed may rise. Falls are unlimited."""

    min_corridor_half_width_m: float = 4.0
    """Floor on the lateral freedom taken from the plan. The planner grants 0 m
    where the route hugs the edge of its cover, and a trunk standing there would
    then be impassable; `droneplanner`'s stem router floors the same thing at the
    same 4 m (``StemPolicy.min_half_width_m``)."""

    corridor_weight: float = 2.0
    """Cost per metre outside the corridor, when the strict pass (which forbids
    it) finds nothing — so a drifted aircraft still plans its way back in."""

    commit_s: float = 0.4
    """How much of the current velocity a replan must keep (seconds of travel)."""

    max_commit_m: float = 2.5

    max_accel_ms2: float = 3.0
    """PX4 ``MPC_ACC_HOR`` — the envelope the feasibility check flies candidates in."""

    response_delay_s: float = OFFBOARD_DELAY_S
    response_tau_s: float = OFFBOARD_TAU_S
    """How the airframe answers a velocity setpoint — dead time, then a first-order
    lag, saturating at ``max_accel_ms2``. Measured from the SITL records
    (``px4_profile.OFFBOARD_*``); the feasibility check flies candidates in it."""

    predict_s: float = 2.5
    predict_margin_m: float = 0.30
    """Smallest predicted air gap a (path, speed) candidate may have. It must cover
    the prediction's own error: the measured response model is 0.18 m out at p95
    one second ahead, and at the old 0.15 a GPS-denied SITL flight accepted
    predictions of 0.22-0.44 m on the way into a trunk it then hit."""

    seen_cell_m: float = 0.6
    """Resolution of the observed-free-space grid behind the sight-distance limit."""

    sight_buffer_m: float = 1.5
    """Distance kept in hand when braking to the edge of observed space."""

    escape_m: float = 1.5
    """Within this distance of the aircraft, cells inside the hard margin are
    allowed (at a heavy cost) so the plan can lead *out* of a squeeze. Beyond it
    they are forbidden outright, so the plan never chooses to thread one."""

    @property
    def contact_m(self) -> float:
        """Centre-to-axis distance at which the airframe touches a trunk."""
        return self.obstacle_radius_m + self.drone_radius_m

    @property
    def hard_m(self) -> float:
        return self.contact_m + self.hard_margin_m

    @property
    def preferred_m(self) -> float:
        return self.contact_m + self.preferred_gap_m


class ObstacleMemory:
    """Trunk evidence in the believed frame, deduplicated on a fine grid.

    Keeping points, not fitted cylinders, keeps it honest about what a depth
    sensor gives: returns on surfaces. A trunk seen from several sides simply
    accumulates several points, and the inflation covers the rest.
    """

    def __init__(self, dedupe_m: float = 0.2) -> None:
        self._dedupe = dedupe_m
        self._points: dict[tuple[int, int], Point] = {}

    def __len__(self) -> int:
        return len(self._points)

    def clear(self) -> None:
        self._points.clear()

    def points(self) -> list[Point]:
        return list(self._points.values())

    def add(self, x: float, y: float) -> bool:
        """Remember a point; returns True if it was new evidence."""
        key = (math.floor(x / self._dedupe), math.floor(y / self._dedupe))
        if key in self._points:
            return False
        self._points[key] = (x, y)
        return True

    def add_scan(
        self, scan: DepthScan, x: float, y: float, heading: float, push_m: float
    ) -> list[Point]:
        """Add every ray that hit something, placed from pose ``(x, y, heading)``.

        Returns the new points (so the caller can check them against its path).
        """
        added: list[Point] = []
        for rel, rng in zip(scan.bearings_rel, scan.ranges, strict=True):
            if rng >= scan.max_range:
                continue
            b = heading + rel
            d = rng + push_m
            px, py = x + d * math.cos(b), y + d * math.sin(b)
            if self.add(px, py):
                added.append((px, py))
        return added

    def forget_beyond(self, x: float, y: float, radius: float) -> None:
        r2 = radius * radius
        stale = [k for k, (px, py) in self._points.items() if (px - x) ** 2 + (py - y) ** 2 > r2]
        for k in stale:
            del self._points[k]


class SeenSpace:
    """Where the sensor has actually looked, in the believed frame.

    🔑 **Occlusion is what made the first version strike trunks it knew about.**
    Offline on the dense manifest, all 3 strikes were trunks already in memory,
    hit at 4-4.7 m/s while the planner was commanding under 2: each was hidden
    behind a nearer trunk until the aircraft was too close to stop. Remembered
    obstacles say where *not* to go; they say nothing about whether the space
    beyond them is empty or merely unseen. A ray that returned at range ``r``
    proves only the first ``r`` metres, so those cells are marked here, and the
    speed is capped to what can stop before the path leaves observed space.
    """

    def __init__(self, cell_m: float) -> None:
        self._cell = cell_m
        self._cells: set[tuple[int, int]] = set()
        # Accumulated navigation correction. Cells are keyed in the continuous
        # (odometry) frame, i.e. believed position minus this offset, so a
        # correction moves the whole grid in O(1) instead of invalidating it.
        self._ox = 0.0
        self._oy = 0.0

    def __len__(self) -> int:
        return len(self._cells)

    def clear(self) -> None:
        self._cells.clear()

    def _key(self, x: float, y: float) -> tuple[int, int]:
        return math.floor((x - self._ox) / self._cell), math.floor((y - self._oy) / self._cell)

    def shift(self, dx: float, dy: float) -> None:
        """Move everything observed by a navigation correction ``(dx, dy)``."""
        self._ox += dx
        self._oy += dy

    def add_scan(self, scan: DepthScan, x: float, y: float, heading: float) -> None:
        step = 0.5 * self._cell
        cells = self._cells
        for rel, rng in zip(scan.bearings_rel, scan.ranges, strict=True):
            b = heading + rel
            ux, uy = math.cos(b) * step, math.sin(b) * step
            n = int(rng / step)
            px, py = x, y
            for _ in range(n):
                px += ux
                py += uy
                cells.add(self._key(px, py))

    def add_disc(self, x: float, y: float, radius: float) -> None:
        """Mark a disc as observed — the aircraft's own position proves it is free."""
        ci, cj = self._key(x, y)
        r = math.ceil(radius / self._cell)
        for di in range(-r, r + 1):
            for dj in range(-r, r + 1):
                if di * di + dj * dj <= r * r:
                    self._cells.add((ci + di, cj + dj))

    def seen(self, x: float, y: float, halo_m: float = 0.0) -> bool:
        """Has the sensor observed the point, and everything within ``halo_m`` of it?"""
        ci, cj = self._key(x, y)
        r = math.ceil(halo_m / self._cell)
        return all(
            (ci + di, cj + dj) in self._cells for di in range(-r, r + 1) for dj in range(-r, r + 1)
        )

    def forget_beyond(self, x: float, y: float, radius: float) -> None:
        ci, cj = self._key(x, y)
        r = radius / self._cell
        self._cells = {k for k in self._cells if (k[0] - ci) ** 2 + (k[1] - cj) ** 2 <= r * r}


@dataclass
class _Grid:
    ox: float
    oy: float
    cell: float
    nx: int
    ny: int
    dist: list[float]
    """Distance from each cell centre to the nearest remembered point (capped)."""

    excess: list[float] | None = None
    """Metres each cell lies OUTSIDE the plan's corridor (<= 0 inside), or None."""

    def index(self, x: float, y: float) -> tuple[int, int]:
        return int((x - self.ox) / self.cell), int((y - self.oy) / self.cell)

    def centre(self, i: int, j: int) -> Point:
        return self.ox + (i + 0.5) * self.cell, self.oy + (j + 0.5) * self.cell

    def inside(self, i: int, j: int) -> bool:
        return 0 <= i < self.nx and 0 <= j < self.ny


def _build_grid(points: list[Point], start: Point, goal: Point, cfg: LocalPlannerConfig) -> _Grid:
    m = cfg.window_margin_m
    ox = min(start[0], goal[0]) - m
    oy = min(start[1], goal[1]) - m
    nx = max(1, math.ceil((max(start[0], goal[0]) + m - ox) / cfg.cell_m))
    ny = max(1, math.ceil((max(start[1], goal[1]) + m - oy) / cfg.cell_m))
    cap = cfg.preferred_m
    dist = [cap] * (nx * ny)
    reach = math.ceil(cap / cfg.cell_m)
    c = cfg.cell_m
    for px, py in points:
        if not (ox - cap <= px <= ox + nx * c + cap and oy - cap <= py <= oy + ny * c + cap):
            continue
        ci, cj = int((px - ox) / c), int((py - oy) / c)
        for i in range(max(0, ci - reach), min(nx, ci + reach + 1)):
            dx = ox + (i + 0.5) * c - px
            row = i * ny
            for j in range(max(0, cj - reach), min(ny, cj + reach + 1)):
                dy = oy + (j + 0.5) * c - py
                d = math.sqrt(dx * dx + dy * dy)
                if d < dist[row + j]:
                    dist[row + j] = d
    return _Grid(ox=ox, oy=oy, cell=c, nx=nx, ny=ny, dist=dist)


Corridor = list[tuple[Point, float]]
"""Route vertices with the lateral half-width the plan grants at each."""


def _corridor_excess(grid: _Grid, corridor: Corridor, cfg: LocalPlannerConfig) -> list[float]:
    """How far each cell lies outside the corridor (negative inside).

    Width is interpolated along each segment and floored at
    ``min_corridor_half_width_m``; a cell takes the most permissive segment.
    """
    c = grid.cell
    x0, y0 = grid.ox, grid.oy
    x1, y1 = x0 + grid.nx * c, y0 + grid.ny * c
    floor = cfg.min_corridor_half_width_m
    segs = []
    for (a, wa), (b, wb) in itertools.pairwise(corridor):
        reach = max(wa, wb, floor)
        if (
            max(a[0], b[0]) + reach < x0
            or min(a[0], b[0]) - reach > x1
            or max(a[1], b[1]) + reach < y0
            or min(a[1], b[1]) - reach > y1
        ):
            continue
        segs.append((a, b, max(wa, floor), max(wb, floor)))
    if not segs:
        return [math.inf] * (grid.nx * grid.ny)
    out = [math.inf] * (grid.nx * grid.ny)
    for i in range(grid.nx):
        px = x0 + (i + 0.5) * c
        row = i * grid.ny
        for j in range(grid.ny):
            py = y0 + (j + 0.5) * c
            best = math.inf
            for a, b, wa, wb in segs:
                abx, aby = b[0] - a[0], b[1] - a[1]
                den = abx * abx + aby * aby
                t = 0.0 if den == 0.0 else ((px - a[0]) * abx + (py - a[1]) * aby) / den
                t = 0.0 if t < 0.0 else 1.0 if t > 1.0 else t
                d = math.hypot(px - a[0] - abx * t, py - a[1] - aby * t) - (wa + (wb - wa) * t)
                if d < best:
                    best = d
            out[row + j] = best
    return out


def _cell_cost(d: float, cfg: LocalPlannerConfig, *, strict: bool) -> float:
    """Multiplier on step length for a cell at distance ``d`` from the nearest point.

    ``inf`` inside contact (half a cell of grid slack is added so a cell whose
    centre clears but whose corner does not is still refused). Inside the hard
    margin: ``inf`` when ``strict``, otherwise a heavy penalty.
    """
    if d < cfg.contact_m + 0.5 * cfg.cell_m:
        return math.inf
    if d < cfg.hard_m:
        return math.inf if strict else 25.0
    if d < cfg.preferred_m:
        f = (cfg.preferred_m - d) / (cfg.preferred_m - cfg.hard_m)
        return 1.0 + cfg.clearance_weight * f * f
    return 1.0


def _free_goal(grid: _Grid, goal: Point, cfg: LocalPlannerConfig) -> tuple[int, int] | None:
    """The goal cell, or the nearest acceptable one if a trunk sits on it."""
    gi, gj = grid.index(*goal)
    gi, gj = min(max(gi, 0), grid.nx - 1), min(max(gj, 0), grid.ny - 1)
    for ring in range(0, int(4.0 / cfg.cell_m) + 1):
        best: tuple[float, int, int] | None = None
        for i in range(gi - ring, gi + ring + 1):
            for j in range(gj - ring, gj + ring + 1):
                if max(abs(i - gi), abs(j - gj)) != ring or not grid.inside(i, j):
                    continue
                d = grid.dist[i * grid.ny + j]
                if d >= cfg.hard_m and (best is None or -d < best[0]):
                    best = (-d, i, j)
        if best is not None:
            return best[1], best[2]
    return None


def plan_local(
    points: list[Point],
    start: Point,
    goal: Point,
    cfg: LocalPlannerConfig,
    corridor: Corridor | None = None,
) -> list[Point] | None:
    """A* from ``start`` to ``goal`` through the remembered trunks.

    Tries **strict** first — no cell inside the hard margin except near the
    aircraft, so the plan cannot choose to thread a gap it cannot afford — and
    only if that finds nothing, **relaxed**, where such cells are merely
    expensive. The relaxed pass exists so a vehicle that has ended up in a squeeze
    can still plan its way out rather than freezing.

    ``corridor`` (the plan's per-vertex lateral freedom) is a hard bound in the
    strict pass and a per-metre cost in the relaxed one. 🔑 Without it, flown at
    400 stems/ha, the aircraft spent 4.7 % of the forest outside the corridor the
    plan granted — by up to 13 m, where the route hugs the edge of its cover —
    i.e. it left the concealment the route was scored on to find wider gaps.

    Returns world-frame points (start first), or ``None`` if no path exists
    within the window — which the caller must treat as "hold", never as "go
    straight".
    """
    grid = _build_grid(points, start, goal, cfg)
    if corridor:
        grid.excess = _corridor_excess(grid, corridor, cfg)
    return _astar(grid, start, goal, cfg, strict=True) or _astar(
        grid, start, goal, cfg, strict=False
    )


def _astar(
    grid: _Grid, start: Point, goal: Point, cfg: LocalPlannerConfig, *, strict: bool
) -> list[Point] | None:
    si, sj = grid.index(*start)
    si, sj = min(max(si, 0), grid.nx - 1), min(max(sj, 0), grid.ny - 1)
    goal_cell = _free_goal(grid, goal, cfg)
    if goal_cell is None:
        return None
    gi, gj = goal_cell
    ny = grid.ny
    start_k, goal_k = si * ny + sj, gi * ny + gj

    escape_cells2 = (cfg.escape_m / cfg.cell_m) ** 2

    def h(i: int, j: int) -> float:
        return math.hypot(i - gi, j - gj)

    g_cost: dict[int, float] = {start_k: 0.0}
    parent: dict[int, int] = {}
    frontier: list[tuple[float, int]] = [(h(si, sj), start_k)]
    closed: set[int] = set()
    expansions = 0
    while frontier:
        _, k = heapq.heappop(frontier)
        if k in closed:
            continue
        if k == goal_k:
            break
        closed.add(k)
        expansions += 1
        if expansions > cfg.max_expansions:
            return None
        i, j = divmod(k, ny)
        gk = g_cost[k]
        for di, dj, step in _NEIGHBOURS:
            ni, nj = i + di, j + dj
            if not (0 <= ni < grid.nx and 0 <= nj < ny):
                continue
            nk = ni * ny + nj
            if nk in closed:
                continue
            near_start = (ni - si) ** 2 + (nj - sj) ** 2 <= escape_cells2
            mult = _cell_cost(grid.dist[nk], cfg, strict=strict and not near_start)
            if mult == math.inf:
                continue
            if grid.excess is not None:
                out = grid.excess[nk]
                if out > 0.0:
                    if strict and not near_start:
                        continue
                    mult *= 1.0 + cfg.corridor_weight * out
            cand = gk + step * mult
            if cand < g_cost.get(nk, math.inf):
                g_cost[nk] = cand
                parent[nk] = k
                heapq.heappush(frontier, (cand + h(ni, nj), nk))
    else:
        return None
    if goal_k not in g_cost:
        return None
    cells = [goal_k]
    while cells[-1] != start_k:
        cells.append(parent[cells[-1]])
    cells.reverse()
    path = [start] + [grid.centre(*divmod(k, ny)) for k in cells[1:]]
    return _densify(_shortcut(path, grid, cfg), 0.5)


def _grid_dist(grid: _Grid, p: Point) -> float:
    i, j = grid.index(*p)
    if not grid.inside(i, j):
        return grid.dist[0] if grid.dist else math.inf
    return grid.dist[i * grid.ny + j]


def _shortcut(path: list[Point], grid: _Grid, cfg: LocalPlannerConfig) -> list[Point]:
    """String-pull the 8-connected path WITHOUT giving back any clearance.

    A grid path zig-zags in 45 deg steps. Flown raw, every zig reads as a turn:
    in the first offline run the commanded speed flickered between 2.8 and
    6.0 m/s on successive replans and the aircraft averaged its way into a trunk.
    A shortcut is accepted only if nowhere along it is closer to a remembered
    point than the stretch of path it replaces — the same rule the prior-map
    router (`droneplanner.planner.stem_route`) applies, for the same reason: an
    unbounded simplification cuts the corners that were keeping it off the trunks.
    """
    if len(path) <= 2:
        return path
    if grid.excess is not None:
        # Never shortcut across the outside of a bend of the corridor either.
        return _shortcut_bounded(path, grid, cfg)
    floor = cfg.contact_m + 0.5 * cfg.cell_m
    out = [path[0]]
    i = 0
    n = len(path)
    while i < n - 1:
        j = i + 1
        worst = _grid_dist(grid, path[j])
        while j + 1 < n:
            worst_next = min(worst, _grid_dist(grid, path[j + 1]))
            if not _segment_clear(path[i], path[j + 1], grid, max(floor, worst_next), cfg):
                break
            worst = worst_next
            j += 1
        out.append(path[j])
        i = j
    return out


def _excess_at(grid: _Grid, p: Point) -> float:
    if grid.excess is None:
        return -math.inf
    i, j = grid.index(*p)
    if not grid.inside(i, j):
        return math.inf
    return grid.excess[i * grid.ny + j]


def _shortcut_bounded(path: list[Point], grid: _Grid, cfg: LocalPlannerConfig) -> list[Point]:
    """:func:`_shortcut` that also keeps every sample inside the corridor (or no
    further outside than the stretch it replaces)."""
    floor = cfg.contact_m + 0.5 * cfg.cell_m
    out = [path[0]]
    i, n = 0, len(path)
    while i < n - 1:
        j = i + 1
        worst = _grid_dist(grid, path[j])
        worst_out = max(0.0, _excess_at(grid, path[j]))
        while j + 1 < n:
            wd = min(worst, _grid_dist(grid, path[j + 1]))
            wo = max(worst_out, _excess_at(grid, path[j + 1]))
            if not _segment_ok(path[i], path[j + 1], grid, max(floor, wd), wo, cfg):
                break
            worst, worst_out = wd, wo
            j += 1
        out.append(path[j])
        i = j
    return out


def _segment_ok(
    a: Point, b: Point, grid: _Grid, need: float, max_out: float, cfg: LocalPlannerConfig
) -> bool:
    steps = max(1, math.ceil(math.dist(a, b) / (0.5 * cfg.cell_m)))
    for k in range(1, steps):
        f = k / steps
        p = (a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f)
        if _grid_dist(grid, p) < need - 1e-9 or _excess_at(grid, p) > max_out + 1e-9:
            return False
    return True


def _segment_clear(a: Point, b: Point, grid: _Grid, need: float, cfg: LocalPlannerConfig) -> bool:
    length = math.dist(a, b)
    steps = max(1, math.ceil(length / (0.5 * cfg.cell_m)))
    for k in range(1, steps):
        f = k / steps
        if _grid_dist(grid, (a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f)) < need - 1e-9:
            return False
    return True


def _clamp(x: float, y: float, limit: float) -> tuple[float, float]:
    n = math.hypot(x, y)
    if n <= limit or n == 0.0:
        return x, y
    return x * limit / n, y * limit / n


def _segment_min_dist(points: list[Point], a: Point, b: Point) -> float:
    abx, aby = b[0] - a[0], b[1] - a[1]
    denom = abx * abx + aby * aby
    best = math.inf
    for px, py in points:
        t = 0.0 if denom == 0.0 else ((px - a[0]) * abx + (py - a[1]) * aby) / denom
        t = max(0.0, min(1.0, t))
        best = min(best, math.hypot(px - a[0] - abx * t, py - a[1] - aby * t))
    return best


def _densify(path: list[Point], step_m: float) -> list[Point]:
    out = [path[0]]
    for a, b in itertools.pairwise(path):
        n = max(1, math.ceil(math.dist(a, b) / step_m))
        dx, dy = b[0] - a[0], b[1] - a[1]
        out.extend((a[0] + dx * k / n, a[1] + dy * k / n) for k in range(1, n + 1))
    return out


def _path_length(path: list[Point]) -> float:
    return sum(math.dist(a, b) for a, b in itertools.pairwise(path))


def _nearest_index(path: list[Point], p: Point, hint: int) -> int:
    best, best_d = hint, math.inf
    for k in range(max(0, hint - 2), len(path)):
        d = math.dist(path[k], p)
        if d < best_d:
            best, best_d = k, d
    return best


def _point_along(path: list[Point], k: int, distance: float) -> tuple[Point, int]:
    remaining = distance
    while k < len(path) - 1:
        seg = math.dist(path[k], path[k + 1])
        if seg >= remaining and seg > 0.0:
            f = remaining / seg
            a, b = path[k], path[k + 1]
            return (a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f), k
        remaining -= seg
        k += 1
    return path[-1], len(path) - 1


def _heading(a: Point, b: Point) -> float:
    return math.atan2(b[1] - a[1], b[0] - a[0])


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class PlannerCommand:
    """What the local planner wants: a horizontal velocity in the believed frame."""

    vx: float
    vy: float
    speed_ms: float
    blocked: bool
    """No path to the local goal was found; the command is a hover."""

    min_gap_ahead_m: float
    """Smallest air gap (surface to airframe edge) along the next few metres of plan."""

    replanned: bool
    limit: str = ""
    """Which limit set the speed: ``cruise``, ``curvature``, ``alignment``,
    ``brake``, ``sight``, ``gap``, ``mission`` (the plan's leg speed), ``ramp``,
    ``feasibility`` or ``min``. Logged per step, so a slow flight says why."""


@dataclass
class LocalPlanner:
    """Stateful: memory, the current local path, and the replan clock."""

    config: LocalPlannerConfig = field(default_factory=LocalPlannerConfig)
    memory: ObstacleMemory = field(default_factory=ObstacleMemory)
    seen: SeenSpace | None = None
    path: list[Point] | None = None
    replans: int = 0
    failures: int = 0
    memory_resets: int = 0
    _last_plan_t: float = -math.inf
    _path_k: int = 0
    _last_pos: Point | None = None
    _observations: int = 0
    _last_t: float | None = None
    _last_vel: Point | None = None
    corrections: int = 0
    """Steps on which a navigation correction was detected and the map shifted."""

    corrected_m: float = 0.0
    _prev_path: list[Point] | None = None
    reverts: int = 0
    """Times the feasibility check kept the previous path over a fresh one."""

    infeasible: int = 0
    """Times no candidate cleared the predicted-pass margin (the least-bad was flown)."""
    _last_speed: float = 0.0
    _last_cmd_t: float | None = None
    _now: float = 0.0
    last_candidates: list[tuple[float, float]] = field(default_factory=list)
    """``(speed, predicted air gap)`` of every candidate the last feasibility check
    scored — logged by ``fly-local`` so a SITL contact can be replayed exactly."""
    _issued: list[tuple[float, float, float]] = field(default_factory=list)
    """``(t, vx, vy)`` setpoints already sent: the airframe is still acting on the
    last ``response_delay_s`` of them whatever is decided now."""

    def __post_init__(self) -> None:
        if self.seen is None:
            self.seen = SeenSpace(self.config.seen_cell_m)

    def observe(
        self,
        scan: DepthScan,
        x: float,
        y: float,
        heading: float,
        *,
        velocity: Point | None = None,
        t_s: float | None = None,
    ) -> list[Point]:
        """Fold a depth scan taken at believed pose ``(x, y, heading)`` into memory.

        🔑 **Navigation corrections move the map with them.** A VTRN fix changes
        the believed position without the aircraft moving; every remembered trunk
        was placed with the *old* error, so left alone it is now wrong relative to
        the aircraft by exactly the correction — and the EV bridge bleeds a fix in
        over ~10 m of travel, so no single step looks like a jump. With velocity
        and time, each step's position is predicted from the last; the part of the
        displacement velocity does not explain is a correction, and the memory is
        shifted by it. That keeps the map in the continuous (odometry) frame a
        real VIO stack would hold it in. Without velocity, a jump larger than
        ``jump_reset_m`` clears the memory instead.
        """
        cfg = self.config
        if self._last_pos is not None:
            lx, ly = self._last_pos
            if velocity is not None and t_s is not None and self._last_t is not None:
                dt = max(0.0, t_s - self._last_t)
                vx, vy = self._last_vel or velocity
                rx = x - (lx + 0.5 * (vx + velocity[0]) * dt)
                ry = y - (ly + 0.5 * (vy + velocity[1]) * dt)
                if math.hypot(rx, ry) > cfg.correction_threshold_m:
                    self._shift(rx, ry)
            elif math.dist((lx, ly), (x, y)) > cfg.jump_reset_m:
                self.memory.clear()
                self._seen().clear()
                self.memory_resets += 1
                self.path = None
        self._last_pos = (x, y)
        self._last_t = t_s
        self._last_vel = velocity
        added = self.memory.add_scan(scan, x, y, heading, cfg.obstacle_radius_m)
        self._seen().add_scan(scan, x, y, heading)
        # The sensor cone starts at the nose; the flanks of the next metre or two
        # are proven free by the aircraft being there, not by any ray.
        self._seen().add_disc(x, y, cfg.drone_radius_m + cfg.hard_margin_m + 0.5)
        self._observations += 1
        if self._observations % 20 == 0:
            self.memory.forget_beyond(x, y, cfg.memory_radius_m)
            self._seen().forget_beyond(x, y, cfg.memory_radius_m)
        return added

    def _shift(self, dx: float, dy: float) -> None:
        """Move everything remembered by a navigation correction ``(dx, dy)``."""
        pts = self.memory.points()
        self.memory.clear()
        for px, py in pts:
            self.memory.add(px + dx, py + dy)
        # 🔴 This used to CLEAR the observed-free grid. The EV bridge bleeds each
        # VTRN fix in over ~10 m, so during a bleed every step is a correction, the
        # sight limit saw nothing, and the first GPS-denied SITL run crawled
        # through OPEN ground at the 0.8 m/s floor.
        self._seen().shift(dx, dy)
        if self.path is not None:
            self.path = [(px + dx, py + dy) for px, py in self.path]
        if self._prev_path is not None:
            self._prev_path = [(px + dx, py + dy) for px, py in self._prev_path]
        self.corrections += 1
        self.corrected_m += math.hypot(dx, dy)

    def _seen(self) -> SeenSpace:
        assert self.seen is not None
        return self.seen

    def _path_threatened(self, added: list[Point]) -> bool:
        """Did new evidence land on the part of the path still to be flown?"""
        if self.path is None or not added:
            return False
        limit = self.config.hard_m
        ahead = self.path[self._path_k :]
        for px, py in added:
            for qx, qy in ahead:
                if (px - qx) ** 2 + (py - qy) ** 2 < limit * limit:
                    return True
        return False

    def command(
        self,
        x: float,
        y: float,
        goal: Point,
        t_s: float,
        added: list[Point] | None = None,
        velocity: Point | None = None,
        max_speed_ms: float | None = None,
        corridor: Corridor | None = None,
    ) -> PlannerCommand:
        """Plan (if due) and return the velocity to fly from believed position ``(x, y)``.

        ``velocity`` (believed frame, m/s) lets the plan respect momentum: see
        :meth:`_plan_committed`. Without it the planner assumes it can turn on the
        spot, which is the assumption that put the reactive layer into trees.

        ``max_speed_ms`` is the mission's own ceiling for this stretch — the
        planner's per-leg speed from the ``.plan`` (forest stem density), which
        the local planner may fly slower than but never faster.
        """
        cfg = self.config
        self._now = t_s
        cmd = self._command(x, y, goal, t_s, added, velocity, max_speed_ms, corridor)
        self._issued.append((t_s, cmd.vx, cmd.vy))
        horizon = t_s - cfg.response_delay_s - 0.5
        while self._issued and self._issued[0][0] < horizon:
            self._issued.pop(0)
        return cmd

    def _command(
        self,
        x: float,
        y: float,
        goal: Point,
        t_s: float,
        added: list[Point] | None,
        velocity: Point | None,
        max_speed_ms: float | None,
        corridor: Corridor | None,
    ) -> PlannerCommand:
        cfg = self.config
        replanned = False
        if (
            self.path is None
            or t_s - self._last_plan_t >= cfg.replan_period_s
            or self._path_threatened(added or [])
        ):
            self._prev_path = self.path
            self.path = self._plan_committed((x, y), goal, velocity, corridor)
            self._last_plan_t = t_s
            self._path_k = 0
            self.replans += 1
            replanned = True
            if self.path is None:
                self.failures += 1

        if self.path is None or len(self.path) < 2:
            return PlannerCommand(0.0, 0.0, 0.0, True, 0.0, replanned)

        path = self.path
        self._path_k = _nearest_index(path, (x, y), self._path_k)
        target, _ = _point_along(path, self._path_k, cfg.pursuit_m)
        dx, dy = target[0] - x, target[1] - y
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            return PlannerCommand(0.0, 0.0, 0.0, False, math.inf, replanned)

        gap = self._min_gap_ahead(path)
        limits = {
            "cruise": cfg.cruise_speed_ms,
            "curvature": self._curvature_speed(path),
            "alignment": self._alignment_speed((dx / norm, dy / norm), velocity),
            "brake": self._brake_speed(path),
            "sight": self._sight_speed(path),
            "gap": self._gap_speed(gap),
            "momentum": self._momentum_speed((x, y), velocity),
        }
        if max_speed_ms is not None:
            limits["mission"] = max_speed_ms
        limit = min(limits, key=lambda k: limits[k])
        speed = limits[limit]
        floor = min(cfg.min_speed_ms, max_speed_ms or cfg.min_speed_ms)
        if speed < floor:
            speed, limit = floor, f"{limit}+min"
        # Decreases pass at once; increases are rate-limited. Without this each
        # replan's slightly different path re-derived a different limit and the
        # command flickered 2.8 <-> 6.0 m/s, which the airframe averages to "fast".
        if self._last_cmd_t is not None:
            up = cfg.speed_rise_ms2 * max(0.0, t_s - self._last_cmd_t)
            if self._last_speed + up < speed:
                speed, limit = self._last_speed + up, "ramp"
        if velocity is not None:
            feasible = self._feasible_speed((x, y), velocity, speed)
            if feasible < speed - 1e-9:
                limit = "feasibility"
            speed = feasible
            path = self.path  # the feasibility check may have reverted to the old path
            if path is None or len(path) < 2:
                return PlannerCommand(0.0, 0.0, 0.0, True, 0.0, replanned, "blocked")
            self._path_k = _nearest_index(path, (x, y), 0)
            target, _ = _point_along(path, self._path_k, self._pursuit(speed))
            dx, dy = target[0] - x, target[1] - y
            norm = math.hypot(dx, dy) or 1.0
        self._last_speed, self._last_cmd_t = speed, t_s
        return PlannerCommand(
            dx / norm * speed, dy / norm * speed, speed, False, gap, replanned, limit
        )

    # -- dynamic feasibility --------------------------------------------------------

    def _pursuit(self, speed: float) -> float:
        cfg = self.config
        return min(cfg.pursuit_m, max(cfg.min_pursuit_m, cfg.pursuit_s * speed))

    def _issued_at(self, t: float, default: Point) -> Point:
        """The setpoint in force at time ``t`` (the last one sent at or before it)."""
        for ts, vx, vy in reversed(self._issued):
            if ts <= t:
                return (vx, vy)
        return default

    def _feasible_speed(self, pos: Point, velocity: Point, speed: float) -> float:
        """Pick a (path, speed) the airframe can actually fly without a close pass.

        🔴 Why: at 800 stems/ha and only 2 m/s the planner flew into a trunk it had
        mapped to within 0.2 m, on a plan claiming 0.8 m of air gap. A replan had
        turned the path 70 deg about a metre from the trunk, and a jerk-limited
        airframe needs ~0.75 s just to *build* lateral acceleration — so it went
        straight on. A path being clear is not the same as the aircraft being
        able to follow it. So each candidate is flown forward ``predict_s`` through
        the measured offboard response (dead time + lag, ``px4_profile``) and
        scored by the closest predicted pass. Candidates, in order of preference:
        the new path at the chosen speed, then slower; the previous path; a stop.
        """
        cfg = self.config
        near = [
            p for p in self.memory.points() if abs(p[0] - pos[0]) < 15 and abs(p[1] - pos[1]) < 15
        ]
        if not near:
            return speed
        candidates: list[tuple[list[Point] | None, float]] = []
        if self.path is not None:
            creep = min(speed, cfg.min_speed_ms * 0.5)
            candidates += [
                (self.path, speed),
                (self.path, speed * 0.6),
                (self.path, speed * 0.3),
                (self.path, creep),
            ]
        if self._prev_path is not None and self._prev_path is not self.path:
            candidates += [(self._prev_path, speed * 0.6), (self._prev_path, speed * 0.3)]
        candidates.append((self.path, 0.0))
        best: tuple[float, list[Point] | None, float] | None = None
        self.last_candidates = []
        for path, v in candidates:
            if path is None:
                continue
            gap = self._predict_gap(pos, velocity, path, v, near)
            if v > 0.0 and gap >= cfg.predict_margin_m:
                # 🔴 ...and after one replan period of it, a stop must still be
                # clean. Each replan in fix3 (SITL, 2026-09-24) nudged the path
                # 0.1-0.15 m closer to a trunk while the aircraft built momentum
                # toward it; every prediction passed, and by the first that did
                # not, stopping took more room than was left. This keeps the
                # aircraft out of states from which no command avoids contact.
                gap = min(
                    gap,
                    self._predict_gap(
                        pos, velocity, path, v, near, stop_after_s=cfg.replan_period_s
                    ),
                )
            self.last_candidates.append((v, gap))
            if gap >= cfg.predict_margin_m:
                if path is not self.path:
                    self.reverts += 1
                self.path = path
                return v
            if best is None or gap > best[0]:
                best = (gap, path, v)
        assert best is not None
        self.infeasible += 1
        self.path = best[1]
        return best[2]

    def _predict_gap(
        self,
        pos: Point,
        velocity: Point,
        path: list[Point],
        speed: float,
        near: list[Point],
        *,
        stop_after_s: float | None = None,
    ) -> float:
        """Closest predicted air gap flying ``path`` at ``speed`` for ``predict_s``
        — or, with ``stop_after_s``, flying it that long and then commanding a stop."""
        cfg = self.config
        dt = 0.1
        px, py = pos
        vx, vy = velocity
        # 🔴 The airframe is still flying the setpoints of the last
        # ``response_delay_s`` — whatever is decided now lands after them. The
        # jerk-limited model this replaced predicted SITL 1 s ahead to 0.40 m (p95);
        # a GPS-denied flight hit a trunk the prediction had passed as clear while
        # the aircraft carried 2 m/s east for 0.5 s after the command turned north.
        k = _nearest_index(path, pos, 0)
        worst = math.inf
        for step in range(int(cfg.predict_s / dt)):
            tt = step * dt
            if tt < cfg.response_delay_s:
                wx, wy = self._issued_at(self._now - cfg.response_delay_s + tt, velocity)
            elif stop_after_s is not None and tt >= cfg.response_delay_s + stop_after_s:
                wx, wy = 0.0, 0.0
            else:
                k = _nearest_index(path, (px, py), k)
                tgt, _ = _point_along(path, k, self._pursuit(speed))
                ex, ey = tgt[0] - px, tgt[1] - py
                en = math.hypot(ex, ey)
                wx, wy = (
                    (ex / en * speed, ey / en * speed) if en > 1e-6 and speed > 0 else (0.0, 0.0)
                )
            ax, ay = _clamp(
                (wx - vx) / cfg.response_tau_s, (wy - vy) / cfg.response_tau_s, cfg.max_accel_ms2
            )
            vx, vy = vx + ax * dt, vy + ay * dt
            px, py = px + vx * dt, py + vy * dt
            for qx, qy in near:
                d = math.hypot(px - qx, py - qy)
                if d < worst:
                    worst = d
        return worst - cfg.contact_m

    # -- speed limits -------------------------------------------------------------

    def _curvature_speed(self, path: list[Point]) -> float:
        """Brake *for* the turn ahead: ``v_now^2 <= v_turn(s)^2 + 2 a s`` for every s.

        🔴 The first version took ``radius = distance_to_turn / turn_angle``, which
        is not a radius at all: it let the aircraft carry cruise speed right up to a
        45 deg bend and then asked it to turn at the bend. Offline that put it into
        a known trunk at 4.7 m/s. Here each point on the upcoming path gets a local
        curvature (heading change over a 2 m chord), hence a cornering speed
        ``sqrt(a_lat / kappa)``, and the speed now is whatever still brakes down
        to that in the distance available.
        """
        cfg = self.config
        k = self._path_k
        pts = path[k : k + int(24.0 / 0.5)]
        if len(pts) < 5:
            return cfg.cruise_speed_ms
        cum = [0.0]
        for a, b in itertools.pairwise(pts):
            cum.append(cum[-1] + math.dist(a, b))
        v2 = cfg.cruise_speed_ms**2
        half = 2  # samples each side; ~1 m at 0.5 m spacing
        for i in range(half, len(pts) - half):
            h_in = _heading(pts[i - half], pts[i])
            h_out = _heading(pts[i], pts[i + half])
            chord = cum[i + half] - cum[i - half]
            if chord <= 0.0:
                continue
            kappa = abs(_wrap(h_out - h_in)) / chord
            if kappa < 1e-3:
                continue
            v_turn2 = cfg.max_lateral_accel_ms2 / kappa
            v2 = min(v2, v_turn2 + 2.0 * cfg.max_brake_accel_ms2 * cum[i])
        return math.sqrt(v2)

    def _brake_speed(self, path: list[Point]) -> float:
        """Never fly faster than can stop within the planned (known-free) path."""
        remaining = _path_length(path[self._path_k :])
        return math.sqrt(2.0 * self.config.max_brake_accel_ms2 * max(0.0, remaining - 1.0))

    def _plan_committed(
        self, pos: Point, goal: Point, velocity: Point | None, corridor: Corridor | None = None
    ) -> list[Point] | None:
        """Plan from where momentum will carry the aircraft, not from where it is.

        🔴 The strike this exists for: 4.5 m from a trunk at 5.5 m/s, a replan
        flipped the path from passing south of it to passing north. The new path
        was clear; the aircraft could not follow it and was carried into the trunk.
        A path that starts by turning hard is a path the airframe cannot fly, so
        the first ``commit_s`` of travel is fixed along the current velocity and
        the search starts from there — provided that stretch is itself clear. If it
        is not (something is dead ahead), plan from the current position and let
        the braking limits do their work.
        """
        cfg = self.config
        pts = self.memory.points()
        if velocity is not None:
            speed = math.hypot(*velocity)
            reach = min(speed * cfg.commit_s, cfg.max_commit_m)
            if speed > 0.5 and reach > cfg.cell_m:
                ahead = (pos[0] + velocity[0] / speed * reach, pos[1] + velocity[1] / speed * reach)
                if _segment_min_dist(pts, pos, ahead) >= cfg.hard_m:
                    tail = plan_local(pts, ahead, goal, cfg, corridor)
                    if tail is not None:
                        return _densify([pos, ahead], 0.5) + tail[1:]
        return plan_local(pts, pos, goal, cfg, corridor)

    def _momentum_speed(self, pos: Point, velocity: Point | None) -> float:
        """Never carry more speed at a trunk than the airframe can shed before it.

        Along the CURRENT velocity — where momentum takes the aircraft whatever is
        commanded — find the first remembered trunk the airframe would touch, and
        cap speed so the measured response (dead time, saturated braking, lag)
        stops it with ``hard_margin_m`` to spare (:meth:`_stop_distance`).
        Passing beside a trunk does not bind it; heading at one does.

        🔴 fix3 and fix4 (GPS-denied SITL, 2026-09-24) both touched the same trunk:
        2.9 m/s straight at it from 1.6 m, when stopping takes ~2 m, while each
        replan's path still went round it. The planner reversed the path at the
        last moment and the airframe, 0.7 s behind its setpoints, went on.
        """
        cfg = self.config
        if velocity is None:
            return cfg.cruise_speed_ms
        speed = math.hypot(*velocity)
        if speed < 0.3:
            return cfg.cruise_speed_ms
        ux, uy = velocity[0] / speed, velocity[1] / speed
        horizon = cfg.cruise_speed_ms * (cfg.response_delay_s + cfg.response_tau_s) + cfg.hard_m
        room = math.inf
        for px, py in self.memory.points():
            dx, dy = px - pos[0], py - pos[1]
            along = dx * ux + dy * uy
            if along <= 0.0 or along > horizon:
                continue
            lateral = abs(dx * uy - dy * ux)
            if lateral >= cfg.contact_m:
                continue
            room = min(room, along - math.sqrt(cfg.contact_m**2 - lateral**2))
        if room == math.inf:
            return cfg.cruise_speed_ms
        budget = room - cfg.hard_margin_m
        if budget <= 0.0:
            return 0.0
        # Largest v whose stop fits the budget (stop distance is monotone in v).
        lo, hi = 0.0, cfg.cruise_speed_ms
        if self._stop_distance(hi) <= budget:
            return hi
        for _ in range(30):
            mid = 0.5 * (lo + hi)
            lo, hi = (mid, hi) if self._stop_distance(mid) <= budget else (lo, mid)
        return lo

    def _stop_distance(self, v: float) -> float:
        """Distance to stop from ``v`` through the measured response: the dead time
        at full speed, then braking saturated at ``max_accel_ms2`` down to the
        speed where the lag stops saturating (``a * tau``), then the lag's tail.

        🔴 The first version used ``v * (delay + tau)`` — right at 3 m/s, but it
        ignores saturation and says 3.5 m from 5 m/s where the airframe needs
        ~6 m. The 5 m/s MVP flight (SITL, 2026-09-24) touched a trunk the limit
        had cleared from 3.8 m away.
        """
        cfg = self.config
        a, tau = cfg.max_accel_ms2, cfg.response_tau_s
        knee = a * tau
        tail = v * tau if v <= knee else (v * v - knee * knee) / (2.0 * a) + knee * tau
        return v * cfg.response_delay_s + tail

    def _alignment_speed(self, direction: Point, velocity: Point | None) -> float:
        """Slow while the velocity is swinging onto the path.

        A misalignment of ``theta`` recovered over the pursuit distance is a turn of
        curvature ``theta / pursuit_m``; fly it at the cornering speed for that."""
        cfg = self.config
        if velocity is None:
            return cfg.cruise_speed_ms
        speed = math.hypot(*velocity)
        if speed < 0.5:
            return cfg.cruise_speed_ms
        cos_t = (direction[0] * velocity[0] + direction[1] * velocity[1]) / speed
        theta = math.acos(max(-1.0, min(1.0, cos_t)))
        if theta < 1e-3:
            return cfg.cruise_speed_ms
        kappa = theta / max(cfg.pursuit_m, 1.0)
        return math.sqrt(cfg.max_lateral_accel_ms2 / kappa)

    def _sight_speed(self, path: list[Point]) -> float:
        """Never fly faster than can stop before the path leaves observed space.

        The swept width matters, not just the centreline: a trunk hidden half a
        metre to the side of the path is as dangerous as one on it."""
        cfg = self.config
        seen = self._seen()
        halo = cfg.drone_radius_m + cfg.hard_margin_m
        visible = 0.0
        k = self._path_k
        prev = path[k]
        while k < len(path):
            p = path[k]
            visible += math.dist(prev, p)
            prev = p
            if not seen.seen(p[0], p[1], halo):
                break
            k += 1
        else:
            return cfg.cruise_speed_ms
        room = max(0.0, visible - cfg.sight_buffer_m)
        return math.sqrt(2.0 * cfg.max_brake_accel_ms2 * room)

    def _min_gap_ahead(self, path: list[Point], look_m: float = 6.0) -> float:
        cfg = self.config
        pts = self.memory.points()
        if not pts:
            return math.inf
        best = math.inf
        travelled = 0.0
        k = self._path_k
        while k < len(path) and travelled <= look_m:
            qx, qy = path[k]
            for px, py in pts:
                best = min(best, math.hypot(px - qx, py - qy))
            if k + 1 < len(path):
                travelled += math.dist(path[k], path[k + 1])
            k += 1
        return best - cfg.contact_m

    def _gap_speed(self, gap: float) -> float:
        cfg = self.config
        if gap == math.inf:
            return cfg.cruise_speed_ms
        lo, hi = cfg.hard_margin_m, cfg.preferred_gap_m
        if gap <= lo:
            return cfg.tight_gap_speed_ms
        if gap >= hi:
            return cfg.cruise_speed_ms
        f = (gap - lo) / (hi - lo)
        return cfg.tight_gap_speed_ms + f * (cfg.cruise_speed_ms - cfg.tight_gap_speed_ms)
