# Reference-data fidelity and the VTRN dependency

An honest look at the question: *how much can we trust bounding GPS-denied drift
by matching the drone's optical feed to a pre-loaded georeferenced basemap, over
the whole world, and in conflict zones where that basemap goes stale?*

Short answer: **less than the idealised model implied, and least exactly where
this drone operates.** This note explains the dependency, what the new
[`correction.py`](src/droneflight/correction.py) module models, what it does not,
and what to do about it. It is the flight-side companion to the planner's
[`../droneplanner/SHORTFALLS.md`](../droneplanner/SHORTFALLS.md).

## The dependency chain

Mission success is a chain, and every link leans on the one below it:

```
reach the endpoint  ←  hold the concealment corridor  ←  bound the drift
                    ←  periodic absolute fixes (VTRN)   ←  a trustworthy basemap
```

Technology-Selection §2.1 is explicit: well-tuned VIO/LIO drifts **0.5–3 % of
distance**, i.e. **25–150 m over 5 km** — "far too much to reliably reach a fixed
endpoint or hold a planned concealment corridor." The recommended fix, and "the
single most important architectural decision in the whole system," is an
**absolute-correction layer**: visual terrain-relative navigation (VTRN) matching
the nadir camera to a georeferenced orthophoto/DSM. §9 states the residual risk
in one line: **"VTRN is only as good as its basemap."** So the whole chain
terminates on one assumption — a good basemap, available when and where a fix is
needed. That assumption is what the user challenged, and it is worth challenging.

## Two independent ways the basemap fails

The old `nav.py` VTRN model was an **idealised reset**: every fixed interval the
error teleports to a small constant residual. That silently assumes a fix is
(1) available everywhere, (2) always accepted, and (3) always correct. Reality
breaks along two independent axes:

### Axis 1 — availability (matchable texture)

A nadir matcher needs distinctive, geo-registerable features. It gets them over
textured open terrain, field edges, roads, and buildings. It gets **few** over:

- **uniform forest canopy** — self-similar, low-contrast, and it *moves* (wind,
  growth, season), so features don't repeat between basemap and flight;
- water, fresh snow, sand, plowed monoculture, deep shadow.

The sting is structural, not incidental: **forest is where this drone
deliberately hides.** The planner *rewards* FLYABLE forest as concealment
(`droneplanner` `fields/composite.py`). So the concealment objective and the
navigation objective are in direct tension — **the drone is most lost exactly
where it is most concealed.** The deeper it hides, the blinder its absolute nav.
Over canopy the fix rate collapses and drift is free to grow between the sparse
open patches.

### Axis 2 — correctness (staleness / change)

An *available* match can still be **wrong** if the reference no longer matches the
world. Over a conflict zone the basemap goes stale fast — new craters, rubble,
destroyed or new construction, defoliation, shifting front lines, seasonal
canopy. A stale scene can produce a **confident-looking but wrong** match: the
matcher locks onto features that have moved or onto a plausible false
correspondence, and reports a fix that is off by tens or hundreds of metres.

This is worse than no fix. A missing fix leaves you dead-reckoning with known,
growing uncertainty. A *wrong* fix injected as truth **corrupts** the estimate
and can be worse than the drift it was meant to correct — and it does so with
false confidence. The only defence is to **not believe** a match that disagrees
too much with where dead-reckoning says you are (an innovation gate). But that
defence has a cruel property: it weakens as drift grows. After a long fix-starved
stretch (see Axis 1), your uncertainty is large, the gate opens wide, and a
blunder sails through. **The fix you most need — after crossing a wide forest
belt — is the one you are least able to validate.**

## What the module now models

[`correction.py`](src/droneflight/correction.py) replaces the magic reset with a
real, robust measurement:

- **`ReferenceMap`** — the basemap's quality as a *spatially varying* field:
  `matchability` (Axis 1), `staleness` (Axis 2), and `accuracy_m` (the residual a
  clean fix leaves). `UniformReference` is the optimistic control;
  `PatchyReference.along_route(...)` paints forest belts (low matchability) and
  stale zones (high staleness) along the route.
- **`OpticalCorrector`** — carries the estimator's self-assessed variance (grows
  with dead-reckoning distance, shrinks on a fix). Each attempt: check there is
  enough texture to match at all → form a measurement (noisier as features thin;
  a blunder offset with probability = staleness) → reject on a **confidence
  floor** (filters low texture) → reject on an **innovation gate** (filters
  blunders) → fuse survivors with a scalar Kalman update so error *shrinks toward
  the fix accuracy* rather than teleporting.

Crucially, the reported confidence uses **feature strength only** — the drone
cannot tell from a good-looking match that the map is stale. So staleness is
caught *only* by the gate, which is what makes the "large-drift blunder" failure
mode real rather than hand-waved.

Run `droneflight nav --plan … --forest-frac F --stale-frac S` to see it. On a
real ~3.6 km planned route (2 % drift):

| Correction model | Max error | Fix availability | Corridor |
|---|---|---|---|
| VIO only (no fixes) | ~71 m | — | ✗ |
| Idealised VTRN (old reset) | ~12 m | (assumed 100 %) | ✓ |
| Optical, 40 % forest / 10 % stale | ~8 m | 55 % | ✓ |
| Optical, 70 % forest / 30 % stale (conflict zone) | ~16 m | **24 %** | **✗** |
| Optical, 40 % stale, **gate disabled** | ~64 m | 72 % | ✗ (blunders fused) |

The idealised model says "12 m, all fine." The realistic model shows the corridor
is held over a good basemap, **lost over a forest-heavy conflict-zone basemap**,
and that the innovation gate is the load-bearing defence against stale-map
corruption — remove it and error blows up even though *more* fixes are "accepted."

## What this is NOT (limits — read before trusting it)

- It is an **error/fidelity emulator, not a matcher.** There is no real feature
  extraction, no real orthophoto/DSM, no cross-view geolocalisation. The fidelity
  fields are **synthetic** — plausible structure and magnitudes, not measured.
- Real numbers require the §6.3 bench: replay recorded nadir imagery over real
  reference maps, per-region, per-season, and report match rate + fix accuracy vs
  a truth reference. Until then, treat `matchability`/`staleness` as *knobs to
  reason with*, not calibrated coverage.
- The corrector is 2-D isotropic and single-hypothesis; a shipped stack fuses
  VTRN in the autopilot EKF (§5.2) with full covariance and would add multi-view
  consistency, not a single innovation gate.

## What to do about it (mitigations)

1. **Co-optimise the route for navigability, not just concealment.** Feed a
   basemap **matchability / currency** layer into the planner's cost surface as a
   *navigation-risk* term, so the route doesn't dive so deep into a fix-desert
   that it goes blind. This is the plan-side fix for the concealment↔nav tension,
   and it belongs in `droneplanner` (see its SHORTFALLS cross-cutting note).
2. **Don't be single-string on ortho-match.** Add complementary absolute
   references that fail differently: terrain-contour / skyline matching against
   the DSM, magnetometer/celestial where usable, radio time-of-arrival. Fuse
   several weak absolute cues rather than betting the mission on one.
3. **Extend the dead-reckoning that must bridge fix deserts.** A tactical-grade
   IMU (§5.1; note export control, §8) and LIO in low-texture foliage buy longer
   coasts between fixes, so wider forest belts are survivable.
4. **Treat basemap currency as a mission gate.** Decide the reference-map
   acquisition/refresh strategy per AO up front (§9); a mission over a volatile
   area with a months-old basemap should be flagged pre-flight, not discovered in
   the air.
5. **Detect change online and down-weight it.** Compare live imagery structure to
   the reference; where they disagree at scale, lower trust (raise the effective
   staleness) instead of gambling on a gate that widens with drift.

## Cross-reference: the planner has the same disease

This note is about *flight-time* fixes, but the **planner builds its corridor
from the same class of geospatial reference** — DTM/DSM, lidar, and (eventually)
land-cover imagery. Its fidelity is just as uneven globally and just as
perishable in a conflict zone. `droneplanner/SHORTFALLS.md` already flags "lidar
currency / DSM-vs-canopy coupling is not validated" (M2 🟡) and "no real-AO
validation" (🔴). The corridor you plan and the fixes you fly are **two
consumers of one shaky input**: a recent, high-resolution, georeferenced picture
of the operating area — which cannot be assumed to exist everywhere, and least of
all where it changes every week.
