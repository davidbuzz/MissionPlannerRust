# ADR 0001 — The map viewport can live inside gpui, if geometry is decimated

**Status:** accepted, 2026-09-23. Settles the Deliverable 7/Deliverable 8 gate in `PLAN.md` §2.4 and §9.1.
**Evidence:** `crates/mp-gui/src/mapview.rs`, run on this box (Intel UHD + NVIDIA Quadro T2000,
X11, gpui pinned at zed `62e5991`, wgpu backend). Screenshot: `docs/progress/d08-map-spike.png`.

## The question

`gpui::Primitive` is a closed eight-variant enum and the wgpu renderer's `Surfaces` batch is an
empty match arm, so a consumer cannot add a custom GPU pass. Does that prevent building the map,
which is the largest single piece of Mission Planner's UI?

## What was measured

A viewport painting, through `canvas()`, what a real map paints: a tile grid, a 100,000-point
survey track as stroked paths, and 2,000 markers as quads. Paint cost measured inside the paint
callback, exponentially smoothed, first three frames discarded as warm-up.

| Configuration | Paint cost | Effective rate |
|---|---:|---|
| 100,000 points, tessellated every frame | 84.8 ms | 12 fps |
| 100,000 points, tessellation cached | 60.7 ms | 16 fps |
| **Decimated to 2 points/pixel (1,451 points), cached** | **2.7 ms** | **~370 fps** |

Phase breakdown at full resolution: tiles 0.04 ms, path clone 27.6 ms, path submit 30.3 ms,
markers 1.8 ms. Decimated: tiles 0.03 ms, clone 0.39 ms, submit 0.43 ms, **markers 1.80 ms**.

## Findings

1. **gpui can host the map with no fork.** Tiles are quads or images, tracks are stroked paths,
   markers are quads — all GPU-rendered by primitives that already exist.

2. **A path holds at most 65,535 vertices.** gpui tessellates into
   `VertexBuffers<lyon::math::Point, u16>`, so a stroked polyline caps out around 20–30k points.
   `PathBuilder::build()` returns an error rather than truncating — and the idiomatic
   `if let Ok(path) = builder.build()` silently drops it. A 100k-point track simply does not
   appear, with no warning. Tracks must be chunked, and build failures must be counted, not
   ignored.

3. **The path API is submit-every-frame.** Caching tessellation helps (85 ms → 61 ms) but does not
   solve it, because a cached `Path` must still be cloned into the scene each paint at a cost
   proportional to vertex count. There is no retained geometry handle.

4. **Therefore decimation is a prerequisite, not an optimisation.** Submitting no more geometry
   than the screen can resolve takes the frame from 60.7 ms to 2.7 ms — 22× — and looks identical,
   because the discarded points were landing inside pixels already covered.

5. **After decimation the markers dominate**, at 1.8 ms for 2,000 quads. Marker culling and
   batching is the next thing to measure, before anything else in the map is tuned.

## Consequences

- Deliverable 8's map is built around a decimation pyramid and view culling from the start. Douglas-Peucker
  or a pre-built pyramid replaces the stride sampling used in this spike, so shape is preserved
  rather than sampled blindly.
- Every `PathBuilder::build()` call site counts failures and surfaces them. A silent `Ok` check is
  treated as a defect in review.
- The 120 fps map target (8.33 ms) is achievable on this hardware with room to spare.
- **Not yet answered: Windows.** This was measured on the Linux wgpu backend. gpui uses Direct3D 11
  on Windows, a different renderer, and Mission Planner's userbase is overwhelmingly Windows. The
  same spike must run there, and on a degraded target (RDP or a VM), before this ADR is relied on
  for scheduling. Until then the abandon conditions in `PLAN.md` §2.4 remain live.
