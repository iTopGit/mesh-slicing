# WarmupBatching.md — parallel edge measurement (spatial reuse)

> **STATUS: REJECTED (tested 2026-07-24).** Batching does **not** solve the warm-up
> problems and makes clean data **worse**. Measured, not guessed — see "Tested result"
> below. Design kept for the record. The real problem (clean data at scale) is still
> **open** — see "Open problem". Do not implement this as-is.
>
> Archived here 2026-09-16 (was `tempPLAN.md` in the project root). `harness/warmup.cc`
> sweeps serially and carries the conclusion in a comment; this file keeps the numbers.

Draft design to fix warm-up scaling (runtime + failure rate) on a **single channel**, without sampling and without changing the contention model. Short version — flesh out into PLAN.md once agreed.

## Problem it solves

Current sweep measures **one edge at a time** → 178 serial windows → ~33 min at 10×10, and time grows with edge count (infeasible at 32×32). We go serial only to keep each measurement clean (measuring two edges at once would let their test traffic contend).

## Core idea

Two edges that are **far enough apart cannot hear each other**, so they can be measured **at the same time** without contaminating each other's reading. Group the edges into a few "far-apart batches" and measure each batch in parallel. On a regular grid the number of batches is roughly **constant** (independent of grid size) → sweep time stops growing with the network.

## Design

1. **Calibrate the safe distance once.** Measure two edges at increasing separation; the smallest separation at which each still reads the same as it does *alone* is the interference range `R_int` (carrier-sense range, wider than the ~50 m decode range). Add margin. One-time step, not per-run.

2. **List which links fight.** Two links *fight* (can't be measured at the same time) if they **share a router** or their endpoints sit **within `R_int`** of each other — a shared router means one radio can't test two links at once; within `R_int` means the two radios hear each other over the air. Build this who-fights-with-who list across all links.

3. **Split links into conflict-free batches (greedy).** Walk the links; drop each into the first batch whose links it doesn't fight; open a new batch only when forced. Each batch = links that can all be measured at once. On a regular grid the batch count stays roughly constant → a handful of batches, not ~178.

4. **Measure one batch at a time, all its edges simultaneously.** Reuse the existing per-edge `prime → L → R` windows unchanged — just fire a whole batch's edges together instead of one edge. Total windows drop from ~178 to (#batches).

5. **Keep the existing backstops.** Retry (re-measure failed edges, now per batch) and median-correction stay as-is — they still catch the residual collisions between test traffic and mesh maintenance, of which there are now far fewer because there are far fewer windows.

## What changes vs stays

- **Changes:** only the *scheduling* — `BeginWarmup` groups edges into batches and launches a batch at once, instead of looping edges serially.
- **Reused unchanged:** `ScheduleEdgeWindows` (per-edge prime/L/R), `WarmupRetryPass`, `MedianCorrectOutliers`, the whole contention/fitness model. Single channel untouched → thesis model untouched.
- **Subsumes `ReorderEdges`:** the batch rule (no shared router, beyond `R_int`) already forbids two windows sharing a node, so the old reorder pass is no longer separately needed.

## Why it solves the problems

- **Runtime (problem 1):** windows ≈ #batches (roughly constant) instead of #edges (linear) → sweep time ~flat across grid sizes. 32×32 becomes tractable.
- **Failure rate (problem 2):** far fewer windows means far fewer chances to collide with maintenance traffic; retry/median handle the rest.
- **Model integrity:** spatial reuse is a *measurement-only* trick — it does not touch how slices contend on the shared channel, so the core claim is unaffected.

> The two claims above (runtime, failure rate) were the *prediction*. Both failed on test — see next.

## Tested result (why rejected)

Ran the batched code (`GroupEdges` + batched `BeginWarmup`) against serial (forced by `rInt = 100000`, one edge per batch), same machinery so only batching differs. `R_int = 75 m`.

**Speed — barely helps, and only at large grids:**

| grid | serial | batch 75 m | speedup |
|---|---|---|---|
| 5×5 (40 edges) | 3m43s | 3m30s | ~none |
| 10×10 (178 edges) | ~33 min | 21m4s | ~1.6× |

Reason: ns-3 wall-clock scales with **packets simulated**, not virtual time. Batching keeps the packet total the same and just concentrates it into brutal concurrent floods (up to 18 saturating edges at once at 10×10). The only real saving is the idle-mesh background (nodes × virtual seconds), which is small until the grid is large.

**Data quality — worse everywhere (the disqualifier):**

| grid | serial outliers | batch outliers | serial fails | batch fails |
|---|---|---|---|---|
| 5×5 | 5 / 40 | **14 / 40** | 0 | 2 |
| 10×10 | ~38 / 178 | **60 / 178** | 7 | 11 |

Batching adds inter-edge interference *on top of* the existing noise → a **third of the 10×10 table** replaced by fabricated medians. Even the "clean" `R_int = 75 m` contaminates, because batches pack up to 18 edges near the interference boundary.

**Verdict:** ~1.6× faster for ~1.6× more contaminated data. For a table feeding NSGA-II, that trade is unacceptable. Rejected.

## Open problem — still unsolved

The core aim — **clean per-link data at scale** — is not solved by anything tried yet:

- Serial + Minstrel: ~12–21 % of edges are outliers (Minstrel's *designed* rate sampling caught mid-swing) + a few permanent losses (timing collisions surviving all retries). Median-correction papers over them but *fabricates* those values.
- Batching: makes both worse (above).
- Longer window / more retries, and ConstantRate (fixed MCS, no sampling): being tested — expected to *reduce* outliers, but even if ConstantRate works, the result is still not fully clean (permanent losses remain, and a fixed MCS biases capacity toward one guessed rate). **A proper fix is future work**, not settled here.

Candidate directions to explore later: deterministic rate + learned per-link MCS; a network-wide settle phase before measuring; smarter retry timing (avoid the maintenance beacon phase); or accepting a small, *bounded* median-correction rate as good enough once it's a handful of edges, not a third.

## Code sketch

Not final — shows the shape. Two new helpers (`Distance`, `Conflict`, `GroupEdges`) plus a rewritten `BeginWarmup`. `ScheduleEdgeWindows` gains an explicit start-time argument so a whole batch can share one start instead of reading `session->nextTime`.

```cpp
// Straight-line distance between two nodes, from their grid positions.
static double
Distance(const MeshTopology& mesh, uint32_t u, uint32_t v)
{
    Vector pu = mesh.nodes.Get(u)->GetObject<MobilityModel>()->GetPosition();
    Vector pv = mesh.nodes.Get(v)->GetObject<MobilityModel>()->GetPosition();
    return CalculateDistance(pu, pv);
}

// Two links fight if they share a router or any endpoint pair is within R_int.
static bool
Conflict(const Edge& e1, const Edge& e2, const MeshTopology& mesh, double rInt)
{
    if (e1.first == e2.first || e1.first == e2.second ||
        e1.second == e2.first || e1.second == e2.second)
        return true;
    uint32_t a[2] = {e1.first, e1.second};
    uint32_t b[2] = {e2.first, e2.second};
    for (uint32_t u : a)
        for (uint32_t v : b)
            if (Distance(mesh, u, v) < rInt)
                return true;
    return false;
}

// Greedy: drop each link into the first batch it doesn't fight; else open a new one.
static std::vector<std::vector<Edge>>
GroupEdges(const std::vector<Edge>& edges, const MeshTopology& mesh, double rInt)
{
    std::vector<std::vector<Edge>> batches;
    for (const Edge& e : edges)
    {
        bool placed = false;
        for (std::vector<Edge>& batch : batches)
        {
            bool fights = false;
            for (const Edge& m : batch)
                if (Conflict(e, m, mesh, rInt)) { fights = true; break; }
            if (!fights) { batch.push_back(e); placed = true; break; }
        }
        if (!placed) batches.push_back({e});
    }
    return batches;
}

void
BeginWarmup(WarmupSession* session)
{
    session->edges = BuildEdgeList(session->mesh);
    std::vector<std::vector<Edge>> batches =
        GroupEdges(session->edges, session->mesh, session->rInt);

    double t = Simulator::Now().GetSeconds();
    const double batchSpan = primeSec + 2 * session->windowSec + guard; // one window's length
    for (std::vector<Edge>& batch : batches)
    {
        for (const Edge& e : batch)
            ScheduleEdgeWindows(session, e, t); // every edge in the batch shares start t
        t += batchSpan;                         // next batch begins after this one ends
    }
    // Retry pass fires after the last batch; it re-batches only the failed edges.
    Simulator::Schedule(Seconds(t - Simulator::Now().GetSeconds()), &WarmupRetryPass, session);
}
```

Notes:
- `rInt` is a new `WarmupSession` field, set from the one-time calibration (step 1).
- `WarmupRetryPass` changes the same way — collect the failed edges, `GroupEdges` them, replay batch-by-batch. Same backstop, now batched.
- `MedianCorrectOutliers`, `ExtractLinkTable`, `WriteLinkTable` unchanged.

## Risks / how to check

- **Wrong `R_int` = silent contamination** (two edges measured too close together read low, and it looks like real data, not a catchable failure). Mitigate: measure `R_int` empirically with margin, and **spot-check** a few edges measured both *in-batch* and *isolated* — if they match, the batching is clean; if in-batch reads worse, spacing is too tight.
- **Edge-of-grid batches** may be smaller/irregular; fine — just means a couple of extra batches, still constant-ish.
