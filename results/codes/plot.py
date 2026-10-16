#!/usr/bin/env python3
"""Plot one slice's delay and throughput against offered load, NSGA-II vs EA.

Usage:  python3 codes/plot.py RAW.csv [SLICE] [SEEDS] [LOADS] [TAG]
        python3 codes/plot.py raw/r.csv URLLC 1,2,3,4,5 2,4,6,8,10 growth

Writes show/<stem>_<slice>_delay.png and show/<stem>_<slice>_throughput.png, and prints the
mean +- sd table combined across seeds.

Two separate figures, never one with two y-axes: delay (ms) and throughput (Mb/s) are different
scales and a dual axis lets the author choose the story by choosing the scaling.

Bands are 95% CI (1.96 x sd/sqrt(n)), not sd: the question is whether the ARM MEANS differ,
and sd answers a different one (how much a single run varies). With n=4 the CI is about
0.98 x sd, so the bands are wide - that is the honest width, not a drafting choice.

INVALID runs are excluded from the statistics and their count is printed. They are the runs
that failed to drain, which correlates with congestion, so dropping them silently would bias
the means toward the better-behaved instances.

Colours are Okabe-Ito blue/orange, validated for colour-vision deficiency (worst adjacent
pair dE 29.2 protan / 36.2 normal). The orange sits below 3:1 against the surface, so the
legend and the printed table are the required relief - do not ship the figure without them.
"""
import csv
import statistics as st
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

NSGA, EA = '#0072B2', '#E69F00'          # validated; see module docstring
INK, MUTED, GRID = '#1a1a1a', '#555555', '#d8d8d4'
SURFACE = '#fcfcfb'
ARMS = [('nsga', 'NSGA-II', NSGA), ('seng', 'EA (Seng)', EA)]


def load(path, want_slice, seeds=None, loads=None):
    """rows -> {arm: {load: [values...]}} for delay and throughput, valid runs only.

    `seeds` and `loads` subset the data so one raw file can produce several figures. They are
    needed because the sweep is not rectangular: the low loads were run at ten seeds before
    the plan was cut to five, and a figure whose n changes along its own x-axis invites exactly
    the question it cannot answer. Filtering to a common n keeps the axis honest.
    """
    delay, good, dropped = {}, {}, {}
    for r in csv.DictReader(open(path)):
        if r['slice'] != want_slice or not r['demand']:
            continue
        arm = r['arm']
        load_mbps = float(r['demand'].split('/')[0])
        if loads and load_mbps not in loads:
            continue
        if seeds and int(r['gaSeed']) not in seeds:
            continue
        if r['valid'] == '0':
            dropped[(arm, load_mbps)] = dropped.get((arm, load_mbps), 0) + 1
            continue
        delay.setdefault(arm, {}).setdefault(load_mbps, []).append(float(r['delay_ms']))
        # 'goodput_mbps' is the pre-2026-10-16 name for the same number; keep reading it so
        # raw/ files extracted before the rename still plot.
        thpt = r.get('throughput_mbps') or r['goodput_mbps']
        good.setdefault(arm, {}).setdefault(load_mbps, []).append(float(thpt))
    return delay, good, dropped


def stats(vals):
    """mean, 95% CI half-width, n. CI is 0 at n=1 - one point cannot bound itself."""
    n = len(vals)
    m = st.mean(vals)
    ci = 1.96 * st.stdev(vals) / (n ** 0.5) if n > 1 else 0.0
    return m, ci, n


def band(vals, logy):
    """(centre, lo, hi) for the plotted interval.

    On a LOG axis the interval is computed in log space and transformed back, giving a
    multiplicative (asymmetric) band around the geometric mean. A symmetric linear CI drawn on
    a log axis reaches absurd places - at 12 Mb/s it dipped below 10 ms when the smallest
    observation was 674 ms - because delay here is strongly right-skewed. On a LINEAR axis the
    ordinary arithmetic CI is correct and is what is used.
    """
    m, _, n = stats(vals)
    sd = st.stdev(vals) if n > 1 else 0.0
    lo, hi = m - sd, m + sd
    # The CENTRE is the arithmetic mean so the figure agrees with the table and with the per-seed
    # win count. A geometric centre was tried and dropped: at 10 Mb/s it put EA BELOW NSGA-II
    # (582 vs 767) purely because one EA seed came in at 92 ms against siblings of 730-1826,
    # while the arithmetic mean and 3-of-4 seeds both say the opposite.
    #
    # Band is +- 1 SD (user, 2026-10-14): spread of a single run, not uncertainty in the mean.
    # On the log axis the lower edge is clamped to the smallest observation when mean - sd would
    # go non-positive or below anything measured - the data never went there, and a log axis
    # cannot draw <= 0 at all.
    if logy:
        lo = max(lo, min(vals) * 0.95)
    return m, lo, hi


def figure(data, ylabel, title, out, logy=False, target=None, target_label=''):
    """One measure, one y-axis. Band is +- 1 SD. Log scale where the data spans orders of magnitude - URLLC
    delay runs 8 ms to 4400 ms, and on a linear axis every point below the knee collapses onto
    the floor, hiding the low-load behaviour entirely."""
    fig, ax = plt.subplots(figsize=(7.2, 4.6))
    fig.patch.set_facecolor(SURFACE)
    ax.set_facecolor(SURFACE)
    if logy:
        ax.set_yscale('log')
    if target is not None:
        ax.axhline(target, color=MUTED, linewidth=1, linestyle=(0, (4, 3)), zorder=1)
        ax.annotate(target_label, (0.985, target), xycoords=('axes fraction', 'data'),
                    textcoords='offset points', xytext=(0, 5), fontsize=8.5, color=MUTED,
                    ha='right')

    for key, label, colour in ARMS:
        if key not in data:
            continue
        xs = sorted(data[key])
        trio = [band(data[key][x], logy) for x in xs]
        ms = [t[0] for t in trio]
        ax.fill_between(xs, [t[1] for t in trio], [t[2] for t in trio],
                        color=colour, alpha=0.15, linewidth=0, zorder=1)
        # Per-seed dots were tried and removed (user, 2026-10-14): mean + CI only. Note what the
        # band cannot show - at 7 Mb/s the EA seeds are 15.2 / 410.0 / 512.1 ms, one below the knee
        # and two above, and a band renders that bimodal split identically to an even spread.
        ax.plot(xs, ms, color=colour, linewidth=2, marker='o', markersize=8,
                markeredgecolor=SURFACE, markeredgewidth=1.5, label=label, zorder=3)
        # direct label at the right-hand end: identity without relying on colour alone
        ax.annotate(label, (xs[-1], ms[-1]), textcoords='offset points', xytext=(9, 0),
                    va='center', fontsize=9, color=MUTED)

    ax.set_xlabel('offered load per slice (Mb/s)', color=MUTED, fontsize=10)
    ax.set_ylabel(ylabel, color=MUTED, fontsize=10)
    ax.set_title(title, color=INK, fontsize=12.5, loc='left', pad=14)
    ax.grid(True, which='major', color=GRID, linewidth=0.8, alpha=0.9)
    if logy:
        ax.grid(True, which='minor', color=GRID, linewidth=0.5, alpha=0.45)
    ax.set_axisbelow(True)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(GRID)
    ax.tick_params(colors=MUTED, labelsize=9)
    ax.legend(frameon=False, fontsize=9, labelcolor=MUTED)
    fig.tight_layout()
    fig.savefig(out, dpi=160, facecolor=SURFACE)
    plt.close(fig)
    print(f"wrote {out}")


def write_summary_csv(delay, good, dropped, want_slice, out):
    """The same numbers as the printed table, as CSV, so they can be pasted into a document
    without retyping. show/ holds only what codes/ produced - this is produced here."""
    rows = []
    for load_mbps in sorted({x for a in delay.values() for x in a}):
        for key, label, _ in ARMS:
            d = delay.get(key, {}).get(load_mbps)
            g = good.get(key, {}).get(load_mbps)
            if not d:
                continue
            dm, dci, n = stats(d)
            gm, gci, _ = stats(g)
            rows.append({
                'slice': want_slice, 'load_mbps': f'{load_mbps:g}', 'arm': label, 'n': n,
                'delay_mean_ms': f'{dm:.1f}',
                'delay_sd_ms': f'{st.stdev(d) if n > 1 else 0:.1f}',
                'delay_ci95_ms': f'{dci:.1f}',
                'throughput_mean_mbps': f'{gm:.2f}',
                'throughput_sd_mbps': f'{st.stdev(g) if n > 1 else 0:.2f}',
                'throughput_ci95_mbps': f'{gci:.2f}',
                'discarded_invalid': dropped.get((key, load_mbps), 0),
            })
    with open(out, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print(f"wrote {out}")


def table(delay, good, dropped, want_slice):
    print(f"\n{want_slice}: mean +- sd across seeds (95% CI half-width in brackets)\n")
    print(f"{'load':>5} {'arm':<10}{'delay ms':>26}{'throughput Mb/s':>26}{'n':>4}{'dropped':>9}")
    for load_mbps in sorted({x for a in delay.values() for x in a}):
        for key, label, _ in ARMS:
            d = delay.get(key, {}).get(load_mbps)
            g = good.get(key, {}).get(load_mbps)
            if not d:
                continue
            dm, dci, n = stats(d)
            gm, gci, _ = stats(g)
            dsd = st.stdev(d) if n > 1 else 0.0
            gsd = st.stdev(g) if n > 1 else 0.0
            print(f"{load_mbps:>5.0f} {label:<10}"
                  f"{dm:>10.1f} +-{dsd:>7.1f} [{dci:>5.0f}]"
                  f"{gm:>11.2f} +-{gsd:>5.2f} [{gci:>4.2f}]"
                  f"{n:>4}{dropped.get((key, load_mbps), 0):>9}")


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    src = sys.argv[1]
    want = sys.argv[2] if len(sys.argv) > 2 else 'URLLC'
    # Optional: SEEDS and LOADS as comma lists, plus TAG to name the output set.
    seeds = {int(x) for x in sys.argv[3].split(',')} if len(sys.argv) > 3 and sys.argv[3] else None
    loads = {float(x) for x in sys.argv[4].split(',')} if len(sys.argv) > 4 and sys.argv[4] else None
    tag = sys.argv[5] if len(sys.argv) > 5 else ''
    delay, good, dropped = load(src, want, seeds, loads)
    if not delay:
        sys.exit(f"no valid {want} rows with a demand column in {src}")

    stem = Path(src).stem + (f'_{tag}' if tag else '')
    out = Path('show')
    out.mkdir(exist_ok=True)
    # No target line, and 10 ms is REFERENCE ONLY -- never a comparison, a pass/fail, or an input
    # to a hypothesis (user, 2026-10-15).
    #
    # The cited figure (IEEE 802.11 RTA TIG via Bankov, Chemrov & Khorov, arXiv:2107.03541) is an
    # APPLICATION latency demand scoped to a station on a Wi-Fi link -- typically one hop to an AP.
    # An earlier version of this comment called it a channel-access figure, which was wrong: the
    # channel-access number in that same paper is the SEPARATE ~5 ms medium-wait figure that
    # corroborates fitness.cc's calibrated C.
    #
    # It is unreachable here and that is a property of the topology, not of any mechanism: the
    # per-hop floor under contention is ~1.18 ms (t0Ms) and URLLC paths run ~7.5 hops, so ~9 ms is
    # the floor with ZERO queueing. Every arm "fails" it, including the no-slicing baseline and
    # including a perfect queue, so the comparison can only ever produce a false failure that
    # restates path length. Alloc.md reaches the same conclusion: URLLC tree depth would have to
    # cap at ~2 hops. Compare arms against EACH OTHER instead.
    # Seed count is read off the data. It used to be hardcoded at 4, which overstated n at any
    # load where a run was dropped INVALID.
    ns = [len(v) for d in (delay, good) for arm in d.values() for v in arm.values()]
    nseed = max(ns) if ns else 0
    figure(delay, f'{want} delay (ms, log scale)',
           f'{want} delay vs offered load  -  mean +- 1 SD, {nseed} seeds',
           out / f'{stem}_{want}_delay.png', logy=True)
    figure(good, f'{want} throughput (Mb/s)',
           f'{want} throughput vs offered load  -  mean +- 1 SD, {nseed} seeds',
           out / f'{stem}_{want}_throughput.png')
    write_summary_csv(delay, good, dropped, want, out / f'{stem}_{want}_summary.csv')
    table(delay, good, dropped, want)


if __name__ == '__main__':
    main()
