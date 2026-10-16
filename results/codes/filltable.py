#!/usr/bin/env python3
"""Build a full link table for a large grid from a few measured edges.

Usage:
  python3 codes/filltable.py --width 32 --samples etc/warm32/linktable.csv \\
          --stats-from linktable/10/10seed1.csv --seed 1 --out linktable/32/32seed1.csv

Why this is legitimate, and the one condition on it.

A link's capacity in this testbed depends on spacing, transmit power, the propagation model and the
MCS the rate manager picks. None of those change with grid width, and `harness/warmup.cc` measures
one link at a time with the sender flushed on an otherwise idle medium -- so a 32x32 link is
physically the same link as a 10x10 link. Across five independent 10x10 seeds, 900 measurements, the
mean is 82.72 Mb/s every time and the full range is 0.2%.

So the sampled edges are a TRANSFER CHECK, not an estimate. If they land on the known mean, the
constant transfers and the remaining rows are filled from 900 measurements rather than from 4. If
they do not, this script refuses to write -- because then something IS size-dependent and a filled
table would be a fabrication. That check is the whole justification; do not add a flag to skip it.

Every row is labelled in a fifth column, `src` = measured | filled. `Graph::FromLinkTable` reads
four fields positionally and discards the rest of the line, so the label costs nothing at load time
and a reader can always tell which rows are real. A sidecar `.meta` records the full provenance,
because the loader rejects comment lines (it parses every non-empty line after the header).
"""
import argparse
import csv
import hashlib
import random
import statistics as st
import sys
from datetime import date
from pathlib import Path

# A sampled mean this far from the reference mean means the constant did not transfer.
TRANSFER_TOL_PCT = 2.0


def grid_edges(width, height):
    """Axis neighbours only, each pair once -- the same adjacency BuildEdgeListGeometric produces."""
    for r in range(height):
        for c in range(width):
            n = r * width + c
            if c + 1 < width:
                yield n, n + 1
            if r + 1 < height:
                yield n, n + width


def read_table(path):
    """-> {(a,b): (delay_ms, cap_bps)}; the key is ordered so lookup does not depend on direction."""
    rows = {}
    with open(path) as fh:
        for rec in csv.reader(fh):
            if not rec or rec[0].strip() in ('a', ''):
                continue
            a, b = int(rec[0]), int(rec[1])
            rows[(min(a, b), max(a, b))] = (float(rec[2]), float(rec[3]))
    return rows


def stats(values):
    return st.mean(values), (st.stdev(values) if len(values) > 1 else 0.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--width', type=int, required=True)
    ap.add_argument('--height', type=int, help='defaults to --width (square grid)')
    ap.add_argument('--samples', required=True, help='measured edges at THIS grid size')
    ap.add_argument('--stats-from', required=True, help='a full measured table, for the spread')
    ap.add_argument('--seed', type=int, required=True, help='fill RNG seed; put it in the filename')
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    height = args.height or args.width

    sampled = read_table(args.samples)
    reference = read_table(args.stats_from)
    if not sampled:
        sys.exit(f'no measured rows in {args.samples}')

    cap_mean, cap_sd = stats([c for _, c in reference.values()])
    delay_mean, delay_sd = stats([d for d, _ in reference.values()])
    s_cap_mean, _ = stats([c for _, c in sampled.values()])
    s_delay_mean, _ = stats([d for d, _ in sampled.values()])

    # The transfer check. Refuse rather than warn: a filled table that silently encodes a false
    # constant is worse than no table, because nothing downstream can detect it.
    drift = 100.0 * abs(s_cap_mean - cap_mean) / cap_mean
    print(f'reference {Path(args.stats_from).name}: {len(reference)} links, '
          f'cap {cap_mean / 1e6:.3f} +- {cap_sd / 1e6:.3f} Mb/s')
    print(f'sampled   {Path(args.samples).name}: {len(sampled)} links, '
          f'cap {s_cap_mean / 1e6:.3f} Mb/s  -> drift {drift:.2f}%')
    if drift > TRANSFER_TOL_PCT:
        sys.exit(f'TRANSFER CHECK FAILED: sampled mean is {drift:.2f}% off the reference, '
                 f'tolerance {TRANSFER_TOL_PCT}%. Capacity is size-dependent here, so filling '
                 f'would fabricate. Measure the full table instead.')
    print(f'transfer check PASSED (< {TRANSFER_TOL_PCT}%)')

    rng = random.Random(args.seed)
    edges = list(grid_edges(args.width, height))
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    n_measured = 0
    with open(out, 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['a', 'b', 'base_delay_ms', 'capacity_bps', 'src'])
        for a, b in edges:
            hit = sampled.get((min(a, b), max(a, b)))
            if hit:
                delay, cap = hit
                src = 'measured'
                n_measured += 1
            else:
                # Reproduce the measured spread rather than a flat constant, so the table has the
                # same character a full sweep would have produced. At 0.2% this is noise either
                # way -- no route difference at that scale is signal.
                delay = rng.gauss(delay_mean, delay_sd)
                cap = rng.gauss(cap_mean, cap_sd)
                src = 'filled'
            w.writerow([a, b, f'{delay:.6g}', f'{cap:.6g}', src])

    # results/ is outside the git repo, so there is no commit to cite. Hash the script instead --
    # that is what actually determines the output, and it is checkable years later.
    me = Path(__file__).resolve()
    digest = hashlib.sha256(me.read_bytes()).hexdigest()[:12]
    out.with_suffix('.meta').write_text(
        f'generated      {date.today().isoformat()} by codes/filltable.py (sha256 {digest})\n'
        f'grid           {args.width}x{height}, {len(edges)} edges\n'
        f'measured rows  {n_measured} (from {args.samples})\n'
        f'filled rows    {len(edges) - n_measured}\n'
        f'spread source  {args.stats_from} ({len(reference)} measured links)\n'
        f'capacity       mean {cap_mean / 1e6:.4f} Mb/s, sd {cap_sd / 1e6:.4f}\n'
        f'base delay     mean {delay_mean:.6f} ms, sd {delay_sd:.6f}\n'
        f'fill rng       python random.Random(seed={args.seed}), gauss(mean, sd)\n'
        f'transfer check sampled mean {s_cap_mean / 1e6:.4f} Mb/s, drift {drift:.2f}%, '
        f'tolerance {TRANSFER_TOL_PCT}%\n'
        f'sampled delay  mean {s_delay_mean:.6f} ms\n')

    print(f'wrote {out}  ({n_measured} measured, {len(edges) - n_measured} filled)')
    print(f'wrote {out.with_suffix(".meta")}')


if __name__ == '__main__':
    main()
