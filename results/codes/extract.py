#!/usr/bin/env python3
"""Turn harness run logs into one tidy CSV for plotting.

Usage:  python3 codes/extract.py raw/out.csv  <log> [<log> ...]

One row per (run, slice). The filename names the EXPERIMENT; everything else lives in
columns, so a plot script filters instead of parsing filenames. That matters here because
a 0.15% difference in the link table was measured to flip the GA's chosen solution and move
URLLC delay 3x - so which table a row came from is load-bearing and belongs in the data,
not in a name someone can mistype.

Runs the gate reports as INVALID are kept with valid=0 rather than dropped. Discarding them
silently would hide that the discards correlate with congestion, which biases whatever mean
is computed from the survivors.
"""
import csv
import os
import subprocess
import re
import sys

CFG = re.compile(r'^config: (.*)$', re.M)
SLICE = re.compile(
    r'^slice (?P<slice>\w+) .*?thpt=(?P<thpt>[\d.]+)Mb/s '
    r'delay=(?P<delay>[\d.]+)ms loss=(?P<loss>[\d.]+)%', re.M)
# A run may print SEVERAL result blocks, one per measurement window (--measureSec=2,10,20). Each
# block is headed by its window length, so rows carry `window_s` and a plot can pick one. Without
# this the parser silently took whichever block came first, which is the LONGEST window -- a quiet
# wrong answer rather than an error.
WINDOW = re.compile(r'^--- measured over the last ([\d.]+) s of traffic ---$', re.M)
SETTLE = re.compile(r'^measurement window OPEN at t=([\d.]+)s .*?queues (.*)$', re.M)
# per-slice queue-disc totals: enqueues per source packet = traffic-weighted path length
HOPS = re.compile(r'^(?P<slice>eMBB|URLLC|mMTC): enq=\d+ drop=\d+\s+'
                  r'policed=(?P<policed>[\d.]+)% of \d+ src pkts\s+\(hops/pkt (?P<hops>[\d.]+)\)', re.M)
DRAIN_OK = re.compile(r'drain: queues empty after ([\d.]+) s')
DRAIN_STALL = re.compile(r'drain: stopped after ([\d.]+) s')
TABLE = re.compile(r'reusing link table (\S+)')
ARM = re.compile(r'=== (NSGA-II knee|Seng scalar)')

FIELDS = ['experiment', 'arm', 'qdisc', 'gaSeed', 'rngRun', 'grid', 'rhoStar', 'H', 'F',
          'mmtcH', 'secondRound', 'demand', 'linkTable', 'window_s', 'settle', 'slice',
          'delay_ms', 'loss_pct', 'throughput_mbps', 'hops_per_pkt', 'policed_pct',
          'valid', 'drain_s', 'runtime_min']


def runtime_min(path):
    """Wall-clock for one run, from the log file's birth time to its last write.

    Derived rather than printed by the simulator, so it works on logs already on disk. Linux
    exposes birth time only through statx, which Python's os.stat does not surface, hence the
    coreutils call; a filesystem without it returns 0 and the field is left blank.

    READ WITH CARE. This is wall clock under whatever else was running at the time, not the
    intrinsic cost of the run. Measured here: load 16 came out FASTER (15.6 min) than load 8
    (31.6) because the load-8 batch shared cores with a long straggler. Only compare runs
    launched at the same concurrency.
    """
    try:
        born = int(subprocess.run(['stat', '-c', '%W', path], capture_output=True,
                                  text=True).stdout.strip())
        if born <= 0:
            return ''
        return f'{(os.stat(path).st_mtime - born) / 60:.1f}'
    except Exception:
        return ''


def parse(path):
    text = open(path).read()
    if not SLICE.search(text):
        return []                       # crashed or still running

    cfg = {}
    m = CFG.search(text)
    if m:                               # self-describing log (harness prints this since 2026-10-14)
        for kv in m.group(1).split():
            k, _, v = kv.partition('=')
            cfg[k] = v
    else:                               # older log: recover what can be recovered, leave the rest blank
        t = TABLE.search(text)
        cfg['linkTable'] = t.group(1) if t else ''
        a = ARM.search(text)
        cfg['router'] = {'NSGA-II knee': 'nsga', 'Seng scalar': 'seng'}.get(
            a.group(1) if a else '', '')

    hops = {m.group('slice'): (m.group('hops'), m.group('policed')) for m in HOPS.finditer(text)}
    rt = runtime_min(path)
    valid = 'drain: INVALID' not in text
    d = DRAIN_OK.search(text) or DRAIN_STALL.search(text)

    # Settle verdict belongs to the SHORTEST window (the only one that queries the watcher).
    settle = ''
    sm = SETTLE.findall(text)
    if sm:
        settle = sm[-1][1].split(' -- ')[0].strip()  # older logs appended an explanatory clause

    # One chunk per window block; a log with no window header is a single whole-run block.
    marks = [(float(m.group(1)), m.end()) for m in WINDOW.finditer(text)]
    chunks = []
    if marks:
        for i, (w, start) in enumerate(marks):
            end = marks[i + 1][1] if i + 1 < len(marks) else len(text)
            chunks.append((w, text[start:end]))
    else:
        chunks.append(('', text))

    rows = []
    for win, chunk in chunks:
      for s in SLICE.finditer(chunk):
        rows.append({
            'experiment': path.split('/')[-1].rsplit('.', 1)[0],
            'arm': cfg.get('router', ''),
            'qdisc': cfg.get('qdisc', ''),
            'gaSeed': cfg.get('gaSeed', ''),
            'rngRun': cfg.get('rngRun', ''),
            'grid': cfg.get('grid', ''),
            'rhoStar': cfg.get('rhoStar', ''),
            'H': cfg.get('H', ''),
            'F': cfg.get('F', ''),
            'mmtcH': cfg.get('mmtcH', ''),
            'secondRound': cfg.get('secondRound', ''),
            'demand': cfg.get('demand', ''),
            'linkTable': cfg.get('linkTable', ''),
            'window_s': win,
            'settle': settle,
            'slice': s.group('slice'),
            'delay_ms': s.group('delay'),
            'loss_pct': s.group('loss'),
            'throughput_mbps': s.group('thpt'),
            'hops_per_pkt': hops.get(s.group('slice'), ('', ''))[0],
            'policed_pct': hops.get(s.group('slice'), ('', ''))[1],
            'valid': int(valid),
            'drain_s': d.group(1) if d else '',
            'runtime_min': rt,
        })
    return rows


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    out, logs = sys.argv[1], sys.argv[2:]
    rows = [r for p in logs for r in parse(p)]
    with open(out, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        w.writerows(rows)
    runs = len({(r['experiment']) for r in rows})
    bad = len({r['experiment'] for r in rows if not r['valid']})
    print(f"{out}: {len(rows)} rows from {runs} run(s), {bad} marked INVALID")


if __name__ == '__main__':
    main()
