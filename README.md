# Resource Allocation and Routing with NSGA-II for Network Slicing in Wireless Mesh Networks

## Result: routing, measured standalone

NSGA-II against Seng et al.'s scalar EA on a 10x10 802.11ax ad-hoc mesh, 50 m spacing, three
IMT-2020 slices (eMBB / URLLC / mMTC) at equal offered load.

90 runs. 5 GA seeds per cell, 40 s of traffic, the last 10 s measured. Mean +- sd across seeds.

| | | load 2 | load 4 | load 6 | load 8 | load 10 | load 16 |
|---|---|---:|---:|---:|---:|---:|---:|
| URLLC delay (ms) | NSGA-II | 7.3 +- 1.3 | 7.5 +- 1.0 | 17.0 +- 7.0 | 46.5 +- 12.9 | 64.6 +- 29.0 | 242.2 +- 146.2 |
| | EA (Seng) | 9.2 +- 1.3 | 9.6 +- 4.7 | 16.7 +- 7.1 | 48.4 +- 15.6 | 186.3 +- 209.9 | 1017.3 +- 91.3 |
| URLLC thpt (Mb/s) | NSGA-II | 2.08 +- 0.00 | 4.16 +- 0.00 | 6.25 +- 0.01 | 8.22 +- 0.32 | 10.46 +- 0.03 | 17.03 +- 0.23 |
| | EA (Seng) | 2.08 +- 0.00 | 4.16 +- 0.00 | 6.25 +- 0.00 | 8.35 +- 0.02 | 10.55 +- 0.15 | 13.72 +- 0.40 |
| eMBB delay (ms) | NSGA-II | 8.1 +- 1.5 | 10.8 +- 2.8 | 26.7 +- 13.9 | 74.2 +- 17.1 | 104.1 +- 41.6 | 301.5 +- 113.9 |
| | EA (Seng) | 9.6 +- 1.5 | 11.8 +- 8.0 | 23.3 +- 8.1 | 69.2 +- 37.7 | 239.3 +- 295.4 | 1173.5 +- 188.3 |
| eMBB thpt (Mb/s) | NSGA-II | 2.08 +- 0.00 | 4.16 +- 0.00 | 6.26 +- 0.01 | 8.37 +- 0.03 | 10.50 +- 0.05 | 17.14 +- 0.28 |
| | EA (Seng) | 2.08 +- 0.00 | 4.16 +- 0.00 | 6.26 +- 0.00 | 8.37 +- 0.04 | 10.65 +- 0.29 | 13.52 +- 2.00 |
| seeds (n) | NSGA-II | 5 | 5 | 5 | 5 | 5 | 5 |
| | EA (Seng) | 5 | 5 | 5 | 5 | 4 | 3 |

Allocation is disabled in both arms, so the routing method is the only variable.

- **Loads 2 to 8.** The two arms report the same delay and throughput. All offered traffic is
  delivered, so the choice of routes has no effect at these loads.
- **Loads 10 and 16.** The two arms diverge.
- **Three runs did not drain**, all in the EA arm; NSGA-II completed 45 of 45. These are excluded,
  which is why EA reports 4 seeds at load 10 and 3 at load 16.
- **Loads 12 and 14 were not measured.** They were omitted for run time rather than excluded for a
  result. The segment between loads 10 and 16 in each figure is therefore interpolated.

### URLLC delay vs offered load

![URLLC delay](results/show/10seed1_routing_full_URLLC_delay.png)

### eMBB throughput vs offered load

![eMBB throughput](results/show/10seed1_routing_full_eMBB_throughput.png)

### Notes

- Throughput counts the IP and UDP headers, so it reads ~4% above the offered load.
- Both arms use the same link table, `results/linktable/10/10seed1.csv`.
- Each load point re-runs the GA, so compare the two arms at one load, not one arm across loads.
- The EA is a port of Seng et al.'s eq 3. Their eq 9 has no published values for its weights, so
  these numbers are not a reproduction of their paper.

## Layout

| path | holds |
|---|---|
| `harness/` | the testbed — one `main` executable |
| `nsga/` | NSGA-II routing backbone, and the Seng baseline |
| `ofdma/` | per-slice allocation (ns-3-free) |
| `gates/` | standalone validation experiments |
| `fail/` | archived dead ends, kept as evidence |
| `results/linktable/` | measured link tables, `<grid>/<grid>seed<N>.csv` |
| `results/raw/` | extracted measurements, one CSV per experiment |
| `results/codes/` | the scripts that turn run output into tables and figures |
| `results/show/` | figures and summary tables — output of `codes/` only |

`results/etc/` is run-output staging and is not tracked.

## Run

```
./ns3 run mesh-slicing/main -- --gridWidth=10 --spacing=50
```

Reproduce the table above:

```
./ns3 run "mesh-slicing/main --gaRoute --gridWidth=10 \
    --linkTable=scratch/mesh-slicing/results/linktable/10/10seed1.csv \
    --router=nsga --gaSeed=1 --embbDemand=16 --urllcDemand=16 --mmtcDemand=16 \
    --trafficSec=40 --measureSec=10 --plainQdisc=ns3::FqCoDelQueueDisc \
    --resultsDir=scratch/mesh-slicing/results/etc/run-a/"

python3 results/codes/extract.py results/raw/out.csv <logs>
python3 results/codes/plot.py results/raw/out.csv URLLC 1,2,3,4,5 2,4,6,8,10,16 full
```
