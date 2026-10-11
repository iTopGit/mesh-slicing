# Dual-Channel Mesh — real OFDMA in a Wi-Fi 6 mesh

## 0. The decision — measured three ways

`grid-dualrole-gate` runs three structures on the **same** 25 nodes, 5x5 @ 50 m, same Spectrum PHY at 22 dBm, same 20 random flow pairs (own `mt19937` — the builds consume ns-3's global RNG differently). Only the MAC/channel structure changes.

| Structure | `--roles` | Radios | Channels | Links at 1 send | RUs possible |
|---|---|---|---|---|---|
| today's harness | `adhoc` | 1 | 1 | 40/40 | ❌ |
| multi-channel ad-hoc | `adhoc-mc` | 2 | 2 | 40/40 | ❌ |
| dual-role BSS | `tree` | 2 | 2 | 24/40 | ✅ |

| 5 Mb/s offered | goodput | delay | loss | flows alive |
|---|---|---|---|---|
| 1ch ad-hoc | 5.06 | 1.37 ms | 0.41 % | 20/20 |
| 2ch BSS | 5.08 | 3.31 ms | 0.00 % | 20/20 |
| **2ch ad-hoc-mc** | 5.06 | **0.99 ms** | 0.45 % | 20/20 |

| 10 Mb/s offered | goodput | delay | loss | flows alive |
|---|---|---|---|---|
| 1ch ad-hoc | 10.02 | 44.6 ms | 1.48 % | 20/20 |
| 2ch BSS | 8.79 | 245 ms | 13.6 % | 20/20 |
| **2ch ad-hoc-mc** | **10.17** | **1.79 ms** | **0.00 %** | 20/20 |

| 40 Mb/s offered | goodput | delay | loss | flows alive |
|---|---|---|---|---|
| 1ch ad-hoc | 24.77 | 442 ms | 39.2 % | 20/20 |
| 2ch BSS | 15.77 | 178 ms | 61.3 % | **17/20** |
| **2ch ad-hoc-mc** | **38.35** | **101 ms** | **5.82 %** | 20/20 |

**`adhoc-mc` wins every load.** At saturation it carries **2.4x the BSS goodput with 10x less loss**. Do not read BSS's 178 ms at 40 Mb/s as a win — three of its flows died, so their packets never enter the mean (survivor bias). Judge by loss and flows alive.

BSS's one apparent advantage — 0.00 % loss at 5 Mb/s — does **not** hold up: `adhoc-mc` also hit 0.00 % at 10 Mb/s. Collision-free access did not translate into a measurable reliability edge.

### Why BSS loses

1. **1.20x more radio sends** (4.00 vs 3.33) — the boss-repeat detour, §1.
2. **AP store-and-forward** — every member→member hop queues at the boss.
3. **MU scheduler overhead** — 6 of 13 BSSs hold a single child, so trigger frames buy nothing there.

`adhoc-mc` takes the second radio's half-duplex win (relay transmits on its own colour's channel, receives on the other) and pays none of those three. Independently confirmed by `hop-delay-gate`: BSS was 1.5–5x slower than 2-radio ad-hoc in a chain.

### So why build BSS at all

One reason, and it is not performance:

| | ad-hoc (either) | BSS |
|---|---|---|
| MU PPDUs emitted | **0** | **1980** measured |
| Who decides who sends | everyone contends (CSMA) | **the AP grants time** |
| Slice control | source rate cap (metering) | scheduler assigns RUs |

`MultiUserScheduler` aborts on any non-AP MAC (`multi-user-scheduler.cc:183`), and 802.11ax MU-OFDMA is AP-centric **in the standard** — IBSS has no trigger frames, in ns-3 or in silicon. No AP → no RUs, ever, at any radio count. BSS is the only structure in which slice-aware RU assignment is *possible*.

**Thesis framing.** Do not claim the dual-role design is faster. Claim the cost and justify it:

> *"We quantified the price of AP-based scheduling in a mesh — 2.4x throughput and 10x loss versus multi-channel ad-hoc at saturation — and accept it because RU-level slice isolation is unreachable without an AP."*

### The measurement this does NOT settle

All runs above carry **20 unsliced random flows** against a **slice-blind** scheduler: all of OFDMA's overhead, none of its purpose. The thesis question is narrower:

> Does slice-aware scheduling protect **URLLC latency** better than rate-shaping over `adhoc-mc`?

Worse aggregate goodput with far better URLLC delay is still a win — that is what slicing is for. Until eMBB+URLLC traffic is run against a slice-aware scheduler, BSS is justified by *capability*, not by these numbers.

Open: `--mu=false` at saturation (how much of the loss is pure MU overhead), and the per-slice URLLC comparison.

---

## 0b. Before dropping dual channel — what to test

Goal: prove the **1-channel harness** (rate-cap slicing) is good enough on its own. If it is, dual channel can be dropped and this whole document becomes a measured cost chapter instead of an implementation plan.

Five tests, in order. Test 3 is the one that actually decides.

| # | Test | Question it answers | Command |
|---|---|---|---|
| 1 | **Does it still run?** | The URLLC 744 → 26 ms result is old. Code changed since (results dir, pcap default, `InstallSliceTraffic` signature). Re-run it. | `--gaRoute --enforce` on 5x5, 3 slices |
| 2 | **Is the capacity estimate right?** | Two known bugs make it wrong, so today's protection may be **luck**. Fix, then re-measure. | `enforcement.cc:90` drops the `max(reuse, 1.0)` clamp, calibrate reuse ≈ 0.72; `allocator.cc` gains a `rhoTotal` ≈ 0.7 aggregate headroom |
| 3 | **How wrong can the estimate be before URLLC breaks?** | **THE DECIDER.** We already know it is fragile: `reuse=2` sent URLLC **26 → 349 ms**. Everything rests on one number being right. | sweep `--reuse` and `--rhoU`, plot URLLC delay |
| 4 | **Does it survive changing demand?** | Shares are computed once. If eMBB steps up mid-run they go stale. | eMBB step change mid-run; if it breaks, land `ofdma-dynamic-gate`'s controller |
| 5 | **Baseline numbers for the write-up** | The "before" the thesis must beat. | `--olsr`, and `--gaRoute` without `--enforce` |

**How to read test 3.**

| Result | Meaning |
|---|---|
| URLLC survives across a **wide** range of wrong estimates | 1 channel is solid → **drop dual channel**, keep this doc as the cost chapter |
| URLLC breaks as soon as the estimate is slightly off | 1 channel is brittle → that brittleness is exactly what real OFDMA fixes, and dual channel earns its cost |

**Why rate-capping might be brittle.** It works by guessing the network's capacity, then telling each slice "send no more than your share". Guess too high → the pipe overfills and URLLC dies. Guess too low → throughput is wasted. Real OFDMA never guesses: the AP hands out RUs every TXOP based on what is actually queued.

---

## 1. Where extra hops come from

### The rule

A node has 2 radios:

```
AP  radio  -> runs its own group; can talk to every member
STA radio  -> belongs to ONE other group; can talk to that group's boss
```

Inside a group, **members cannot talk to each other**. The boss repeats the frame.

So a single "hop" costs 1, 2 or 3 radio sends:

| Shape | Sends |
|---|---|
| boss → its member | **1** |
| member → its boss | **1** |
| member → member (same group) | **2** — boss repeats |
| across two groups | **3+** |

### Why it can't be fixed with 2 radios

One STA radio = membership in one group = **one** neighbour reachable in a single send.

```
25 nodes x 1 membership  =  25 direct links possible
5x5 grid has                40 links
                            -----------------------
                            15 links must detour
```

Pure counting. No boss assignment escapes it. The current gate already achieves the maximum, 24.

Reaching all 4 neighbours directly would mean joining 4 groups — 4 radios **and** 4 channels, because a node's radios must sit on different channels or they jam each other. (Note: the 4 bosses are all "black" in a checkerboard, but they are still 4 *separate* groups, so they still need 4 *separate* channels. Colour ≠ channel.)

### Worked example — nodes 0, 1, 5, 6

```
 0 --- 1          bosses:  1 is boss of 0
 |     |                   6 is boss of 1 and 5
 5 --- 6
```

`0 → 5` — they are neighbours, but no group holds both:

```
0 → 1     0 is 1's member                1 send
1 → 6     1 goes up to its boss          1 send
6 → 5     boss hands down to member      1 send
                                         -------
                                         3 sends
```

The middle looks like "1 talks to 5", but 1 and 5 are both *members* of group 6. Boss 6 must repeat. That hidden repeat is the third send.

### Measured on the 5x5 (gate's boss rule = step toward centre)

All 40 neighbour pairs:

| Sends | Pairs |
|---|---|
| 1 ✅ | 24 |
| 3 | 12 |
| **5** | **4** |

Longer pairs — note distance predicts almost nothing:

| Pair | Grid steps apart | Path (bold = hidden boss repeat) | Sends |
|---|---|---|---|
| 0 → 7 | 3 | 0→1→6→7 | **3** |
| 0 → 2 | 2 | 0→1→6→**7**→2 | 4 |
| 0 → 11 | 3 | 0→1→6→7→**12**→11 | 5 |
| 0 → 10 | 2 | 0→1→6→7→**12**→11→10 | **6** |

`0 → 10` is 2 steps away and costs 6 sends. `0 → 7` is 3 steps away and costs 3. What matters is **whether you share a boss chain**, not how close you are.

### How much it actually hurts

| Cause | Latency cost |
|---|---|
| Extra hops | **~1 ms** |
| Running above the capacity knee | **~200 ms** |

Measured whole-mesh average at clean load: 2.52 ms over ~4 sends, so ~0.6 ms per send. The detour is real but small. **Congestion is the latency killer, not the detour** — and scheduled access is what attacks congestion.

### The free knob

**Which** neighbour is your boss is yours to choose, and costs nothing. Two rules worth trying:

1. **Protect URLLC** — put URLLC pairs on boss chains so they get 1-send hops; let eMBB absorb the detours.
2. **Spread load** — today's "step toward centre" rule funnels ~30 % of all pairs through the centre node (busiest relay 294 vs typical ~120).

Boss choice is resource allocation, decided before routing. It fits the thesis separation.

---

## 2. The enabling fact

`MultiUserScheduler` only installs on an HE **AP** (`multi-user-scheduler.cc:183`, hard abort), and one `WifiNetDevice` has one MAC type fixed at install. That looked like "OFDMA is impossible in a mesh". It is not:

**One node can carry two `WifiNetDevice`s with different MAC types on different channels** — AP on ch A, STA on ch B. Real wireless repeaters work this way.

Two radios also satisfy the colouring paper's collision rule 3 (never transmit and receive in the same band at once) by construction.

### Traps — all cost real days

| Trap | Fact |
|---|---|
| **Yans cannot do OFDMA** | It models a channel as one indivisible unit, so it cannot represent RUs. Every MU PPDU fails **silently** — no error. `SpectrumWifiPhy` is mandatory (`wifi-he-network.cc:179-182`). |
| **Spectrum costs ~6 dB vs Yans** | Same geometry/loss model/MCS: Yans 0 % loss at the 16.02 dBm default, Spectrum 100 %. Spectrum needs **22 dBm**, which also covers 50 m spacing. |
| **Association ≠ working link** | Management frames go out non-HT on the primary 20 MHz, ~6 dB quieter than an 80 MHz HE data PPDU. STAs associate at powers where data delivers nothing. Always judge by goodput. |
| **80 MHz needs a centre channel number** | 42/58/106/122/138/155/171. Channel 36 aborts with *"No unique channel found"*. |
| **Loopback is device 0 AND interface 0** | Device and Ipv4 interface indices coincide — never add one. A wrong index silently SIGSEGVs in `RouteOutput`. Use `Ipv4::GetInterfaceForDevice`. |
| `PhyTxPsduBegin` | Takes `WifiTxVector` **by value**; the doc typedef (`wifi-phy.h:728`) contradicts the real callback (`wifi-phy.h:1527`). Mismatch prints identical "got"/"expected". |
| `PhyRxBegin` | Signature is `(Ptr<const Packet>, RxPowerWattPerChannelBand)`. |

---

## 3. Architecture — BSSs over the GRID, not the GA tree

Assign AP/STA roles and channels over the **physical grid**, once, at setup. The GA then routes over the resulting BSS-link graph.

Deriving BSS structure from the GA's tree would make **resource allocation depend on routing** — the coupling this project exists to avoid, and the reverse of the colouring paper's order.

Consequences: routing and allocation stay separate; topology is built at setup, no two-phase run; **OFDMA is testable without the GA at all**.

Pairing rules:
- A node's AP channel == its members' STA channel.
- A node's own AP channel **must differ** from its own STA channel.

**Channel count follows cycle parity.** 2-colouring needs no odd cycles. A rectangular grid is **bipartite** → 2 channels, always. A mesh with an odd cycle needs 3.

Current gate rule: `chan = (x+y) % 2`. A boss is always a grid neighbour, so it always has the opposite parity — the two radios can never collide. Verified at 5x5: **0 violations**.

---

## 4. How slices get separated

802.11ax addresses an RU by **AID** (the short number an AP gives a STA at association). Each MU transmission carries a map: *"RU1 → AID 3, RU2 → AID 7"*. **The label is per device, not per slice.** Two RUs cannot go to one STA — that is 802.11be multi-RU, which ax lacks and ns-3 has no scheduler for.

| Hop shape | Mechanism | Simultaneous? |
|---|---|---|
| **Branch** — slices take different children | **RU per AID** | yes, different subcarriers at once |
| **Chain** — all slices cross one AP→STA link | **A-MPDU composition** (Marrone's "filling ratio") | one transmission, sequential inside |

Both are AP-controlled and deterministic. Neither is CSMA — no backoff between slices, no collisions.

SU-OFDMA still counts: an AP with one STA gives it the whole channel as one RU, and the win is that the AP *schedules* instead of everyone contending. **No ≥2-STA requirement.**

**Thesis wording** — claim ✅ *"scheduled access replaces contention-based access"* and ✅ *"slices receive proportional airtime under AP control"*. Never ❌ *"slices are isolated in frequency"* (that needs slices on separate STAs).

**Two channels are for the dual-role node, not for OFDMA.** OFDMA lives inside one channel.

### Relation to the source papers

```
colouring paper  -> which CHANNEL each BSS uses   (between BSSs)
ax MU scheduler  -> split that channel into RUs   (inside a BSS)
slice-aware ver. -> assign those RUs by slice     (the contribution)
```

Different layers, so they compose. We adopt the colouring paper's interference model at **channel** granularity and drop its time-slot half (ns-3 has no slot scheduler). Adaptation, not reproduction — state that limitation.

**The prize:** `RrMultiUserScheduler` is round-robin per STA and **slice-blind**. Marrone et al. published a slice-aware one (`SlicingScheduler`, driven by `ratio_slice`) for a single AP, single hop. Porting it into mesh BSSs = slice-aware RU assignment in a multi-hop mesh, which is in neither paper. (`ratio_slice` ≠ our `rho`: theirs is a per-slice throughput share with the channel deliberately full; our `rho` exists only because we enforce over a shared contended medium.)

---

## 5. What is proven

| Gate | Question | Verdict |
|---|---|---|
| `dual-role-gate` | Can one node be AP + STA at once, doing real MU-OFDMA? | ✅ **PASS ×3** — assoc both directions; **1980 DL-MU PPDUs**, 2 STAs each (AIDs 1+2); 12.5 MB forwarded per leaf |
| `hop-delay-gate` | Is AP-scheduled access faster than CSMA in a **chain**? | ❌ **negative** — the win is the 2nd radio+channel (892×), not OFDMA. `bss` is 1.5–5× slower in a chain (1 STA/BSS = nothing to multiplex) |
| `star-ofdma-gate` | Does OFDMA pay in a **star**? | ⚠️ **split** — UPLINK (4→1) **+76 % goodput** (41.6→73.0 Mb/s), loss 50 %→10.7 %. DOWNLINK no gain. Judge goodput, not delay |
| `ring-bss-gate` | Does the structure survive **cycles**? | ✅ **all PASS** on 4/6/8-node rings — parity (2/3/2 ch), 2-BSS membership, and forwarding through a node with **two STA radios**. Forcing 2 channels on the odd ring reproduces the predicted violation |
| `grid-dualrole-gate` | Does it hold at **harness scale**? | ✅ **PASS** — 5x5 @ 50 m, 13 BSSs, 24 STA radios, 600 routes, 20 random flows, no crash |

**Direction matters:** OFDMA pays uplink (many→one), not downlink. In a mesh, uplink = children→parent, the common case. ns-3 supports it: `RrMultiUserScheduler::EnableUlOfdma` defaults **true**, doing BSRP → Basic Trigger Frame → TB PPDU (`rr-multi-user-scheduler.cc:147-166`).

---

## 6. Capacity — the numbers to design against

`grid-dualrole-gate`, 5x5 @ 50 m, 22 dBm, 20 random flows:

| Offered | Delivered | Loss | Delay | Goodput |
|---|---|---|---|---|
| 5 Mb/s | 20/20 | **0.000 %** | **2.52 ms** | 5.08 |
| 6 Mb/s | 20/20 | 1.42 % | 6.71 ms | 6.01 |
| 10 Mb/s | 20/20 | 17.4 % | 197 ms | 8.40 |
| 40 Mb/s | 12/20 | 59.3 % | 146 ms | 16.6 |

**Knee at ~6 Mb/s.** Where it goes:

| Step | Value |
|---|---|
| one BSS link, 50 m, 80 MHz | **110 Mb/s** (measured) |
| avg radio sends per flow | 4.00 |
| ceiling with perfect reuse | 27.5 Mb/s |
| measured at saturation | 16.6 Mb/s (60 % — CSMA + hidden terminals) |
| measured clean | 5 Mb/s |

300 Mb/s never appears: SNR at 50 m is ~12 dB, which caps the MCS.

⚠️ **Judge by loss and flows-delivered, never by mean delay alone.** The 40 Mb/s row shows *lower* mean delay than the 10 Mb/s row — survivor bias, because the long flows died and only short fast ones remain to average.

**Earlier baseline** (modelled enforcement, 5x5, 2026-08-19), kept for comparison: URLLC delay 211/636/744/621 ms OFF → 27.7/23.5/26.3/38.0 ms ON, 0 % loss at every load. Link R ≈ 50 Mb/s, end-to-end ≈ 11 Mb/s, mean hops ≈ 3.27, `C` ≈ 4.53 ms/hop.

---

## 7. What is left

| Piece | Where | Effort |
|---|---|---|
| **Slice-aware scheduler** — the actual contribution | port from MeditCom repo | hard |
| Boss selection that protects URLLC | `grid-dualrole-gate` | easy |
| Yans → Spectrum, 22 dBm | `harness/topology.cc` | medium — **prerequisite** |
| 2 devices/node, one subnet per BSS | `harness/topology.cc`, `harness/internet.cc` | medium — a flat /16 breaks ARP; `AddHostRouteTo` already takes an interface index |
| Warm-up per channel | `harness/warmup.cc` | medium |
| Contention per **channel** not per link | `nsga/fitness.cc` | medium — colouring makes non-interference structural, so the link-only model becomes *correct* rather than approximate |

**Unchanged:** `nsga/individual.h`, `nsga/nsga2.cc`, `nsga/graph.cc`, `ofdma/allocator.cc`, `ofdma/enforcement.cc`, the `--enforce` path.

**Tests not yet run:** same-channel BSS interference vs distance (gives the minimum reuse distance); slices onto RUs (expected to **fail** — the scheduler is slice-blind; that negative result motivates the port); Spectrum PHY cost at 10x10; GA integration.

---

## 8. Does not buy

- **1 ms URLLC.** Floor is `C × hops`; multi-hop keeps this above 1 ms. IMT-2020's 1 ms assumes single-hop scheduled licensed spectrum. Report the gap.
- **The time-slot half** of the colouring scheme.

## 9. Rejected — do not revisit

| Idea | Why |
|---|---|
| Single AP for the whole network | single-hop star → no routes → GA pointless |
| Backhaul + access dual channel | the experiment has no client nodes; traffic is generated on routers |
| Two radios on the **same** channel | self-interference at ~0 m + half duplex. Broken, not cheaper |
| Flipping one device between AP and STA | MAC type is fixed at `wifi.Install()` |
| Tuning `--reuse` / `rho` for low URLLC latency | trades eMBB against URLLC; does not make OFDMA real |
| 3–4 radios per node to kill the detour | works (40/40 links, 3 channels) but is grid-specific and not flexible; **2 radios + detour accepted instead** |

## 10. References

| Source | Use |
|---|---|
| `source/Resource_Allocation_in_OFDMA_Wireless_Mesh_Networks.pdf` (Priesler & Reichman, ICOF 2016) | colouring, collision rules, grid solution, routing/allocation separation |
| `source/Slicing/Smart Resource Allocation to Enable Network Slicing.pdf` (Marrone et al., MeditCom 2025) | ax OFDMA slicing in ns-3, `ratio_slice`, published `SlicingScheduler` |
| `examples/wireless/wifi-he-network.cc:179-182,331` | Spectrum requirement; `SetMultiUserScheduler` usage |
| `examples/wireless/mixed-wired-wireless.cc:157,304` | two MAC types on one node |
| `src/wifi/model/he/rr-multi-user-scheduler.cc:147-166` | UL MU-OFDMA (BSRP → Basic TF → TB PPDU), on by default |
