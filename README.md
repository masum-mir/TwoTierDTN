# TwoTierDTN — ONE Simulator Extension

**Cross-Tier Routing and Bundle Management in Disruption-Tolerant UAV Networks**

This repository extends the ONE (Opportunistic Network Environment) Simulator with a custom hierarchical DTN router (`TwoTierRouter`), an energy consumption model with movement-cost tracking, and a corresponding reporting module. The implementation targets a UAV-assisted, two-tier DTN topology: **Node → Tower → Drone → Tower → Node**.

---

## 1. Overview of Components

| File | Package | Purpose |
|---|---|---|
| `TwoTierRouter.java` | `routing` | Custom router implementing strict two-tier topology enforcement with four borrowed routing mechanisms |
| `EnergyModel.java` | `routing.util` | Energy consumption model — scanning, scan-response, transmission, and movement costs |
| `EnergyLevelReport.java` | `report` | Per-node, per-tick snapshot of remaining energy and cumulative movement-energy consumption |
| `default_settings2.txt` | settings | Scenario configuration: 13 host groups, 3 router variants, full TTL/buffer parameter sweep |

---

## 2. TwoTierRouter — Implemented Features

`TwoTierRouter` extends `ActiveRouter` and combines four mechanisms borrowed and adapted from existing ONE Simulator router implementations.

### Feature 1 — Immunity (anti-flooding) 

- After a node forwards (or receives) message *M*, the message ID and timestamp are recorded in `immuneMessages`.
- For the next `immunityTime` seconds (default **600 s**, configurable via `TwoTierRouter.immunityTime`), the node refuses to re-accept the same message via `checkReceiving()` → `DENIED_POLICY`.
- **Effect**: prevents the same bundle copy from being re-flooded back and forth on every contact opportunity.

### Feature 2 — Custody-protected buffer (eviction guard) 

- On receipt, each message ID is timestamped in `custodyMessages`.
- For `immunityTime × custodyFraction` seconds (default **0.5 → 300 s**, configurable via `TwoTierRouter.custodyFraction`), the message is shielded from `getNextMessageToRemove()` and cannot be evicted even under buffer pressure.
- Once the custody window expires, the message becomes a normal eviction candidate (oldest-first, by `receiveTime`).
- **Effect**: freshly relayed bundles survive long enough to reach the next relay opportunity (tower ↔ drone) instead of being dropped immediately under buffer pressure.

### Feature 3 — PRoPHET-weighted, energy-aware drone selection 

- Towers maintain a per-drone delivery-predictability table `dronePreds` (PRoPHET formula):
  - On each tower↔drone contact: `P_new = P_old + (1 − P_old) × P_INIT` (`P_INIT = 0.75`)
  - Aging applied every `SECONDS_IN_UNIT = 30 s`: `P ← P × GAMMA^(Δt/30)` (`GAMMA = 0.98`)
- When multiple drones are simultaneously in range, the tower computes a **composite score**:

  ```
  score(drone) = P(tower, drone) × (E_current(drone) / E_initial(drone))
  ```

- Drone connections are sorted by descending `score`, and the message is offered to the highest-scoring drone first.
- **Effect**: prefers drones that are both *frequently encountered* (reliable relay) and *energy-rich* (sustainable relay), avoiding hand-off of bundles to near-depleted UAVs.

### Feature 4 — Delete-after-upload 

- Implemented in `transferDone(Connection con)`.
- When a **Node** successfully transfers a message to a **Tower** (the uplink hop), the node immediately deletes its own copy via `deleteMessage(id, false)`.
- Towers and Drones (relay roles) retain their copy until the next hop confirms receipt.
- **Effect**: frees node-side buffer space immediately after uplink, allowing the node's limited buffer (2 MB for pedestrian/vehicle groups) to accept new incoming (downlink) messages.

### Strict Topology Enforcement — `isAllowedRelay()`

All four features above operate within a hard topology constraint enforced at `checkReceiving()`:

| From → To | Allowed? | Notes |
|---|---|---|
| Node → Tower | ✅ | Uplink |
| Node → Node | ❌ | No peer-to-peer epidemic spreading |
| Node → Drone | ❌ (default) | Direct Node↔Drone contact not modeled; one hard-coded exception for host `m126` |
| Tower → Drone | ✅ | Backbone uplink |
| Tower → Node (destination only) | ✅ | Final-hop delivery only |
| Tower → Node (non-destination) | ❌ | No blind broadcast to unintended nodes |
| Drone → Tower | ✅ | Backbone downlink |
| Drone → Node | ❌ | Drones never talk to ground nodes directly |

### Forwarding Logic — `update()`

1. **Tower role**:
   - Pass 1 — scan buffer for any message whose destination is a directly-connected node and deliver it immediately.
   - Pass 2 — sort drone connections by `score(drone)` (Feature 3) and relay the oldest eligible message to the best-scoring drone, skipping messages already held by the peer or whose destination is already directly reachable.
2. **Node / Drone role**: strict single-hop forwarding — oldest-first message to the first allowed peer (Node→Tower, or Drone→Tower) that does not already hold the message.

Messages are always processed **oldest-first** (`sort by receiveTime`) to prevent starvation of long-resident bundles.

---

## 3. EnergyModel — Implemented Features

`EnergyModel` (in `routing.util`) is instantiated per-host when `initialEnergy` is present in the settings, and is propagated to the router via the `ModuleCommunicationBus` (`comBus`).

### Tracked energy components

| Component | Setting | Description |
|---|---|---|
| Initial energy | `Group.initialEnergy` | Starting battery level (single value or `[min;max]` range, uniformly sampled) |
| Scanning cost | `Group.scanEnergy` | Cost per device-discovery scan; charged once per second if scanning is active |
| Scan-response cost | `Group.scanResponseEnergy` | Cost incurred when another host connects (device-discovery response) |
| Transmission cost | `Group.transmitEnergy` | Cost per second while the interface is actively transferring data |
| **Movement cost** (new) | `Group.movementEnergy` | Cost per second of *active movement* — drones only |
| Warm-up period | `Group.energyWarmup` | Simulation time before energy depletion begins (0 = from start) |

### Movement-energy tracking (new addition)

- `updateMovementEnergy(simTime, isMoving)` is called once per tick, **only for Drone-role hosts**, from `TwoTierRouter.update()`.
- Movement is detected by comparing the host's current `Coord` against the previous tick's location (a defensive `.clone()` is required, since `getLocation()` returns a live reference).
- If the drone has moved since the last tick, `movementEnergy × Δt` is deducted from the battery and accumulated into `movementConsumed`.
- The cumulative movement-energy value is published on the `comBus` under `Energy.movement` (`MOVEMENT_ENERGY_ID`) so it can be read by `EnergyLevelReport`.
- **Note**: Pedestrian, vehicle, and tower groups also receive a `movementEnergy` value via the shared `Group.*` defaults, but `updateMovementEnergy()` is never invoked for non-drone hosts — so this cost is **inert for all roles except drones**.

### Energy bookkeeping

- `reduceEnergy(amount)` clamps the result at zero — energy never goes negative.
- `getInitialEnergy()` returns the midpoint of the configured range (or the single value), used as the normalisation denominator in Feature 3's drone-score calculation.
- `comBus` properties registered on first update: `Energy.value` (current), `Energy.initial` (starting reference), `Energy.movement` (cumulative movement drain).

---

## 4. EnergyLevelReport — Implemented Features

- A `SnapshotReport` / `UpdateListener` that runs at fine granularity (`EnergyLevelReport.granularity = 1` second).
- For each host, writes a single line:

  ```
  <hostName> remaining:<currentEnergy> movement:<cumulativeMovementEnergy>
  ```

- Reads `Energy.value` and `Energy.movement` directly from each host's `comBus`.
- Throws `SimError` if a host has no energy model registered (`Energy.value` property missing), ensuring all hosts in the scenario are configured consistently.

---

## 5. Scenario Configuration — `default_settings2.txt`

### Topology (13 host groups, 126 + 4 = 130 hosts total)

| Group(s) | Role | Count | Movement | Notes |
|---|---|---|---|---|
| 1, 3, 4, 13 | Pedestrian (`p`, `w`, `z`) | 25 each | ClusterMovement | 2 MB buffer |
| 2 | Vehicle (`c`) | 25 | ClusterMovement | 2 MB buffer |
| 5 | Primary Drone (`D`) | 2 | RandomWaypoint, 8–9 m/s | High-speed interface |
| 6–9 | Static Tower (`T`) | 1 each (4 total) | Stationary | 256 MB buffer, dual interface (bt + highspeed) |
| 10–12 | Primary Drone (`D`) | 1 each (3 total) | MapRouteMovement, 4–6 m/s | Follow `DronePath1.wkt` / `DronePath2.wkt` |

### Experiment sweep (210 simulation runs)

- **Routers**: `TwoTierRouter`, `DirectDeliveryRouter`, `ProphetRouter` (3)
- **Message TTL**: 60, 100, 140, 180, 220, 260, 300 minutes (7)
- **Buffer size** (ground nodes): 16, 28, 40, 52, 64 MB (5)
- **Mobility seeds**: 17, 57 (2)
- Total: 3 × 7 × 5 × 2 = **210 runs**

### Reports generated per run

1. `MessageStatsReport` — delivery ratio, overhead ratio, latency, hop count
2. `MessageGraphvizReport` — message-flow graph structure
3. `EnergyLevelReport` — per-second residual energy + movement-energy per host

---
  
