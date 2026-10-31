# Tail-sitting VTOL UAV Integration Report for Pushpak Vimana Framework

| Item | Detail |
|---|---|
| Project | Pushpak Vimana |
| Mission domain | Precision agriculture and rural healthcare |
| Operating region | Uttarakhand hills (Haldwani, Bageshwar) |
| Aircraft | Tail-sitting VTOL UAV, twin-motor flying wing |
| Autopilot | Pixhawk with ArduPilot (ArduPlane, QuadPlane tailsitter) |
| Status | Preliminary design. Figures are design estimates. Validate them with SITL, bench tests, and flight tests. |

## Contents

1. [Introduction](#1-introduction)
2. [Mission and Requirements](#2-mission-and-requirements)
3. [Configuration and Architecture](#3-configuration-and-architecture)
4. [Structure, Aerodynamics, and FEA/CFD Analysis](#4-structure-aerodynamics-and-feacfd-analysis)
5. [Propulsion and Simulation](#5-propulsion-and-simulation)
6. [Control, Testing, and Safety Protocols](#6-control-testing-and-safety-protocols)
7. [References](#7-references)

---

## 1. Introduction

Pushpak Vimana is a dual-purpose drone framework. It serves farms and rural clinics in the hills of Uttarakhand. Two swappable payload modules define it:

- Agri-spray module: targeted delivery of bio-inputs to terraced fields.
- Medical cold-chain module: temperature-controlled transport of vaccines, insulin, and diagnostic samples.

The terrain sets the design problem. Hill roads are slow and close during monsoon landslides. Villages sit on slopes. Flat launch sites are rare. A multirotor reaches these sites but drains its battery in cruise. A fixed-wing aircraft cruises efficiently but needs a runway or launcher. A tail-sitting VTOL removes both limits.

### How the tail-sitter complements the framework

| Framework need | Tail-sitter contribution |
|---|---|
| Small launch footprint | Takes off and lands vertically from a 3 m by 3 m pad. No runway. No launcher. |
| Range per battery | Wing-borne cruise needs about 80 percent less power than hover (310 W against 1,750 W). |
| Mechanical simplicity | Two motors provide lift in hover and thrust in cruise. No tilt mechanism. No idle lift rotors. |
| Modular payloads | One fuselage bay at the center of gravity carries either module. |
| Common avionics | Same Pixhawk, GPS, telemetry, and ground station as the base framework. |

### Known trade-offs

- Hover is sensitive to wind because the wing faces the airflow.
- Landing needs a slope of 5° or less.
- The payload bay turns 90° between hover and cruise. Each module must work in both orientations.

Sections 3 to 6 address each trade-off.

---

## 2. Mission and Requirements

### 2.1 Performance requirements

| Parameter | Requirement |
|---|---|
| Payload | 1.5 kg to 3.0 kg |
| Maximum takeoff mass (MTOW) | 10.5 kg |
| Empty mass (no payload) | 7.3 kg design, 7.5 kg limit |
| Mission radius | 20 km (40 km round trip) at 3.0 kg payload |
| Cruise speed | 18 m/s (65 km/h) |
| Stall speed | 12 m/s or lower at 2,000 m density |
| Endurance | 45 min or more at 3.0 kg. 60 min or more at 1.5 kg. |
| Cruise altitude | 120 m AGL or lower |
| Launch and landing altitude | Up to 2,500 m AMSL |
| Launch and landing wind | 8 m/s sustained, 10 m/s gusts |
| Cruise wind | Up to 12 m/s |
| Landing slope | 5° or less |
| Temperature range | -5 °C to 40 °C |
| Design air density | 1.0 kg/m³ (about 2,000 m ISA) |

### 2.2 Payload modules

| Module | Content | Payload mass |
|---|---|---|
| Agri-spray | Collapsible bladder tank, positive-displacement pump, downward nozzle. Doses spots in hover. | 1.5 L to 3.0 L |
| Medical cold-chain | Insulated box, phase-change packs, temperature logger. Holds 2 °C to 8 °C. | 1.5 kg to 3.0 kg including box |

Both modules use one quick-release mount. The mount carries power and a serial data link to the autopilot.

### 2.3 Autopilot and avionics

| Item | Specification |
|---|---|
| Flight controller | Pixhawk class (Cube Orange or Pixhawk 6X), ArduPlane 4.x |
| Vehicle type | QuadPlane tailsitter (`Q_ENABLE = 1`) |
| Sensors | Dual GNSS, redundant IMUs, barometer, compass, pitot airspeed sensor |
| Telemetry | 865 to 867 MHz link (license-free band in India). Optional LTE for BVLOS routes. |
| Terrain data | SRTM terrain database on the autopilot and ground station |
| Ground station | Mission Planner or QGroundControl, shared with the base framework |

### 2.4 Mass budget

| Item | Mass (kg) |
|---|---|
| Airframe and payload bay | 2.3 |
| Propulsion (2 motors, 2 ESCs, 2 propellers) | 1.7 |
| Battery (12S3P Li-ion, 21700 cells) | 2.6 |
| Avionics, servos, wiring | 0.7 |
| Empty mass | 7.3 |
| Payload (maximum) | 3.0 |
| Total | 10.3 |
| Margin | 0.2 |
| MTOW | 10.5 |

### 2.5 Energy budget (3.0 kg payload, 20 km radius)

| Phase | Power | Time | Energy |
|---|---|---|---|
| Hover (takeoff, transitions, landing) | 1,750 W | 4 min | 117 Wh |
| Cruise (40 km at 18 m/s) | 310 W | 37 min | 191 Wh |
| Avionics and payload systems | 65 W average | 41 min | 45 Wh |
| Total | | | 353 Wh |
| Usable battery (80 percent of 544 Wh) | | | 435 Wh |
| Reserve | | | 82 Wh (19 percent) |

The reserve equals 16 minutes of cruise or 2.8 minutes of hover.

Estimated endurance: 57 min at 3.0 kg and 71 min at 1.5 kg.

Method: hover power uses actuator disk theory with a figure of merit of 0.65 and a motor plus ESC efficiency of 0.85. Cruise power uses a lift-to-drag ratio of 9 and a propulsive efficiency of 0.60.

---

## 3. Configuration and Architecture

### 3.1 Why a tail-sitter

| Option | Hover | Cruise efficiency | Added mass and parts | Fit for hilly sites |
|---|---|---|---|---|
| Multirotor | Yes | Low | Low | Short range |
| Fixed-wing with launcher | No | High | Launcher and open landing area | Poor |
| Quad-plane VTOL | Yes | Medium | Four lift motors idle in cruise | Heavy, extra drag |
| Tail-sitter | Yes | High | Two motors serve both phases | Small pad, long range |

The tail-sitter combines vertical takeoff and landing agility with fixed-wing cruise. A typical sortie runs as follows:

1. Lift off vertically from a terrace or clinic yard.
2. Pitch forward and climb on the wing.
3. Cruise along a valley or ridge line.
4. Pitch up and hover down at the village pad.

### 3.2 Airframe layout

| Parameter | Value |
|---|---|
| Configuration | Tapered flying wing with central pod and winglets |
| Wingspan | 2.2 m |
| Mean chord | 0.6 m |
| Wing area | 1.32 m² |
| Aspect ratio | 3.7 |
| Leading-edge sweep | 20° |
| Airfoil | Reflexed, low pitching moment (MH45 class) |
| Wing loading at MTOW | 7.95 kg/m² |
| Motors | 2, mounted at 35 percent semi-span, leading edge |
| Control surfaces | 2 elevons, 28 percent chord, behind the propeller discs |
| Landing gear | 4 carbon legs, stance 0.9 m by 0.5 m |
| CG height on the pad | 0.35 m |

Calculated values at 2,000 m density:

- Cruise lift coefficient at 18 m/s: 0.48.
- Stall speed with a maximum lift coefficient of 1.1: 11.9 m/s.
- Static tip-over angle: 54° across the span and 35° fore-aft.

### 3.3 Control surface mechanics

The aircraft has no tail, rudder, or tilting parts. Two elevons and two motors control all three axes.

| Axis | Hover (wing vertical) | Cruise (wing horizontal) |
|---|---|---|
| Heading about the vertical axis | Differential elevon in propeller wash | Roll: differential elevon |
| Tilt about the span axis | Symmetric elevon in propeller wash | Pitch: symmetric elevon |
| Tilt across the span | Differential motor thrust | Yaw: differential motor thrust |
| Altitude and speed | Collective throttle | Collective throttle |

Key points:

- In hover, elevon authority comes from propeller wash. It scales with throttle, not airspeed.
- In cruise, elevon authority scales with dynamic pressure.
- Winglets add weak yaw stability. Differential thrust supplies yaw authority.
- Place the elevons directly behind the propeller discs. This raises hover authority.

ArduPilot output mapping:

```
SERVO1_FUNCTION   77    # Elevon left
SERVO2_FUNCTION   78    # Elevon right
SERVO3_FUNCTION   73    # Throttle left
SERVO4_FUNCTION   74    # Throttle right
```

### 3.4 Risks and mitigations

| Risk | Cause | Mitigation |
|---|---|---|
| Drift or tip-over in hover wind | Wing acts as a sail | Wind limit of 8 m/s. Weathervane the nose into the wind. Wide landing stance. |
| Tip-over on the ground | Broadside wind | Shelter or tie down above 8 m/s. |
| Stall during transition | High angle of attack | Fast pitch-over. Propeller wash adds lift. Altitude buffer of 40 m. |
| Slope landing damage | Tall, narrow stance | 5° slope limit. Level the pad. |
| One motor failure in hover | Twin-motor layout has no hover redundancy | Parachute recovery. See Section 6.3. |

### 3.5 Payload orientation

The fuselage axis is vertical in hover and horizontal in cruise.

- Agri-spray: use a collapsible bladder and a positive-displacement pump. Both work in any orientation.
- Medical cold-chain: strap the insulated box to the mount. Position phase-change packs on all sides. Log temperature for the full sortie.
- Keep the CG within 10 mm of the design point. Slide the battery on its rail to trim CG for each payload.

---

## 4. Structure, Aerodynamics, and FEA/CFD Analysis

### 4.1 Airframe materials

| Component | Material | Reason |
|---|---|---|
| Wing skin | 3K twill carbon prepreg, 2 plies over a 5 mm PVC foam core | High stiffness at low mass |
| Main spar | Unidirectional carbon spar caps with a shear web | Bending strength |
| Pod | Carbon monocoque with glass-fiber panels at antenna zones | Carbon blocks radio signals. Glass lets GNSS and telemetry through. |
| Motor mounts | 7075-T6 aluminum with carbon plate | Thrust and torque loads |
| Landing legs | 12 mm carbon tubes with TPU feet | Light, energy absorbing |
| Payload mount | Carbon plate with aluminum latch | Quick swap, secure hold |

Use coupon-tested allowables for the final layup. Do not rely on catalog values.

### 4.2 Structural FEA

Model setup:

- Shell elements for skins and spar webs. Solid elements for motor mounts and latches.
- Layered composite properties by ply.
- Tools: ANSYS Mechanical with ACP, Abaqus, or CalculiX with Gmsh.
- Mesh study: peak stress changes by less than 5 percent between two refinements.

Load cases:

| Case | Condition | Factor |
|---|---|---|
| LC1 | Cruise maneuver, +3.5 g and -1.5 g | 1.5 ultimate |
| LC2 | 10 m/s design gust at cruise speed | 1.5 ultimate |
| LC3 | Hover at maximum thrust (113 N per motor) | 1.5 ultimate |
| LC4 | One motor at full thrust, one idle | 1.5 ultimate |
| LC5 | Landing at 2 m/s sink rate on the four legs | 1.5 ultimate |
| LC6 | Ground wind at 12 m/s, broadside (about 105 N) | Check tip-over and leg loads |

Acceptance criteria:

| Check | Criterion |
|---|---|
| Ply failure | Tsai-Wu or Hashin index below 1.0 at ultimate load |
| Margin of safety | Zero or greater at every load case |
| Wingtip deflection | 10 percent of semi-span or less at limit load |
| Buckling | Load factor of 1.0 or more at ultimate |
| Vibration | Run a Campbell diagram. Keep modes clear of motor 1P and 2P at hover and cruise rpm. |

Ground wind check at 12 m/s, ρ = 1.0 kg/m³:

- Drag = 0.5 × 1.0 × 12² × 1.32 × 1.1 = 105 N.
- Overturning moment = 105 N × 0.30 m = 31 N·m.
- Restoring moment = 103 N × 0.25 m = 26 N·m.
- Result: the aircraft tips at 12 m/s. At 8 m/s the overturning moment is 14 N·m. This sets the 8 m/s ground limit.

### 4.3 Aerodynamics and CFD

Flow conditions:

- Reynolds number: Re = ρVc/μ = (1.0 × 18 × 0.6) / (1.8 × 10⁻⁵) = 6 × 10⁵.
- Low Reynolds number brings laminar separation bubbles. Model transition.

CFD setup:

- Solver: OpenFOAM or ANSYS Fluent.
- 3D RANS with k-omega SST and a transition model (γ-Reθ).
- Wall resolution: y+ near 1.
- Domain: 20 chord lengths in every direction.
- Propellers: actuator disk models.

CFD cases:

| Case | Sweep | Output |
|---|---|---|
| Cruise polar | Angle of attack -2° to 12°, propeller on and off | CL, CD, Cm, L/D |
| Stall | Angle of attack to 18° | Tip stall onset. Elevon effectiveness after stall. |
| Crosswind | Sideslip -15° to +15° at 12 m/s | Yaw moment, winglet effect |
| Hover wind | Wing broadside, 0 to 12 m/s, propeller wash on | Drag, tilt moment, elevon authority |
| Transition | Pitch from vertical at 90°, 75°, 60°, 45°, 30°, 15°, 0° | Lift, drag, moment at each step |

### 4.4 High wind resistance

| Feature | Effect |
|---|---|
| Thrust margin of 2.2:1 | The airframe holds altitude at tilt angles up to 63° (cos θ = 1/2.2). |
| Hover at 8 m/s wind | Wind drag of 46 N needs a 24° tilt and 113 N of thrust. This equals 1.1 times hover thrust. |
| Weathervane control | The nose turns into the wind and the wing faces the airflow edge-on. |
| Low-profile pod | Less frontal area in crosswind. |
| Washout at the wingtip | Keeps elevons effective near stall. |
| Reflexed airfoil | Trims without a tail. Keeps drag low. |

Wing area drives hover wind loads. A smaller wing lowers hover drag and raises stall speed. The 1.32 m² area balances both.

---

## 5. Propulsion and Simulation

### 5.1 Thrust-to-weight sizing

Requirement: static thrust-to-weight ratio above 2:1.

| Step | Result |
|---|---|
| MTOW | 10.5 kg (103 N) |
| Minimum total static thrust at 2:1 | 21 kg (206 N) |
| Design total static thrust (2.2:1) | 23 kg (226 N) |
| Design thrust per motor | 11.5 kg (113 N) |
| Hover thrust per motor | 5.25 kg (51.5 N) |
| Hover throttle | About 65 to 70 percent |
| Thrust headroom in hover | 2.2 times |

### 5.2 Motor, ESC, propeller, and battery

| Component | Specification |
|---|---|
| Motors | 2 brushless DC outrunners, 110 to 130 KV, 12S, 3.2 kW peak each |
| ESCs | 2 × 12S, 120 A peak, with telemetry (DShot or DroneCAN) |
| Propellers | 24 × 11 in carbon fixed pitch, 2 blades |
| Battery | 12S3P Li-ion, 21700 high-drain cells, 12.6 Ah, 544 Wh |
| Battery burst rating | 150 A or more for 10 s |
| Full-thrust current | About 135 A at 46 V |

Propeller check (first-order estimate with static CT ≈ 0.095 and CP ≈ 0.04):

| Quantity | Hover | Full thrust |
|---|---|---|
| Thrust per motor | 51.5 N | 113 N |
| Speed | 3,750 rpm | 5,560 rpm |
| Tip speed | 120 m/s | 178 m/s (Mach 0.52) |
| Shaft power per motor | About 0.8 kW | About 2.7 kW |

Propeller pitch compromise:

- Cruise needs a pitch speed above 18 m/s. An 11 in pitch at 4,200 rpm gives 19.6 m/s.
- A lower pitch raises static thrust and lowers cruise efficiency.
- Confirm the final choice with manufacturer thrust tables and a bench test.

Select cells with a burst rating of 50 A or more. A semi-solid-state LiPo pack is an alternative if the Li-ion pack fails the burst test.

### 5.3 SITL framework

Tools:

| Tool | Role |
|---|---|
| ArduPilot SITL | Runs the flight code on a PC |
| MAVProxy | Console, map, and scripting |
| Mission Planner or QGroundControl | Mission planning and monitoring |
| pymavlink | Scripted test automation |
| MAVExplorer or Mission Planner log review | Log analysis |

Setup (Ubuntu):

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile

# List the available frames and select the tailsitter model
sim_vehicle.py --help

# Start the simulation with the project parameter file
sim_vehicle.py -v ArduPlane -f quadplane-tailsitter --console --map \
  --add-param-file=pushpak_vtol.parm
```

Starting parameter file `pushpak_vtol.parm` (values for SITL tuning, not flight-ready):

```
Q_ENABLE            1
Q_FRAME_CLASS       10
Q_TAILSIT_ENABLE    1
Q_TAILSIT_ANGLE     60
Q_M_THST_HOVER      0.65
Q_M_BAT_VOLT_MAX    50.4
Q_M_BAT_VOLT_MIN    39.6
Q_LAND_FINAL_SPD    50
TRIM_ARSPD_CM       1800
ARSPD_FBW_MIN       14
ARSPD_FBW_MAX       24
TERRAIN_ENABLE      1
```

Parameter names follow ArduPlane 4.x. Check each name against the firmware version you flash.

SITL test matrix:

| Test | Setup | Pass criterion |
|---|---|---|
| Hover hold | Wind 0, 5, 8 m/s (`SIM_WIND_SPD`, `SIM_WIND_DIR`) | Position error 1 m or less. Altitude error 0.5 m or less. |
| Forward transition | Start at 40 m AGL | Airspeed above 15 m/s within 10 s. Altitude change within 8 m. |
| Back transition and landing | 5 m/s wind | Touchdown within 2 m of target. Sink rate 1 m/s or less. |
| Turbulence | `SIM_WIND_TURB` 2 to 5 | No loss of control. Attitude error below 15°. |
| Single motor loss in cruise | Fail one motor output | Aircraft holds level flight and reaches a safe area. |
| Low battery | Lower `SIM_BATT_VOLTAGE` | RTL at the low threshold. Land at the critical threshold. |
| GPS loss | `SIM_GPS_DISABLE` = 1 | Failsafe action runs as designed. |
| RC and GCS loss | `SIM_RC_FAIL` = 1, stop telemetry | RTL then landing at home. |
| Geofence breach | Fly outside the polygon | RTL starts within 2 s. |
| Monte Carlo | Mass ±10 percent, CG ±15 mm, thrust ±10 percent, wind direction 0° to 360° | 95 percent of runs pass every criterion above. |

Test progression after SITL:

1. Bench test with propellers off.
2. Tethered hover.
3. Free hover at 2 m, then 10 m.
4. Hover with a dummy payload mass.
5. Transition at altitude.
6. Cruise loop.
7. Full mission with a dummy payload.
8. Live agri-spray and medical modules.

---

## 6. Control, Testing, and Safety Protocols

### 6.1 Gain scheduling for hover to cruise transition

The control surfaces work on two different physics. In hover, propeller wash drives authority. In cruise, dynamic pressure drives authority. The controller schedules gains on airspeed and throttle.

| Regime | Airspeed | Attitude control | Gain basis |
|---|---|---|---|
| Hover | 0 to 6 m/s | Multicopter-style attitude and rate loops | Throttle (propeller wash) |
| Transition | 6 to 15 m/s | Blend of both loops | Blend weight w(V) |
| Cruise | 15 m/s and above | Fixed-wing loops with TECS and L1 | Dynamic pressure |

Blend weight:

```
w(V) = smoothstep((V - V1) / (V2 - V1))     V1 = 6 m/s, V2 = 15 m/s
command = (1 - w) × hover_command + w × cruise_command
```

Cruise gain scaling:

```
K_cruise(V) = K_ref × (V_ref / V)²           V_ref = 18 m/s
```

Clamp the scale factor between 0.5 and 2.5. This keeps the loops stable at low speed.

Hover elevon gain scaling:

```
K_hover(throttle) = K_ref × (throttle_hover / throttle)
```

In ArduPilot, tune the gain scaling with `Q_TAILSIT_GSCMSK` and `Q_TAILSIT_GSCMIN`. Set the transition trigger angle with `Q_TAILSIT_ANGLE`.

Forward transition sequence:

1. Climb in hover to 40 m AGL. This gives an altitude buffer.
2. Pitch forward at 45°/s or less. Boost collective throttle to hold altitude.
3. Accelerate from 8 to 15 m/s. The controller blends from hover to cruise loops.
4. Settle at 18 m/s in cruise.
5. Abort to hover if airspeed stays below 12 m/s after 10 s.

Back transition sequence:

1. Cut throttle and pitch up with the elevons.
2. Let airspeed fall below 12 m/s.
3. Switch to the hover loops at the trigger angle.
4. Hover and descend to the pad.

Tune hover loops first, then cruise loops, then the transition. Review logs after every flight.

### 6.2 Pre-flight checks and calibrations

Site checks:

- [ ] Wind at 8 m/s or less, gusts at 10 m/s or less.
- [ ] No rain. No fog below the route altitude.
- [ ] Pad slope at 5° or less. Clear radius of 10 m.
- [ ] Airspace and permission check complete.
- [ ] Route altitude checked against terrain data. AGL stays at 120 m or less.

Airframe checks:

- [ ] Inspect wing skin, spar joints, legs, and hinges.
- [ ] Check elevon direction and travel against the ground station display.
- [ ] Check motor direction and propeller orientation.
- [ ] Tighten propeller nuts. Inspect blades for cracks.
- [ ] Balance the aircraft at the marked CG.

Sensor calibration:

| Sensor | Procedure | Frequency |
|---|---|---|
| Accelerometer | Six-position calibration | New airframe and after hard landing |
| Compass | Outdoors, away from metal and vehicles | Each new site |
| Compass-motor interference | Run CompassMot with props off and a secure airframe | After wiring changes |
| Gyro | Keep the aircraft still at power-up | Every flight |
| Airspeed | Cover the pitot tube, zero the offset | Every flight |
| Barometer | Set ground pressure at the site | Every flight |
| ESC | Throttle range calibration | After ESC change |
| RC | Stick range and failsafe values | Monthly |

Power and navigation checks:

- [ ] Battery cell voltage difference at 30 mV or less.
- [ ] Battery at full charge.
- [ ] GNSS with 12 satellites or more. HDOP below 1.2.
- [ ] EKF status green. Home position set.
- [ ] Vibration (VIBE) below 30 m/s² in a hover test.
- [ ] Geofence and failsafe parameters loaded.
- [ ] Telemetry link verified at full range of the sortie.

Payload checks:

- [ ] Payload mass entered in the ground station.
- [ ] CG checked with the payload fitted.
- [ ] Latch locked.
- [ ] Agri-spray: bladder sealed, pump primed, no leaks.
- [ ] Medical: box sealed, phase-change packs conditioned, temperature logger running.

Set `ARMING_CHECK` to all checks. Fix every pre-arm warning before flight.

### 6.3 Emergency fail-safes

| Trigger | Action | Starting settings |
|---|---|---|
| RC loss, short | Continue the mission in AUTO or hold position in hover | `FS_SHORT_ACTN`, `FS_SHORT_TIMEOUT` = 1.5 s |
| RC loss, long | RTL, then quadplane landing at home | `FS_LONG_ACTN`, `FS_LONG_TIMEOUT` = 5 s, `Q_RTL_MODE` = 1 |
| GCS link loss | RTL | `FS_GCS_ENABL` |
| Low battery | Warn, then RTL when remaining energy drops to return energy plus 20 percent | `BATT_LOW_VOLT` = 42.0 V (3.5 V per cell), `BATT_FS_LOW_ACT` = RTL |
| Critical battery | Land at once | `BATT_CRT_VOLT` = 39.6 V (3.3 V per cell), `BATT_FS_CRT_ACT` = land |
| Geofence breach | RTL | `FENCE_ENABLE` = 1, `FENCE_ACTION` = RTL |
| GPS or EKF failure in hover | Land straight down | EKF failsafe set to QLAND |
| GPS or EKF failure in cruise | Switch to FBWA. Steer by the ground station. | `FS_EKF_ACTION` |
| Motor or ESC failure in cruise | Fly on one motor to the nearest safe area. Trigger parachute below the safe-flight limit. | Parachute settings below |
| Loss of control or motor failure in hover | Deploy parachute | `CHUTE_ENABLED` = 1, `CHUTE_ALT_MIN` = 40 m |

Geofence design:

- Polygon fence around the sortie corridor.
- Circle fence of 25 km around home.
- Altitude limit set from terrain data, not from home altitude. Valley terrain changes by hundreds of meters.
- RTL altitude set per route, above the highest terrain on the route.

A parachute recovery system lowers ground risk over villages. Hover with one motor is not possible on a twin-motor tail-sitter.

Compliance:

- The aircraft falls in the small category (2 kg to 25 kg) of the Drone Rules, 2021.
- Hold a remote pilot certificate.
- Obtain approval for flights beyond visual line of sight.

### 6.4 Post-flight review

- Motor output balance within 10 percent.
- Vibration below 30 m/s².
- Battery sag within the plan.
- EKF innovations in range.
- Transition altitude change within 8 m.
- Landing position error within 2 m.
- Update the logbook. Record every anomaly before the next flight.

---

## 7. References

- ArduPilot Plane documentation: QuadPlane and Tailsitter pages (ardupilot.org/plane).
- ArduPilot SITL documentation (ardupilot.org/dev).
- Drone Rules, 2021, Ministry of Civil Aviation, Government of India.
- Standard Atmosphere (ISA) tables for density at altitude.
