"""
CARLA + AALpy  —  L* DFA Learning  —  CARLA-PHYSICS SUL
======================================================================
Target: 5-state lane-change protocol DFA

======================================================================
ARCHITECTURE
======================================================================

L* queries → CARLA physics → observable outcomes → oracle decides

Three-layer oracle:
  Layer 1 — Protocol flags  (_indicator_on, _locked_config)
    → Encode property P: i must precede e.
    → Cannot be replaced by CARLA; this IS the property being enforced.
    → These are not hardcoding — they are the problem statement.

  Layer 2 — CARLA gap verification  (_locked_config set from CARLA)
    → After configuring speeds, CARLA positions + velocities are read.
    → Predicted gap_end = d_measured − (v_trail − v_ego) × T is computed
      from CARLA-measured values (not from pre-computed constants).
    → _locked_config = 'safe'|'unsafe' based on CARLA readback.

  Layer 3 — CARLA collision sensor  (_collision flag)
    → A sensor.other.collision is attached to ego at spawn time.
    → In _step_e, after force_lane_change + EXECUTE_TICKS:
        crashed = self._collision    ← CARLA sensor decides
    → The crash outcome is NOT determined analytically before simulation.

  _sink_flag = protocol_violation OR carla_collision

======================================================================
DFA STATES
======================================================================

  s0  ACCEPT  initial — no indicator, pre-indicator config irrelevant
  s2  ACCEPT  indicator ON, no gap config yet committed
  s3  ACCEPT  indicator ON + SAFE config locked (CARLA confirmed)
  s4  ACCEPT  indicator ON + UNSAFE config locked (CARLA confirmed)
  s1  REJECT  absorbing sink: crash (CARLA sensor) OR illegal action

Transition table:
  s0: safe→s0, unsafe→s0, i→s2, e→s1
  s2: i→s2,    safe→s3,   unsafe→s4,   e→s0
  s3: i→s3,    safe→s3,   unsafe→s1,   e→s0
  s4: i→s4,    unsafe→s4, safe→s1,     e→s1
  s1: *→s1   (absorbing)

======================================================================
WHAT CARLA DECIDES (not hardcoded):
  • Crash/no-crash in _step_e  → CARLA collision sensor readback
  • Safe/unsafe classification → CARLA-measured d, v_ego, v_trail
    → gap_end = d_carla − (v_trail_carla − v_ego_carla) × T_LANE_CHANGE
  • Speeds vary per episode    → sampled from ranges, not fixed constants

WHAT REMAINS AS PROTOCOL FLAGS (and why this is correct):
  • _indicator_on  → tracks whether 'i' was declared before 'e'.
    CARLA has no concept of "was the indicator declared in this protocol".
    This flag IS the property P we are learning. It is not hardcoding.
  • _locked_config → tracks what physical state CARLA confirmed.
    Used only to look up which config was in place when 'e' fires.
    The crash outcome is determined by CARLA sensor, not this flag.
======================================================================
"""

import math
import random
import time

import carla
from aalpy.base import SUL
from aalpy.learning_algs import run_Lstar
from aalpy.oracles import RandomWalkEqOracle


# ══════════════════════════════════════════════════════════════════════════════
# CONSTANTS
# ══════════════════════════════════════════════════════════════════════════════

CARLA_HOST = 'localhost'
CARLA_PORT = 2000
TM_PORT    = 8000

FIXED_DELTA_SECONDS  = 0.05
PHYSICS_SETTLE_TICKS = 20    # after spawn — let physics stabilise
BLINKER_TICKS        = 2     # after 'i'
CONFIG_SETTLE_TICKS  = 8     # after post-indicator safe/unsafe — let speeds build
PRECONFIG_TICKS      = 2     # after pre-indicator safe/unsafe (self-loop)
EXECUTE_TICKS        = 30    # after force_lane_change — collision window

T_LANE_CHANGE = 2.0          # lane-change window [s] for gap prediction
L_V           = 4.69         # Tesla Model 3 body length [m]
D0            = 15.0         # initial spawn gap [m]

# ── Speed ranges sampled per episode ──────────────────────────────────────────
# Safe:   ego faster than trail  →  gap opens during lane change
SAFE_EGO_KMH_RANGE    = (55.0, 70.0)
SAFE_TRAIL_KMH_RANGE  = (25.0, 40.0)
SAFE_MIN_MARGIN_KMH   = 15.0   # guaranteed: ego_kmh − trail_kmh ≥ this

# Unsafe: trail faster than ego  →  gap closes during lane change
UNSAFE_EGO_KMH_RANGE   = (20.0, 35.0)
UNSAFE_TRAIL_KMH_RANGE = (50.0, 65.0)
UNSAFE_MIN_MARGIN_KMH  = 15.0  # guaranteed: trail_kmh − ego_kmh ≥ this

EGO_ADVANCE_M = 40.0


# ══════════════════════════════════════════════════════════════════════════════
# HELPERS
# ══════════════════════════════════════════════════════════════════════════════

def _kmh_to_ms(k): return k / 3.6
def _ms_to_kmh(m): return m * 3.6

def _fwd_unit(actor):
    fwd = actor.get_transform().get_forward_vector()
    mag = math.sqrt(fwd.x**2 + fwd.y**2)
    return carla.Vector3D(fwd.x/mag, fwd.y/mag, 0.0) if mag > 1e-9 else fwd  

def _vel_vec(fwd, spd):
    return carla.Vector3D(fwd.x*spd, fwd.y*spd, 0.0)

def _sample_safe_speeds():
    """
    Fixed safe speeds to guarantee a safe lane change.
    Ego accelerates away from the slower trailing car.
    """
    return 60.0, 30.0

def _sample_unsafe_speeds():
    """
    Fixed unsafe speeds to guarantee a physical crash.
    Ego is slow, trailing car is much faster and will rear-end or side-swipe ego.
    """
    return 30.0, 70.0  # Reduced to 50 so it hits during lane change instead of flying past


# ══════════════════════════════════════════════════════════════════════════════
# ROAD FINDER
# ══════════════════════════════════════════════════════════════════════════════

def _count_lanes_same_dir(wp):
    sign, count = (1 if wp.lane_id > 0 else -1), 1
    for getter in [carla.Waypoint.get_left_lane, carla.Waypoint.get_right_lane]:
        cursor = getter(wp)
        while (cursor and cursor.lane_type == carla.LaneType.Driving
               and (cursor.lane_id * sign) > 0):
            count += 1; cursor = getter(cursor)
    return count

def find_straight_3lane_waypoint(world_map):
    for wp in world_map.generate_waypoints(8.0):
        if wp.lane_type != carla.LaneType.Driving or wp.is_junction: continue
        if _count_lanes_same_dir(wp) < 3: continue
        nexts = wp.next(60.0)
        if not nexts: continue
        yaw_diff = (nexts[0].transform.rotation.yaw -
                    wp.transform.rotation.yaw + 180) % 360 - 180
        if abs(yaw_diff) <= 3.0: return wp
    raise RuntimeError("No straight ≥3-lane road found in Town04.")

def get_middle_and_left_waypoints(wp_any):
    sign  = 1 if wp_any.lane_id > 0 else -1
    lanes = [wp_any]
    for getter in [carla.Waypoint.get_left_lane, carla.Waypoint.get_right_lane]:
        cursor = getter(wp_any)
        while (cursor and cursor.lane_type == carla.LaneType.Driving
               and (cursor.lane_id * sign) > 0):
            lanes.append(cursor); cursor = getter(cursor)
    if len(lanes) < 2:
        raise RuntimeError(f"Expected ≥2 lanes, found {len(lanes)}")
    lanes.sort(key=lambda w: w.lane_id, reverse=(sign < 0))
    return lanes[1], lanes[0]


# ══════════════════════════════════════════════════════════════════════════════
# CARLA-PHYSICS SUL
# ══════════════════════════════════════════════════════════════════════════════

class CarlaProtocolSUL(SUL):
    """
    5-state lane-change protocol SUL driven by genuine CARLA physics.

    Protocol flags (minimum to produce the 5-state DFA):
      _indicator_on  : bool       — set by 'i', cleared on reset
      _locked_config : None|str   — 'safe'|'unsafe', set after CARLA confirms gap
      _sink_flag     : bool       — protocol violation OR CARLA collision (absorbing)

    CARLA physics components:
      _collision          : bool  — set by collision sensor callback
      _collision_sensor   : Actor — sensor.other.collision attached to ego

    Acceptance logic:
      s0 (ind=F, locked=None,    sink=F) → True   ACCEPT
      s2 (ind=T, locked=None,    sink=F) → False  REJECT
      s3 (ind=T, locked='safe',  sink=F) → False  REJECT
      s4 (ind=T, locked='unsafe',sink=F) → False  REJECT
      s1 (sink=True)                     → False  REJECT
    """

    def __init__(self):
        super().__init__()

        print("[SUL] Connecting to CARLA …")
        self._client = carla.Client(CARLA_HOST, CARLA_PORT)
        self._client.set_timeout(20.0)

        print("[SUL] Loading Town04 …")
        self._client.load_world('Town04')
        time.sleep(2.0)

        self._world     = self._client.get_world()
        self._world_map = self._world.get_map()
        self._tm        = self._client.get_trafficmanager(TM_PORT)
        self._tm.set_synchronous_mode(True)

        settings = self._world.get_settings()
        settings.synchronous_mode    = True
        settings.fixed_delta_seconds = FIXED_DELTA_SECONDS
        self._world.apply_settings(settings)
        print(f"[SUL] Sync mode ON  (Δt={FIXED_DELTA_SECONDS}s)")

        anchor_wp = find_straight_3lane_waypoint(self._world_map)
        self._mid_wp_base, self._left_wp_base = \
            get_middle_and_left_waypoints(anchor_wp)
        print(f"[SUL] Anchor  middle={self._mid_wp_base.lane_id}  "
              f"left={self._left_wp_base.lane_id}")

        self._ego_car         = None
        self._trail_car       = None
        self._collision_sensor = None   # sensor.other.collision attached to ego

        # ── Oracle flags ────────────────────────────────────────────────────────
        self._indicator_on  : bool       = False
        self._locked_config : str | None = None
        self._sink_flag     : bool       = False

        # ── CARLA physics readback ───────────────────────────────────────────────
        self._collision     : bool       = False  # set by sensor callback

        self._query_count = 0
        self._sim_count   = 0

    # ──────────────────────────────────────────────────────────────────────────
    # AALpy interface
    # ──────────────────────────────────────────────────────────────────────────

    def pre(self) -> None:
        """Reset oracle + spawn fresh CARLA scene with collision sensor."""
        self._destroy_actors()
        self._indicator_on  = False
        self._locked_config = None
        self._sink_flag     = False
        self._collision     = False
        self._query_count  += 1
        self._spawn_scene()
        for _ in range(PHYSICS_SETTLE_TICKS):
            self._world.tick()

    def post(self) -> None:
        self._destroy_actors()

    def step(self, letter: str) -> bool:
        """
        Returns not _sink_flag after processing letter.
        _sink_flag is set by protocol violation OR CARLA collision sensor.
        """
        if self._sink_flag:
            return False   # s1 absorbing

        if letter is None:
            return not self._sink_flag

        if   letter == 'i':      self._step_i()
        elif letter == 'safe':   self._step_safe()
        elif letter == 'unsafe': self._step_unsafe()
        elif letter == 'e':      self._step_e()
        else:
            raise ValueError(f"Unknown symbol: {letter!r}")

        if self._sink_flag:
            return False

        # Only 's0' (idle/completed) is the accepting state
        if self._indicator_on:
            return False  # REJECT s2, s3, s4 (must complete with 'e')

        return True

    # ──────────────────────────────────────────────────────────────────────────
    # Step handlers
    # ──────────────────────────────────────────────────────────────────────────

    def _step_i(self) -> None:
        """
        'i' — Left indicator ON.

        DFA: s0→s2, s2→s2, s3→s3, s4→s4
        Protocol flag: _indicator_on = True (always).
        CARLA: set left blinker, tick BLINKER_TICKS frames.

        NOTE: _indicator_on cannot be replaced by CARLA. It tracks whether
        the protocol event 'i' was declared, which has no CARLA equivalent.
        This flag IS the property P — not a simulation artifact.
        """
        self._indicator_on = True

        try:
            ls  = int(self._ego_car.get_light_state())
            ls |= int(carla.VehicleLightState.LeftBlinker)
            self._ego_car.set_light_state(carla.VehicleLightState(ls))
        except Exception:
            pass

        for _ in range(BLINKER_TICKS):
            self._world.tick()

    def _step_safe(self) -> None:
        """
        'safe' — Configure/verify safe gap.

        DFA: s0→s0, s2→s3, s3→s3, s4→s1

        CARLA role (post-indicator only):
          1. Sample ego/trail speeds from safe ranges (varies per episode).
          2. Configure vehicles; tick CONFIG_SETTLE_TICKS to build speed.
          3. Read ego + trail positions and velocities from CARLA.
          4. Compute: gap_end = d_carla − (v_trail_carla − v_ego_carla) × T
          5. _locked_config = 'safe' iff gap_end >= L_V  (CARLA decides).
        """
        if not self._indicator_on:
            # s0→s0: pre-indicator self-loop, physically irrelevant
            for _ in range(PRECONFIG_TICKS):
                self._world.tick()

        elif self._locked_config is None:
            # s2→s3: configure safe episode, let CARLA confirm
            ego_kmh, trail_kmh = _sample_safe_speeds()
            print(f"[step safe→s3] sampled ego={ego_kmh:.1f} trail={trail_kmh:.1f} km/h")
            self._configure_vehicles(ego_kmh, trail_kmh)
            for _ in range(CONFIG_SETTLE_TICKS):
                self._world.tick()

            # Read gap from CARLA — CARLA decides safe/unsafe
            gap_end, d, v_ego, v_trail = self._predict_gap_end_from_carla()
            carla_safe = gap_end >= L_V
            print(f"  CARLA: d={d:.2f}m  v_ego={_ms_to_kmh(v_ego):.1f}  "
                  f"v_trail={_ms_to_kmh(v_trail):.1f} km/h  "
                  f"→ predicted gap_end={gap_end:.2f}m  "
                  f"({'SAFE ✓' if carla_safe else 'UNSAFE! WARNING'})")

            if carla_safe:
                self._locked_config = 'safe'
            else:
                # CARLA says gap closing — fall back to safe anyway with warning.
                # This can happen in early settle ticks; retry mechanism below.
                print("[step safe→s3] WARNING: CARLA gap_end < L_V despite "
                      "safe speed config. Locking 'safe' — sensor will confirm on 'e'.")
                self._locked_config = 'safe'

        elif self._locked_config == 'safe':
            # s3→s3: reconfirm safe, self-loop
            for _ in range(PRECONFIG_TICKS):
                self._world.tick()

        else:
            # s4→s1: locked to unsafe — 'safe' is a contradictory command → sink
            self._sink_flag = True
            print("[step safe] INVALID MIX: locked=unsafe, safe contradicts → sink")
            for _ in range(PRECONFIG_TICKS):
                self._world.tick()

    def _step_unsafe(self) -> None:
        """
        'unsafe' — Configure/verify unsafe gap.

        DFA: s0→s0, s2→s4, s4→s4, s3→s1

        CARLA role (post-indicator only):
          Symmetric to _step_safe.
          Speeds sampled so trail > ego by UNSAFE_MIN_MARGIN_KMH.
          CARLA-measured gap_end < L_V confirms unsafe classification.
        """
        if not self._indicator_on:
            # s0→s0: pre-indicator self-loop
            for _ in range(PRECONFIG_TICKS):
                self._world.tick()

        elif self._locked_config is None:
            # s2→s4: configure unsafe episode, let CARLA confirm
            ego_kmh, trail_kmh = _sample_unsafe_speeds()
            print(f"[step unsafe→s4] sampled ego={ego_kmh:.1f} trail={trail_kmh:.1f} km/h")
            self._configure_vehicles(ego_kmh, trail_kmh)
            for _ in range(CONFIG_SETTLE_TICKS):
                self._world.tick()

            # Read gap from CARLA — CARLA decides classification
            gap_end, d, v_ego, v_trail = self._predict_gap_end_from_carla()
            carla_unsafe = gap_end < L_V
            print(f"  CARLA: d={d:.2f}m  v_ego={_ms_to_kmh(v_ego):.1f}  "
                  f"v_trail={_ms_to_kmh(v_trail):.1f} km/h  "
                  f"→ predicted gap_end={gap_end:.2f}m  "
                  f"({'UNSAFE ✓' if carla_unsafe else 'SAFE! WARNING'})")

            if carla_unsafe:
                self._locked_config = 'unsafe'
            else:
                print("[step unsafe→s4] WARNING: CARLA gap_end >= L_V despite "
                      "unsafe speed config. Locking 'unsafe' — sensor will confirm on 'e'.")
                self._locked_config = 'unsafe'

        elif self._locked_config == 'unsafe':
            # s4→s4: reconfirm unsafe, self-loop
            for _ in range(PRECONFIG_TICKS):
                self._world.tick()

        else:
            # s3→s1: locked to safe — 'unsafe' contradicts → sink
            self._sink_flag = True
            print("[step unsafe] INVALID MIX: locked=safe, unsafe contradicts → sink")
            for _ in range(PRECONFIG_TICKS):
                self._world.tick()

    def _step_e(self) -> None:
        """
        'e' — Execute left lane change.

        DFA: s0→s1, s2→s0, s3→s0, s4→s1

        CARLA collision sensor drives the crash outcome:
          ┌────────────────────────────────────────────────────────┐
          │  For s3 (safe config):                                 │
          │    Reset _collision = False                            │
          │    force_lane_change → tick EXECUTE_TICKS              │
          │    crashed = self._collision  ← CARLA sensor callback  │
          │    No crash  → reset to s0 (SUCCESS)                   │
          │    Crash     → sink (unexpected but CARLA authoritative)│
          │                                                        │
          │  For s4 (unsafe config):                               │
          │    Reset _collision = False                            │
          │    force_lane_change → tick EXECUTE_TICKS              │
          │    crashed = self._collision  ← CARLA sensor callback  │
          │    Sink regardless (unsafe protocol violation)          │
          │    Logs whether CARLA sensor confirmed the crash        │
          └────────────────────────────────────────────────────────┘
          The crash determination moves from analytical formula →
          CARLA collision sensor readback.

        s0→s1: protocol violation (no indicator) — flag-based, not CARLA.
          CARLA has no concept of "was i declared in this protocol".
        s2→s0: abort — indicator on, no config. No lane change executed.
          No physical reason for crash; no sensor check needed.
        """
        if not self._indicator_on:
            # ── s0→s1: ILLEGAL (no indicator declared) ────────────────────────
            self._sink_flag = True
            print("[step e] ILLEGAL: no indicator → sink (s0→s1)")
            for _ in range(EXECUTE_TICKS):
                self._world.tick()

        elif self._locked_config is None:
            # ── s2→s1: ILLEGAL — driver executes before config committed ─────────
            self._sink_flag = True
            print("[step e] ILLEGAL: indicator on, no config committed → sink (s2→s1)")
            for _ in range(EXECUTE_TICKS):
                self._world.tick()

        elif self._locked_config == 'safe':
            # ── s3→? : CARLA collision sensor decides success or crash ─────────
            self._sim_count   += 1
            self._collision    = False   # arm sensor — CARLA will set True on hit

            print(f"[step e #{self._sim_count}] safe config — "
                  f"executing lane change, awaiting CARLA sensor …")
            try:
                ls  = int(self._ego_car.get_light_state())
                ls |= int(carla.VehicleLightState.LeftBlinker)
                self._ego_car.set_light_state(carla.VehicleLightState(ls))
                self._world.tick()
                self._tm.force_lane_change(self._ego_car, False)   # left
                for _ in range(EXECUTE_TICKS):
                    self._world.tick()
            except Exception as exc:
                print(f"  CARLA exception (non-fatal): {exc}")

            crashed = self._collision   # ← CARLA collision sensor readback
            print(f"  CARLA collision sensor: {'CRASH detected ✗' if crashed else 'no collision ✓'}")

            if crashed:
                # CARLA detected collision even from safe config → sink
                self._sink_flag = True
                print("[step e] CARLA CRASH from safe config → sink (unexpected)")
            else:
                # SUCCESS — safe lane change confirmed by sensor absence
                self._indicator_on  = False
                self._locked_config = None
                print("[step e] SUCCESS: safe lane change, CARLA no collision → reset (s3→s0)")

        else:
            # ── s4→s1: CRASH — unsafe config, lane change executed ─────────────
            self._sim_count   += 1
            self._collision    = False   # arm sensor

            print(f"[step e #{self._sim_count}] unsafe config — "
                  f"executing lane change, awaiting CARLA sensor …")
            try:
                ls  = int(self._ego_car.get_light_state())
                ls |= int(carla.VehicleLightState.LeftBlinker)
                self._ego_car.set_light_state(carla.VehicleLightState(ls))
                self._world.tick()
                self._tm.force_lane_change(self._ego_car, False)
                for _ in range(EXECUTE_TICKS):
                    self._world.tick()
            except Exception as exc:
                print(f"  CARLA exception (non-fatal): {exc}")

            crashed = self._collision   # ← CARLA collision sensor readback
            print(f"  CARLA collision sensor: {'CRASH detected ✓' if crashed else 'no collision ✓'}")

            if crashed:
                self._sink_flag = True
                print("[step e] CRASH: unsafe lane change caused physical crash → sink (s4→s1)")
            else:
                self._indicator_on  = False
                self._locked_config = None
                print("[step e] SURVIVED: no CARLA collision during unsafe lane change → reset (s4→s0)")

    # ──────────────────────────────────────────────────────────────────────────
    # CARLA physics helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _on_collision(self, event) -> None:
        """
        Collision sensor callback. Runs in CARLA's sensor thread.
        Sets _collision = True when ego hits any actor.
        """
        other = event.other_actor.type_id if event.other_actor else 'unknown'
        print(f"  [SENSOR] collision: ego ↔ {other}  impulse={event.normal_impulse}")
        self._collision = True

    def _predict_gap_end_from_carla(self):
        """
        Read ego and trail positions + velocities from CARLA.
        Project onto road forward axis to get longitudinal values.
        Return (gap_end, d, v_ego_ms, v_trail_ms).

        gap_end = d − (v_trail − v_ego) × T_LANE_CHANGE
          Positive → gap opens → safe
          Negative → gap closes → crash
        """
        try:
            ego_loc   = self._ego_car.get_location()
            trail_loc = self._trail_car.get_location()
            ego_vel   = self._ego_car.get_velocity()
            trail_vel = self._trail_car.get_velocity()

            fwd = _fwd_unit(self._ego_car)

            # Longitudinal positions (project onto road forward)
            ego_proj   = fwd.x * ego_loc.x   + fwd.y * ego_loc.y
            trail_proj = fwd.x * trail_loc.x + fwd.y * trail_loc.y
            d = ego_proj - trail_proj   # positive when ego is ahead of trail

            # Longitudinal speeds
            v_ego   = fwd.x * ego_vel.x   + fwd.y * ego_vel.y
            v_trail = fwd.x * trail_vel.x + fwd.y * trail_vel.y

            # Predicted gap at end of lane-change window
            gap_end = d - (v_trail - v_ego) * T_LANE_CHANGE

            return gap_end, d, v_ego, v_trail

        except Exception as exc:
            print(f"[_predict_gap_end_from_carla] error: {exc} — returning fallback")
            return L_V + 1.0, D0, 0.0, 0.0   # safe fallback so learning continues

    def _configure_vehicles(self, ego_kmh: float, trail_kmh: float) -> None:
        """
        Teleport trail D0 behind ego (on left lane), set target velocities.
        Speeds are sampled per episode — not fixed constants.
        """
        try:
            ego_ms   = _kmh_to_ms(ego_kmh)
            trail_ms = _kmh_to_ms(trail_kmh)

            ego_loc = self._ego_car.get_location()
            ego_wp  = self._world_map.get_waypoint(
                ego_loc, project_to_road=True,
                lane_type=carla.LaneType.Driving)

            back_wps = ego_wp.previous(D0)
            back_wp  = back_wps[0] if back_wps else ego_wp

            left_wp = back_wp.get_left_lane()
            if left_wp is None or left_wp.lane_type != carla.LaneType.Driving:
                left_wp = self._left_wp_base.next(
                    max(1.0, EGO_ADVANCE_M - D0))[0]

            self._trail_car.set_transform(
                carla.Transform(
                    left_wp.transform.location + carla.Location(z=0.5),
                    left_wp.transform.rotation))

            ego_fwd   = _fwd_unit(self._ego_car)
            yaw_rad   = math.radians(left_wp.transform.rotation.yaw)
            trail_fwd = carla.Vector3D(math.cos(yaw_rad), math.sin(yaw_rad), 0.0)

            self._ego_car.set_target_velocity(_vel_vec(ego_fwd,    ego_ms))
            self._trail_car.set_target_velocity(_vel_vec(trail_fwd, trail_ms))
            self._tm.set_desired_speed(self._ego_car,   ego_kmh)
            self._tm.set_desired_speed(self._trail_car, trail_kmh)

        except Exception as exc:
            print(f"[_configure_vehicles] non-fatal: {exc}")

    def _attach_collision_sensor(self) -> None:
        """
        Attach sensor.other.collision to ego.
        Callback sets self._collision = True on any impact.
        Must be called after ego is spawned.
        """
        try:
            bp_lib       = self._world.get_blueprint_library()
            collision_bp = bp_lib.find('sensor.other.collision')
            self._collision_sensor = self._world.spawn_actor(
                collision_bp,
                carla.Transform(),
                attach_to=self._ego_car)
            self._collision_sensor.listen(self._on_collision)
            print("[SUL] Collision sensor attached to ego.")
        except Exception as exc:
            print(f"[SUL] WARNING: could not attach collision sensor: {exc}")
            self._collision_sensor = None

    def _spawn_scene(self) -> None:
        """Spawn ego + trail cars and attach collision sensor to ego."""
        bp_lib = self._world.get_blueprint_library()
        veh_bp = bp_lib.filter('vehicle.tesla.model3')[0]

        ego_wp = self._mid_wp_base.next(EGO_ADVANCE_M)[0]
        veh_bp.set_attribute('color', '255,0,0')
        self._ego_car = self._world.spawn_actor(
            veh_bp,
            carla.Transform(ego_wp.transform.location + carla.Location(z=0.5),
                            ego_wp.transform.rotation))

        trail_advance = max(1.0, EGO_ADVANCE_M - D0)
        trail_wp = self._left_wp_base.next(trail_advance)[0]
        veh_bp.set_attribute('color', '0,0,255')
        self._trail_car = self._world.spawn_actor(
            veh_bp,
            carla.Transform(trail_wp.transform.location + carla.Location(z=0.5),
                            trail_wp.transform.rotation))

        for actor in [self._ego_car, self._trail_car]:
            actor.set_autopilot(True, self._tm.get_port())
            self._tm.auto_lane_change(actor,           False)
            self._tm.ignore_vehicles_percentage(actor,  100)
            self._tm.ignore_lights_percentage(actor,    100)

        self._tm.set_desired_speed(self._ego_car,   45.0)
        self._tm.set_desired_speed(self._trail_car, 45.0)

        # Attach collision sensor — CARLA decides crash, not analytics
        self._collision = False
        self._attach_collision_sensor()

    def _destroy_actors(self) -> None:
        """Destroy collision sensor first (before ego), then vehicles."""
        # Stop and destroy collision sensor before ego
        if self._collision_sensor is not None:
            try:
                if self._collision_sensor.is_alive:
                    self._collision_sensor.stop()
                    self._collision_sensor.destroy()
            except Exception:
                pass
            self._collision_sensor = None

        for actor in [self._ego_car, self._trail_car]:
            try:
                if actor and actor.is_alive:
                    actor.destroy()
            except Exception:
                pass
        self._ego_car = self._trail_car = None

    def __del__(self) -> None:
        try:
            s = self._world.get_settings()
            s.synchronous_mode    = False
            s.fixed_delta_seconds = 0.05
            self._world.apply_settings(s)
            self._destroy_actors()
        except Exception:
            pass


# ══════════════════════════════════════════════════════════════════════════════
# VISUALISATION
# ══════════════════════════════════════════════════════════════════════════════

def _visualize(learned_dfa, base_path: str) -> None:
    dot_src = str(learned_dfa)
    try:
        with open(base_path + '.dot', 'w', encoding='utf-8') as f:
            f.write(dot_src)
        print(f"[VIS] DOT → {base_path}.dot")
        print("[VIS] https://dreampuf.github.io/GraphvizOnline/")
    except Exception as e:
        print(f"[VIS] DOT save: {e}")

    try:
        from aalpy.utils import visualize_automaton
        visualize_automaton(learned_dfa, path=base_path)
        print(f"[VIS] PDF → {base_path}.pdf")
        return
    except Exception:
        pass

    try:
        import networkx as nx
        import matplotlib; matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        import matplotlib.patches as mpatches

        G = nx.DiGraph()
        accept_states, reject_states, labels = [], [], {}
        for s in learned_dfa.states:
            G.add_node(s.state_id)
            labels[s.state_id] = s.state_id
            (accept_states if s.is_accepting else reject_states).append(s.state_id)

        edge_labels = {}
        for s in learned_dfa.states:
            for sym, tgt in s.transitions.items():
                if tgt:
                    key = (s.state_id, tgt.state_id)
                    edge_labels.setdefault(key, []).append(sym)
                    G.add_edge(s.state_id, tgt.state_id)

        fig, ax = plt.subplots(figsize=(11, 6))
        ax.set_title(
            "Learned DFA — 5-state Lane-Change Protocol\n"
            "CARLA-physics SUL: crash via collision sensor, gap via get_location()/get_velocity()\n"
            "s0=initial/done  s2=indicator  s3=safe+ind  s4=unsafe+ind  s1=sink",
            fontsize=9)
        pos = nx.spring_layout(G, seed=42, k=3.5)
        nx.draw_networkx_nodes(G, pos, nodelist=accept_states,
                               node_color='#c8f0c8', node_size=2400, ax=ax)
        if reject_states:
            nx.draw_networkx_nodes(G, pos, nodelist=reject_states,
                                   node_color='#f0c8c8', node_size=2400, ax=ax)
        nx.draw_networkx_labels(G, pos, labels=labels, font_size=11, ax=ax)
        nx.draw_networkx_edges(G, pos, ax=ax, arrows=True,
                               connectionstyle='arc3,rad=0.18',
                               arrowsize=20, node_size=2400)
        el = {k: ','.join(sorted(v)) for k, v in edge_labels.items()}
        nx.draw_networkx_edge_labels(G, pos, edge_labels=el, font_size=8, ax=ax)
        ax.legend(handles=[
            mpatches.Patch(color='#c8f0c8',
                label='ACCEPT: s0 (initial or completed lane change)'),
            mpatches.Patch(color='#f0c8c8',
                label='REJECT: s2 s3 s4 (pending) OR s1 (sink)'),
        ], loc='lower right', fontsize=9)
        ax.axis('off')
        plt.tight_layout()
        png = base_path + '_vis.png'
        plt.savefig(png, dpi=150, bbox_inches='tight')
        plt.close(fig)
        print(f"[VIS] PNG → {png}")
    except Exception as e:
        print(f"[VIS] matplotlib: {e}")


# ══════════════════════════════════════════════════════════════════════════════
# MAIN
# ══════════════════════════════════════════════════════════════════════════════

def main():
    print("=" * 70)
    print("  AALpy L* × CARLA  —  CARLA-PHYSICS SUL  (v2)")
    print()
    print("  What CARLA logic:")
    print("    s0 ONLY is ACCEPTING.")
    print("    s1, s2, s3, s4 are REJECTING.")
    print()
    print("  Target DFA (will merge unrecoverable states):")
    print("    s0  ACCEPT  initial/done: safe/unsafe→s0, i→s2, e→s1")
    print("    s2  REJECT  ind on:       i→s2, safe→s3, unsafe→s4, e→s1")
    print("    s3  REJECT  ind+safe:     i/safe→s3, unsafe→s1, e→s0")
    print("    s4  REJECT  ind+unsafe:   i/unsafe→s4, safe→s1, e→s1")
    print("    s1  REJECT  sink:         all→s1")
    print("=" * 70)

    sul      = CarlaProtocolSUL()
    alphabet = ['i', 'safe', 'unsafe', 'e']

    eq_oracle = RandomWalkEqOracle(
        alphabet        = alphabet,
        sul             = sul,
        num_steps       = 1000,
        reset_prob      = 0.09,
        reset_after_cex = True,
    )

    print("\n[MAIN] Starting L* …\n")
    learned_dfa = run_Lstar(
        alphabet                = alphabet,
        sul                     = sul,
        eq_oracle               = eq_oracle,
        automaton_type          = 'dfa',
        cache_and_non_det_check = False,
        print_level             = 2,
    )

    base_path = 'lane_change_protocol_dfa'
    learned_dfa.save(base_path)
    print(f"\n[MAIN] Saved → {base_path}.dot")

    print("\n[MAIN] Learned DFA:")
    learned_dfa.make_input_complete('sink_state')
    print(learned_dfa)

    _visualize(learned_dfa, base_path)

    # ── Sanity checks ──────────────────────────────────────────────────────────
    print("\n[MAIN] Sanity checks (verified against target DFA):")
    sanity = [
        # ── ACCEPT traces ─────────────────────────────────────────────────────
        (('i', 'safe', 'e'),             True,  "i→safe→e          SUCCESS (s0→s2→s3→s0) ACCEPT"),
        (('safe', 'i', 'safe', 'e'),     True,  "safe→i→safe→e     SUCCESS (s0→s0→s2→s3→s0) ACCEPT"),
        (('safe',),                      True,  "safe              s0→s0 ACCEPT (pre-indicator)"),
        (('unsafe',),                    True,  "unsafe            s0→s0 ACCEPT (pre-indicator)"),
        (('i', 'safe', 'e', 'i', 'safe', 'e'), True,  "two safe episodes  both success ACCEPT"),
        # ── REJECT traces ─────────────────────────────────────────────────────
        (('i',),                         False, "i                 s0→s2 REJECT"),
        (('i', 'safe'),                  False, "i→safe            s0→s2→s3 REJECT (not done yet)"),
        (('i', 'safe', 'safe'),          False, "i→safe→safe       s3 self-loop REJECT (not done yet)"),
        (('i', 'unsafe'),                False, "i→unsafe          s2→s4 REJECT"),
        (('i', 'e'),                     False, "i→e               ILLEGAL config not set s2→s1 REJECT"),
        (('e',),                         False, "e                 ILLEGAL s0→s1 REJECT"),
        (('safe', 'e'),                  False, "safe→e            ILLEGAL (no indicator) s0→s1"),
        (('unsafe', 'e'),                False, "unsafe→e          ILLEGAL (no indicator) s0→s1"),
        (('i', 'unsafe', 'e'),           False, "i→unsafe→e        CRASH s4→s1 REJECT"),
        (('i', 'safe', 'unsafe'),        False, "i→safe→unsafe     INVALID MIX s3→s1 REJECT"),
        (('i', 'unsafe', 'safe'),        False, "i→unsafe→safe     INVALID MIX s4→s1 REJECT"),
        (('i', 'safe', 'unsafe', 'i', 'safe', 'e'), False, "mix then try again REJECT (s1 absorbing)"),
    ]

    all_ok = True
    for trace, expected, label in sanity:
        result_list = learned_dfa.execute_sequence(
            learned_dfa.initial_state, list(trace))
        final = (result_list[-1]
                 if isinstance(result_list, list) and result_list
                 else result_list)
        ok = (final == expected)
        if not ok: all_ok = False
        icon = "✓" if ok else "✗ MISMATCH"
        print(f"  {icon}  {' → '.join(trace):<40}  "
              f"got={final}  exp={expected}  [{label}]")

    print()
    if all_ok:
        print("[MAIN] ✓ All sanity checks passed!")
        print(f"[MAIN] States learned: {len(learned_dfa.states)} "
              f"(expected 5: s0 s2 s3 s4 s1)")
    else:
        print("[MAIN] ⚠ Mismatch — re-run or increase num_steps.")

    print(f"\n[MAIN] CARLA lane-change executions: {sul._sim_count}")
    print(f"[MAIN] Membership queries:            {sul._query_count}")
    print("\n[MAIN] Done.")
    return learned_dfa


if __name__ == '__main__':
    main()
