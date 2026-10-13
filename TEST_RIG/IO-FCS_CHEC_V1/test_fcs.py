import ctypes
import math
import os
import random
import shutil
import subprocess
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))

# ----------------------------------------------------------------------------- build + load
def build_lib():
    gcc = shutil.which("gcc") or shutil.which("cc")
    if gcc is None:
        raise unittest.SkipTest("no C compiler found")
    out = os.path.join(tempfile.mkdtemp(prefix="fcs_"), "libfcs_air.so")
    cmd = [gcc, "-shared", "-fPIC", "-O2", "-std=c99", "-Wall", "-Wextra",
           "-I", HERE, os.path.join(HERE, "fcs_pywrap.c"), "-o", out, "-lm"]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError("C build failed:\n" + r.stderr)
    return ctypes.CDLL(out)

lib = None

def load():
    global lib
    if lib is not None:
        return
    L = build_lib()
    L.w_new.restype = ctypes.c_void_p
    L.w_free.argtypes = [ctypes.c_void_p]
    L.w_start_airborne.argtypes = [ctypes.c_void_p]
    L.w_step.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint16), ctypes.c_uint32,
                         ctypes.c_float, ctypes.c_float, ctypes.c_int, ctypes.c_int, ctypes.c_uint32,
                         ctypes.c_uint32, ctypes.c_float, ctypes.POINTER(ctypes.c_uint16)]
    L.w_get.argtypes = [ctypes.c_void_p, ctypes.c_int]
    L.w_get.restype = ctypes.c_double
    L.w_cfg.argtypes = [ctypes.c_int]
    L.w_cfg.restype = ctypes.c_double
    lib = L

# channel indices and mode ids (match the C enums)
AIL, ELE, THR, RUD, HOLD, FLAP = range(6)
DISARMED, GROUND, NORMAL, HOLD_M, FAILSAFE = range(5)

def cfg(i): return lib.w_cfg(i)
VS_CLEAN, VS_FULL, VFE_FULL, VFE_MID, VNE, CEILING, CEIL_THR_CAP, FS_THR, BANK_MAX, THR_RECOVERY, \
    AIL_MAX, SLEW_SURF, SLEW_THR_UP, SLEW_FLAP, V_REF, ALT_SPIKE, ALT_REJ_MAX, SLEW_THR_DN = range(18)


def clamp(x, lo, hi): return lo if x < lo else hi if x > hi else x

# ----------------------------------------------------------------------------- toy plant
class Plant:
    """Pitch / speed / altitude only. NOT an aerodynamic model."""
    def __init__(self, v=18.0, alt=60.0):
        self.theta = self.q = 0.0
        self.v, self.alt = v, alt

    def step(self, ele, thr, dt):
        stall_m = -6.0 * (10.5 - self.v) if self.v < 10.5 else 0.0
        self.q += (8.0 * ele - 5.0 * self.q - 12.0 * self.theta + stall_m) * dt
        self.theta += self.q * dt
        self.v += (0.4 * (30.0 * thr - self.v) - 6.0 * math.sin(self.theta)) * dt
        self.v = max(self.v, 0.0)
        self.alt += self.v * math.sin(self.theta) * dt

# ----------------------------------------------------------------------------- harness
class Sim:
    """One aircraft + receiver + sensors. Set the input attributes, call run()."""
    def __init__(self, airborne=True, v=18.0, alt=100.0, plant=None):
        load()
        self.h = lib.w_new()
        if airborne:
            lib.w_start_airborne(self.h)
        self.dt = 0.01
        self.now = 0
        # pilot inputs
        self.ail = self.ele = self.rud = 0.0       # -1..1
        self.thr = 0.0                              # 0..1
        self.flap = 0                               # 0, 1 (half), 2 (full)
        self.hold = False
        # air data (open loop unless a plant is attached)
        self.v, self.alt = v, alt
        self.v_valid = self.alt_valid = True
        self.plant = plant
        # fault injection
        self.link = True
        self.air_fresh = True
        self._rc_stamp = 0
        self._air_stamp = 0
        self.out = [1500] * 6

    def __del__(self):
        try:
            lib.w_free(self.h)
        except Exception:
            pass

    # state getters
    def get(self, i): return lib.w_get(self.h, i)
    mode = property(lambda s: int(s.get(0)))
    armed = property(lambda s: bool(s.get(1)))
    airborne = property(lambda s: bool(s.get(2)))
    holding = property(lambda s: bool(s.get(3)))
    rc_ok = property(lambda s: bool(s.get(4)))
    asp_ok = property(lambda s: bool(s.get(5)))
    alt_ok = property(lambda s: bool(s.get(6)))
    v_f = property(lambda s: s.get(7))
    alt_f = property(lambda s: s.get(8))
    vz_f = property(lambda s: s.get(9))
    bank_est = property(lambda s: s.get(10))
    pitch_est = property(lambda s: s.get(11))
    o_thr = property(lambda s: s.get(14))
    o_flap = property(lambda s: s.get(16))

    def step(self):
        if self.plant:
            self.v, self.alt = self.plant.v, self.plant.alt
        if self.link:
            self._rc_stamp = self.now
        if self.air_fresh:
            self._air_stamp = self.now
        us = lambda x: int(round(1500 + 500 * clamp(x, -1, 1)))
        rc = (ctypes.c_uint16 * 6)(
            us(self.ail), us(self.ele),
            int(round(1000 + 1000 * clamp(self.thr, 0, 1))), us(self.rud),
            2000 if self.hold else 1000, (1000, 1500, 2000)[self.flap])
        out = (ctypes.c_uint16 * 6)()
        lib.w_step(self.h, rc, self._rc_stamp, self.v, self.alt, int(self.v_valid),
                   int(self.alt_valid), self._air_stamp, self.now, self.dt, out)
        self.out = list(out)
        self.now += int(self.dt * 1000)
        if self.plant:
            self.plant.step(self.get(13), self.get(14), self.dt)

    def run(self, seconds, each=None):
        for _ in range(int(round(seconds / self.dt))):
            self.step()
            if each:
                each(self)

    # convenience: output in -1..1 form
    def surf(self, ch): return (self.out[ch] - 1500) / 500.0


# ============================================================================ 1. UNIT SCENARIOS
class TestArmingAndGround(unittest.TestCase):
    def test_throttle_interlock_blocks_power_until_idle_seen(self):
        s = Sim(airborne=False, v=0, alt=0)
        s.thr = 0.8                                  # throttle high at power-up
        s.run(1.5)
        self.assertEqual(s.mode, DISARMED)
        self.assertEqual(s.out[THR], 1000, "throttle must stay at idle while disarmed")
        s.thr = 0.0
        s.run(0.5)                                   # idle seen -> arms
        self.assertEqual(s.mode, GROUND)
        s.thr = 0.3
        s.run(1.0)
        self.assertAlmostEqual(s.out[THR], 1300, delta=3)

    def test_disarmed_surfaces_still_follow_sticks(self):
        s = Sim(airborne=False, v=0, alt=0)
        s.thr = 0.8; s.ail = 0.5
        s.run(1.5)
        self.assertEqual(s.mode, DISARMED)
        self.assertGreater(s.out[AIL], 1600)

    def test_ground_has_no_flight_protections(self):
        # at zero airspeed the stall-throttle floor must NOT spin the prop up on the runway
        s = Sim(airborne=False, v=0, alt=0)
        s.run(0.5)                                   # arm at idle
        s.thr = 0.2
        s.run(1.5)
        self.assertEqual(s.mode, GROUND)
        self.assertFalse(s.airborne)
        self.assertAlmostEqual(s.out[THR], 1200, delta=3)

    def test_airborne_latch_and_landing_rollout_disarms(self):
        s = Sim(airborne=False, v=0, alt=0)
        s.run(0.5)
        s.v = 1.3 * cfg(VS_CLEAN)
        s.run(1.0)                                   # airspeed filter (tau 0.2 s) needs time to settle
        self.assertTrue(s.airborne)
        self.assertEqual(s.mode, NORMAL)
        s.thr = 0.5                                  # pilot has not pulled throttle to idle yet
        s.v = 0.3 * cfg(VS_CLEAN)                    # rollout
        s.run(1.0)
        self.assertTrue(s.airborne, "must not unlatch instantly (stall blip)")
        s.run(2.5)
        self.assertFalse(s.airborne)
        self.assertFalse(s.armed, "re-arm required after landing")
        self.assertEqual(s.mode, DISARMED)
        self.assertEqual(s.out[THR], 1000)
        s.thr = 0.0                                  # idle seen again -> re-arms
        s.run(0.2)
        self.assertEqual(s.mode, GROUND)


class TestStallProtection(unittest.TestCase):
    def test_up_elevator_removed_nose_down_bias_and_throttle_floor_near_stall(self):
        s = Sim(v=cfg(VS_CLEAN) * 1.05)              # below the 1.15 Vs margin
        s.ele = 1.0; s.thr = 0.3
        s.run(2.0)
        self.assertLess(s.surf(ELE), -0.1, "elevator should be driven nose-down")
        self.assertGreater(s.o_thr, 0.6, "power floor should lift throttle")
        self.assertGreater(s.o_thr, s.thr)

    def test_no_intervention_with_healthy_margin(self):
        s = Sim(v=18.0)
        s.ele = 0.3; s.thr = 0.5
        s.run(2.0)
        self.assertGreater(s.surf(ELE), 0.05)
        self.assertAlmostEqual(s.o_thr, 0.5, delta=0.01)

    def test_stall_speed_rises_with_dead_reckoned_bank(self):
        # same speed: fine wings-level, but a long hard turn raises the effective stall speed
        v = 1.40 * cfg(VS_CLEAN)
        a = Sim(v=v); a.thr = 0.3; a.run(2.0)
        b = Sim(v=v); b.thr = 0.3; b.ail = 1.0; b.run(6.0)
        self.assertGreater(b.bank_est, math.radians(25))
        self.assertGreaterEqual(b.o_thr, a.o_thr)

    def test_aileron_authority_reduced_near_stall(self):
        slow = Sim(v=cfg(VS_CLEAN) * 1.1); slow.ail = 0.8; slow.run(0.3)
        fast = Sim(v=18.0);                fast.ail = 0.8; fast.run(0.3)
        self.assertLess(abs(slow.surf(AIL)), abs(fast.surf(AIL)))


class TestOverspeed(unittest.TestCase):
    def test_throttle_cut_and_no_push_over_near_vne(self):
        s = Sim(v=cfg(VNE) * 1.02)
        s.thr = 1.0; s.ele = -1.0
        s.run(2.0)
        self.assertEqual(s.out[THR], 1000)
        self.assertGreaterEqual(s.out[ELE], 1500, "must not command nose-down at Vne")

    def test_partial_throttle_scaling_between_90_and_100_percent_vne(self):
        s = Sim(v=cfg(VNE) * 0.95)
        s.thr = 1.0
        s.run(2.0)
        self.assertTrue(0.1 < s.o_thr < 0.9)


class TestGainScheduling(unittest.TestCase):
    def test_smaller_deflection_when_fast(self):
        slow = Sim(v=15.0); slow.ele = 0.3; slow.thr = 0.5; slow.run(0.6)
        fast = Sim(v=26.0); fast.ele = 0.3; fast.thr = 0.5; fast.run(0.6)
        self.assertGreater(abs(slow.surf(ELE)), 2.0 * abs(fast.surf(ELE)))


class TestBankLimiter(unittest.TestCase):
    def test_sustained_full_aileron_is_bounded_then_decays(self):
        s = Sim(v=18.0); s.thr = 0.6; s.ail = 1.0
        peak = [0.0]
        def track(sim): peak[0] = max(peak[0], sim.bank_est)
        s.run(10.0, each=track)
        self.assertLess(peak[0], cfg(BANK_MAX) * 1.02)
        self.assertLess(abs(s.surf(AIL)), cfg(AIL_MAX), "aileron must taper below its authority limit")
        s.ail = 0.0
        s.run(6.0)
        self.assertLess(s.bank_est, math.radians(5))


class TestAltitudeProtection(unittest.TestCase):
    def test_above_ceiling_throttle_capped_and_no_up_elevator(self):
        s = Sim(v=18.0, alt=cfg(CEILING) + 20)
        s.thr = 1.0; s.ele = 1.0
        s.run(2.0)
        self.assertLessEqual(s.o_thr, cfg(CEIL_THR_CAP) + 0.01)
        self.assertLess(s.surf(ELE), 0.0, "ceiling bias should push nose-down")

    def test_below_ceiling_margin_is_untouched(self):
        s = Sim(v=18.0, alt=60.0)
        s.thr = 1.0; s.ele = 0.4
        s.run(2.0)
        self.assertGreater(s.o_thr, 0.95)
        self.assertGreater(s.surf(ELE), 0.1)

    def test_climb_angle_limiter_tapers_up_elevator(self):
        # steady 5 m/s climb at 18 m/s is ~16 deg: past the 15 deg flight-path limit
        s = Sim(v=18.0, alt=40.0)
        s.thr = 0.6; s.ele = 0.5
        s.plant = None
        def rise(sim): sim.alt += 5.0 * sim.dt
        s.run(3.0, each=rise)
        self.assertGreater(s.vz_f, 3.5)
        self.assertLess(s.surf(ELE), 0.05, "up-elevator should be tapered away")


class TestFlaps(unittest.TestCase):
    def test_full_flap_request_capped_to_half_between_vfe_limits(self):
        s = Sim(v=(cfg(VFE_FULL) + cfg(VFE_MID)) / 2)
        s.flap = 2; s.thr = 0.5
        s.run(4.5)
        self.assertAlmostEqual(s.o_flap, 0.5, delta=0.02)

    def test_flaps_refused_above_mid_speed(self):
        s = Sim(v=cfg(VFE_MID) + 2)
        s.flap = 2; s.thr = 0.5
        s.run(3.0)
        self.assertEqual(s.out[FLAP], 1000)

    def test_no_retract_when_slow_but_retracts_when_safe(self):
        s = Sim(v=cfg(VFE_FULL) - 1.5); s.thr = 0.5
        s.flap = 2; s.run(4.0)
        self.assertGreater(s.o_flap, 0.95)
        s.v = 1.2 * cfg(VS_CLEAN)                    # slow
        s.flap = 0; s.run(3.0)
        self.assertGreater(s.o_flap, 0.95, "must keep flaps while slow")
        s.v = 18.0
        s.run(4.0)
        self.assertLess(s.o_flap, 0.05)

    def test_flap_slew_rate_respected(self):
        s = Sim(v=cfg(VFE_FULL) - 2); s.thr = 0.5; s.flap = 2
        s.run(1.0)
        self.assertLessEqual(s.o_flap, cfg(SLEW_FLAP) * 1.0 + 0.02)


class TestHold(unittest.TestCase):
    def setUp(self):
        self.s = Sim(v=18.0)
        self.s.thr = 0.7; self.s.ele = 0.15; self.s.ail = 0.2
        self.s.run(0.5)
        self.before = list(self.s.out)

    def test_latches_inputs_and_survives_stick_release(self):
        s = self.s
        s.hold = True
        s.run(0.05)
        self.assertTrue(s.holding)
        s.ele = s.ail = 0.0; s.thr = 0.9            # release sticks, throttle stick higher than latched
        s.run(1.0)
        self.assertTrue(s.holding)
        for ch in (AIL, ELE):
            self.assertAlmostEqual(s.out[ch], self.before[ch], delta=30, msg=f"channel {ch} drifted")
        self.assertAlmostEqual(s.out[THR], self.before[THR], delta=20)

    def test_throttle_can_always_be_pulled_back_in_hold(self):
        s = self.s
        s.hold = True; s.run(0.1)
        s.thr = 0.3; s.run(1.5)
        self.assertTrue(s.holding)
        self.assertAlmostEqual(s.out[THR], 1300, delta=5)

    def test_big_stick_deflection_breaks_hold_until_switch_cycled(self):
        s = self.s
        s.hold = True; s.run(0.1)
        s.ail = 1.0; s.run(0.2)
        self.assertFalse(s.holding)
        s.ail = 0.0; s.run(0.5)
        self.assertFalse(s.holding, "must stay out while the switch is still high")
        s.hold = False; s.run(0.1)
        s.hold = True; s.run(0.1)
        self.assertTrue(s.holding)

    def test_hold_does_not_engage_with_switch_already_high_at_boot_or_on_ground(self):
        g = Sim(airborne=False, v=0, alt=0)
        g.run(0.5)
        g.hold = True; g.run(0.5)
        self.assertFalse(g.holding)

    def test_protections_still_apply_on_top_of_hold(self):
        s = self.s
        s.ele = 0.6; s.thr = 0.2; s.run(0.3)
        s.hold = True; s.run(0.1)
        s.v = cfg(VS_CLEAN) * 1.05                   # speed collapses while holding up-elevator
        s.run(2.0)
        self.assertTrue(s.holding)
        self.assertLess(s.surf(ELE), 0.0)
        self.assertGreater(s.o_thr, 0.5)


class TestFailsafe(unittest.TestCase):
    def test_link_loss_enters_failsafe_and_recovers(self):
        s = Sim(v=18.0); s.thr = 0.8; s.flap = 1; s.run(3.0)
        s.link = False
        s.run(0.05)
        self.assertNotEqual(s.mode, FAILSAFE, "not instant")
        s.run(0.25)                                  # 0.30 s: past the 250 ms frame timeout, inside the debounce
        self.assertNotEqual(s.mode, FAILSAFE, "debounce: a short dropout must not trigger failsafe")
        s.run(0.15)                                  # 0.45 s: timeout + debounce elapsed
        self.assertEqual(s.mode, FAILSAFE)
        s.run(4.0)
        self.assertAlmostEqual(s.o_thr, cfg(FS_THR), delta=0.02)
        self.assertLess(s.o_flap, 0.02, "flaps retract in failsafe")
        self.assertGreater(s.surf(AIL), 0.0, "gentle turn")
        s.link = True
        s.run(0.2)
        self.assertEqual(s.mode, FAILSAFE, "recovery is debounced")
        s.run(0.5)
        self.assertEqual(s.mode, NORMAL)

    def test_failsafe_airspeed_loop_pulls_up_when_fast_pushes_when_slow(self):
        fast = Sim(v=24.0); fast.link = False; fast.run(1.0)
        slow = Sim(v=13.0); slow.link = False; slow.run(1.0)
        self.assertGreater(fast.surf(ELE), slow.surf(ELE))

    def test_link_loss_on_the_ground_leaves_everything_neutral_and_power_off(self):
        s = Sim(airborne=False, v=0, alt=0)
        s.run(0.5); s.thr = 0.4; s.run(1.0)
        self.assertGreater(s.out[THR], 1300)
        s.link = False
        s.run(1.5)
        self.assertEqual(s.mode, FAILSAFE)
        self.assertEqual(s.out[THR], 1000)
        for ch in (AIL, ELE, RUD):
            self.assertAlmostEqual(s.out[ch], 1500, delta=2)

    def test_stale_rc_frames_keep_last_good_command_during_debounce(self):
        s = Sim(v=18.0); s.ail = 0.2; s.thr = 0.5; s.run(1.0)
        before = s.out[AIL]
        s.link = False
        s.run(0.05)                                  # shorter than debounce
        self.assertAlmostEqual(s.out[AIL], before, delta=15)


class TestSensorFaults(unittest.TestCase):
    def test_baro_spike_is_rejected(self):
        s = Sim(v=18.0, alt=100.0); s.thr = 0.6; s.run(1.0)
        s.alt = 100.0 + cfg(ALT_SPIKE) * 3
        s.run(0.3)                                   # shorter than the re-accept window
        self.assertLess(abs(s.alt_f - 100.0), 1.0)
        self.assertLess(abs(s.vz_f), 0.5)
        s.alt = 100.0
        s.run(0.5)
        self.assertLess(abs(s.alt_f - 100.0), 1.0)

    def test_persistent_altitude_step_is_eventually_accepted(self):
        s = Sim(v=18.0, alt=100.0); s.thr = 0.6; s.run(1.0)
        s.alt = 100.0 + cfg(ALT_SPIKE) * 3
        s.run(cfg(ALT_REJ_MAX) * 0.01 + 0.5)
        self.assertAlmostEqual(s.alt_f, s.alt, delta=1.0)
        self.assertLess(abs(s.vz_f), 1.0, "re-init must not create a fake climb rate")

    def test_airspeed_failure_disables_speed_protection_but_keeps_flying(self):
        s = Sim(v=cfg(VS_CLEAN) * 1.05)              # low, but the sensor is dead
        s.v_valid = False; s.thr = 0.3; s.ele = 0.5
        s.run(4.0)
        self.assertFalse(s.asp_ok)
        self.assertTrue(s.airborne, "latch frozen when airspeed is lost")
        self.assertEqual(s.mode, NORMAL)
        self.assertAlmostEqual(s.o_thr, 0.3, delta=0.01, msg="no stall floor without airspeed")
        self.assertGreater(s.surf(ELE), 0.0)

    def test_altitude_failure_disables_altitude_protection(self):
        s = Sim(v=18.0, alt=cfg(CEILING) + 30)
        s.alt_valid = False; s.thr = 1.0
        s.run(2.0)
        self.assertFalse(s.alt_ok)
        self.assertGreater(s.o_thr, 0.95)

    def test_stale_air_data_counts_as_invalid_and_recovers_cleanly(self):
        s = Sim(v=18.0, alt=100.0); s.run(1.0)
        s.air_fresh = False
        s.run(0.5)
        self.assertFalse(s.asp_ok); self.assertFalse(s.alt_ok)
        s.air_fresh = True
        s.run(0.3)
        self.assertTrue(s.asp_ok); self.assertTrue(s.alt_ok)
        self.assertAlmostEqual(s.v_f, 18.0, delta=0.1)

    def test_nan_and_garbage_air_data_rejected(self):
        s = Sim(v=18.0, alt=100.0); s.run(1.0)
        s.v = float("nan"); s.run(0.3)
        self.assertFalse(s.asp_ok)
        s.v = 500.0; s.run(0.3)
        self.assertFalse(s.asp_ok)
        s.v = 18.0; s.run(0.3)
        self.assertTrue(s.asp_ok)


# ============================================================================ 2. CLOSED LOOP
class TestClosedLoop(unittest.TestCase):
    def test_pilot_holds_full_back_stick_for_60s_envelope_holds(self):
        s = Sim(plant=Plant(v=18.0, alt=60.0))
        s.thr = 0.6; s.ele = 1.0
        stats = dict(max_alt=0.0, min_v=99.0)
        def track(sim):
            stats["max_alt"] = max(stats["max_alt"], sim.plant.alt)
            stats["min_v"] = min(stats["min_v"], sim.plant.v)
        s.run(60.0, each=track)
        self.assertLess(stats["max_alt"], cfg(CEILING) + 8.0, f"ceiling busted: {stats}")
        self.assertGreater(stats["min_v"], cfg(VS_CLEAN), f"flew below stall speed: {stats}")

    def test_pilot_holds_full_forward_stick_and_full_power_never_exceeds_vne(self):
        s = Sim(plant=Plant(v=20.0, alt=300.0))
        s.thr = 1.0; s.ele = -1.0
        stats = dict(max_v=0.0)
        def track(sim): stats["max_v"] = max(stats["max_v"], sim.plant.v)
        s.run(40.0, each=track)
        self.assertLess(stats["max_v"], cfg(VNE), f"Vne exceeded: {stats}")

    def test_throttle_chop_in_cruise_does_not_stall(self):
        s = Sim(plant=Plant(v=18.0, alt=80.0))
        s.thr = 0.0; s.ele = 0.4                     # power off and pulling back: classic stall setup
        stats = dict(min_v=99.0)
        def track(sim): stats["min_v"] = min(stats["min_v"], sim.plant.v)
        s.run(30.0, each=track)
        self.assertGreater(stats["min_v"], cfg(VS_CLEAN) * 0.95, f"{stats}")

    def test_hold_in_closed_loop_keeps_flying_after_stick_release(self):
        s = Sim(plant=Plant(v=18.0, alt=60.0))
        s.thr = 0.6; s.ele = 0.12; s.run(2.0)
        s.hold = True; s.run(0.1)
        s.ele = 0.0
        s.run(20.0)
        self.assertTrue(s.holding)
        self.assertGreater(s.plant.v, cfg(VS_CLEAN) * 1.1)
        self.assertLess(s.plant.alt, cfg(CEILING) + 8.0)


# ============================================================================ 3. FUZZ / INVARIANTS
class TestInvariants(unittest.TestCase):
    def test_random_abuse_keeps_outputs_sane(self):
        rng = random.Random(1234)
        for trial in range(8):
            s = Sim(airborne=rng.random() < 0.7, v=rng.uniform(0, 28), alt=rng.uniform(0, 140))
            s.out = [1500, 1500, 1000, 1500, 1000, 1000]   # true power-up output state
            prev = list(s.out)
            max_step = {
                AIL: cfg(SLEW_SURF) * s.dt * 500 + 1.5,
                ELE: cfg(SLEW_SURF) * s.dt * 500 + 1.5,
                RUD: cfg(SLEW_SURF) * s.dt * 500 + 1.5,
                THR: max(cfg(SLEW_THR_UP), cfg(SLEW_THR_DN)) * s.dt * 1000 + 1.5,
                FLAP: cfg(SLEW_FLAP) * s.dt * 1000 + 1.5,
            }
            s.plant = Plant(v=s.v, alt=s.alt) if trial % 2 == 0 else None
            for k in range(6000):
                if k % rng.randint(20, 120) == 0:    # new random sticks every so often
                    s.ail, s.ele, s.rud = (rng.uniform(-1, 1) for _ in range(3))
                    s.thr = rng.choice([0.0, rng.random(), 1.0])
                    s.flap = rng.choice([0, 1, 2])
                    s.hold = rng.random() < 0.3
                if k % 200 == 0:                     # random faults
                    s.link = rng.random() > 0.15
                    s.air_fresh = rng.random() > 0.1
                    s.v_valid = rng.random() > 0.1
                    s.alt_valid = rng.random() > 0.1
                if s.plant is None and k % 50 == 0:
                    s.v = rng.uniform(0, 40); s.alt = rng.uniform(-5, 200)
                s.step()
                for ch in range(6):
                    self.assertTrue(1000 <= s.out[ch] <= 2000, f"ch{ch}={s.out[ch]} out of range")
                for ch, lim in max_step.items():
                    self.assertLessEqual(abs(s.out[ch] - prev[ch]), lim,
                                         f"ch{ch} slewed too fast at tick {k}")
                if s.mode == DISARMED:
                    self.assertEqual(s.out[THR], 1000, "power while disarmed")
                if s.mode == FAILSAFE and not s.airborne:
                    self.assertEqual(s.out[THR], 1000)
                for i in (7, 8, 9, 10, 11, 12, 13, 14, 15, 16):
                    self.assertTrue(math.isfinite(s.get(i)), f"state {i} not finite")
                prev = list(s.out)


if __name__ == "__main__":
    load()
    unittest.main(verbosity=2 if "-v" in sys.argv else 1, argv=[a for a in sys.argv if a != "-v"])
