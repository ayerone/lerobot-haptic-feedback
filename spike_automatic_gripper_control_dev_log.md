# Automatic Gripper Force Control — Development Log (spike_automatic branch)

## Goal

Explore automatic closed-loop control of the follower gripper during teleoperation. Instead of the operator directly commanding gripper position via the gimbal, the robot maintains a target gripping force of 20 (0–100 scale) once contact is detected. The operator can override with intentional squeeze, and can break out of auto-grip by opening the teleop significantly.

---

## Phase 1: Basic P Controller

**What we tried:** A proportional controller adjusting `_auto_grip_pos` each cycle based on force error: `_auto_grip_pos += P_GAIN * (force - FORCE_SETPOINT)`. Initial gain: 0.05.

**Result:** Oscillation at ~3 Hz. Classic P-controller hunting.

**Fix:** Reduced gain to 0.01 and added a ±4 unit deadband around the setpoint — only adjust when `|force - 20| > 4`.

---

## Phase 2: Oscillation from State Reset

**What happened:** Even with reduced gain and deadband, force oscillated between 0 and ~55, with gripper head amplitude ~1/3 cm.

**Root cause:** When the P controller opened the gripper slightly above the contact point, force dropped to 0. The "no contact" branch reset `_auto_grip_pos = None`, handing control back to the gimbal. The gripper released, then re-contacted with a slam. Cycle repeated.

**Fix:** In the no-contact branch, hold `_auto_grip_pos` at its last value instead of resetting. The gripper stays put until the user explicitly breaks out. `_auto_grip_pos` is only cleared via the break condition (user opens teleop 3+ units past target).

---

## Phase 3: Still Springing Open Too Much

**What happened:** Contact at position X, force spikes to 50+ at X−3. P controller opens gripper back toward X. At X, force = 0 and object is no longer held firmly.

**Root cause:** The P controller's accumulated corrections overshoot past the initial contact point back to no-grip territory.

**Fix:** Record `_contact_pos = gripper_pos` when auto-grip first activates. Clamp `_auto_grip_pos = min(_auto_grip_pos, _contact_pos)` after each P update — the gripper can never open past where contact was first detected.

---

## Phase 4: Break Condition Not Working

**What happened:** Operator could not open the gripper while holding an object. The break condition fired (`_auto_grip_pos = None`) but auto-grip immediately re-activated in the same `compute()` call because force was still above the deadband.

**Fix:** Added `_auto_grip_broken` latch flag. When break fires, set latch. The gripping branches skip auto-grip activation while latch is set. Latch clears only when force drops to zero (object fully released).

---

## Phase 5: Torque Feedback (Spring)

The vibration feedback was left disabled on this branch. Added a spring torque to give the operator a feel for how far the gimbal is closed beyond the follower's actual gripper position:

```python
def _grip_spring_torque(self, gimbal_pos, reference):
    displacement = max(0.0, reference - gimbal_pos)
    return -GRIP_SPRING_SCALAR * math.sqrt(displacement)
```

- Negative torque pushes the gimbal toward opening.
- Reference = `gripper_pos` (robot's actual position), not `_auto_grip_pos`.
- `sqrt` gives a steep onset (strong sense of contact) that levels off at larger displacements.
- `GRIP_SPRING_SCALAR = 1.0` (tuned up from 0.03 after switching to sqrt; linear was too weak at small displacements).

**Key issue along the way:** Initially used `_auto_grip_pos` as the reference. The P controller drives `_auto_grip_pos` below the operator's natural gimbal position, so `reference < gimbal_pos` → torque always zero until the operator actively squeezed past it. Fix: use `gripper_pos` (actual robot position) as reference.

---

## Phase 6: Ceiling Refinement — `_tightest_pos`

**Problem:** `_contact_pos` ceiling prevented large bounces but small oscillations remained. Explanation: at `_contact_pos`, force ≈ 0 → P controller closes → tiny force spike → P opens → ceiling clamps → repeat. The remaining oscillation is in the servo's own PID loop (hardware), not addressable in Python.

**Improvement:** Replaced `_contact_pos` with `_tightest_pos` — the minimum `_auto_grip_pos` ever reached. The ceiling is `_tightest_pos` (previously tried a blend toward `_contact_pos`, settled on blend = 0 for minimum bounce-back). Residual bounce is servo hardware deadband/overshoot.

---

## Phase 7: Intentional Squeeze Override

**Goal:** While holding at force=20, operator can squeeze the gimbal past `_auto_grip_pos` to command 1–2 units of extra closure.

**First attempt (proportional nudge):** `squeeze = SCALAR * (auto_grip_pos - gimbal_pos)`, lowering `_tightest_pos`. Failed — P controller immediately re-opened to seek force=20, canceling the squeeze.

**Second attempt (direct follow):** When `gimbal_pos < _auto_grip_pos`, set `_auto_grip_pos = max(gimbal_pos, _tightest_pos - MAX_INTENTIONAL_SQUEEZE)` directly. The ceiling stays at `_tightest_pos`, preventing the P controller from locking in the tighter position — squeeze is momentary (releases when operator eases off).

**Key bug:** Initially used `_squeeze_floor = contact_pos - MAX_INTENTIONAL_SQUEEZE` as the hard floor. But P controller closed `_tightest_pos` well below `_contact_pos` before the squeeze was attempted. When operator squeezed, `max(gimbal_pos, _squeeze_floor)` floored out ABOVE `_tightest_pos` — commanding the gripper MORE open, not tighter. Fix: floor is now always `_tightest_pos - MAX_INTENTIONAL_SQUEEZE`, so the operator always has 2 units of headroom below wherever P control has settled.

---

## Current Constants (spike_automatic branch)

```python
FORCE_SETPOINT = 20.0
AUTO_GRIP_P_GAIN = 0.01
AUTO_GRIP_DEADBAND = 4.0
AUTO_GRIP_BREAK_THRESHOLD = 3.0
GRIP_SPRING_SCALAR = 1.0
MAX_INTENTIONAL_SQUEEZE = 2.0
```

---

## Known Remaining Behavior

- Small residual bounce at contact is from the servo's hardware control loop, not Python-addressable.
- The force sensor response is nearly a step function over ~3 units of travel, making exact force regulation at a setpoint inherently difficult without physical compliance (foam, flexible jaw tip).
