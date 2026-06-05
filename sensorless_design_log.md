# Sensorless Haptic Feedback — Design Log

## Goal

Explore whether the SO-101 follower gripper motor's own bus signals (`Present_Current`, `Present_Load`) can provide useful haptic feedback to the operator via the gimbal motor, without a dedicated force sensor.

---

## Phase 1: Basic Signal → Torque

**Concept:** Sign of torque from `Present_Load` (direction of servo effort), magnitude from `Present_Current` (how hard the servo is working).

```python
torque = -copysign(SCALAR * current, load)
```

**Result:** Worked — noticeably felt resistance when gripping. Felt jumpy and vibratory (~10 Hz oscillations).

---

## Phase 2: EMA Smoothing on Input Signals

**Problem:** `Present_Load` and `Present_Current` have ~10 Hz oscillations from the servo's own PID hunting. These fed directly into the gimbal torque as vibration.

**Fix:** Exponential moving average (α=0.2) on both input signals before computing torque. At 60 Hz, this gives ~2 Hz cutoff — well below the 10 Hz noise.

```python
smooth_current += α * (current - smooth_current)
smooth_load    += α * (load    - smooth_load)
```

**Result:** Much smoother feel.

---

## Phase 3: Gripper Velocity Weighting

**Problem:** Oscillation when releasing the gimbal suddenly. Root cause: the bilateral loop (gimbal position → gripper command → servo effort → gimbal torque → gimbal position) becomes unstable with no hand damping it.

Also: movement current (gripper closing in free space) is similar in magnitude to light contact current. Need to distinguish them.

**Fix:** Compute gripper velocity from successive position readings. Apply a soft gate: high velocity → attenuate torque (likely motion current, not contact force). Near-stationary → full torque.

```python
gripper_vel_weight = 1 / (1 + GRIPPER_VELOCITY_K * |gripper_velocity|)
torque = -copysign(SCALAR * gripper_vel_weight * smooth_current, smooth_load)
```

`GRIPPER_VELOCITY_K = 0.05`. At 60 Hz with 0–100 gripper range (full travel ~1 second), typical free-motion velocity is 50–80 units/s → weight ~0.2–0.3.

**Result:** Better feel. Still oscillated when released.

---

## Phase 4: EMA Smoothing on Output Torque

**Problem:** ~20 Hz vibration in the torque output remained despite input smoothing.

**Fix:** EMA (α=0.3) applied to the final torque before writing to the gimbal. At 60 Hz, this gives ~14 dB rejection at 20 Hz.

```python
smooth_torque += TORQUE_SMOOTH_ALPHA * (raw_torque - smooth_torque)
```

**Result:** 20 Hz vibration eliminated. A new problem appeared: 1 Hz large-amplitude oscillation when releasing the gimbal.

---

## Phase 5: Attempted Gimbal Velocity Damping (Failed)

**Attempt:** Add a damping torque `= +k * gimbal_velocity` to resist free swinging. 

**Problems:**
- First attempt had the sign wrong (positive feedback instead of damping) — made it massively worse.
- After fixing the sign: still worse than without damping, because raw gimbal velocity (computed from successive position reads at 60 Hz) is too noisy. Encoder quantization noise at 60 Hz translates to large spurious velocity spikes that injected high-frequency noise into the torque.

**Lesson:** Additive velocity-based torque is sign-sensitive and noise-amplifying. Abandoned.

---

## Phase 6: Gimbal Velocity as Multiplicative Gate (Partial Fix)

**Insight:** Use gimbal velocity not as an additive torque (sign-sensitive, noise-dangerous) but as a multiplicative gain reduction — same structure as the gripper velocity weight. Noise just causes extra attenuation, which is safe.

```python
smooth_gimbal_vel += 0.3 * (gimbal_vel - smooth_gimbal_vel)
gimbal_vel_weight = 1 / (1 + GIMBAL_VELOCITY_K * |smooth_gimbal_vel|)
```

At 1 Hz oscillation with moderate amplitude, peak gimbal velocity ~30–60 units/s. With `GIMBAL_VELOCITY_K=0.2`: weight ~0.08–0.14 → loop gain cut by ~7–12x.

**Result:** Better, but didn't fully eliminate the 1 Hz oscillation.

---

## Phase 7: Current Lockout (Breakthrough)

**Insight:** When the system is truly quiescent — nothing being gripped, no active contact — `smooth_current` should be near zero. There is no reason to drive the gimbal at all. A lockout threshold prevents the idle loop from even starting to oscillate.

```python
if smooth_current < CURRENT_LOCKOUT_THRESHOLD:
    raw_torque = 0.0
```

`CURRENT_LOCKOUT_THRESHOLD = 2.0` (raw counts). Idle smooth current sits well below this; genuine gripping current rises above it.

**Result:** Oscillation on release eliminated completely. Gimbal sits still when not actively gripping.

**Note:** Gimbal velocity gating was then removed (it was no longer needed and added complexity). Removing it caused a mild tendency to oscillate to return.

---

## Phase 8: Velocity-Weighted Lockout (Final Form)

**Problem after removing gimbal velocity gate:** Mild oscillation returned. The issue: current during oscillation can briefly exceed the lockout threshold, re-engaging the torque and sustaining the loop. Also, movement current (gripper opening/closing in free space) can exceed the threshold even without contact.

**Key insight:** The lockout threshold was treating all current equally. But we already knew from the correlation analysis that **high current + high velocity = motion current** (not contact), while **high current + low velocity = contact force**. The threshold should discount current that occurs during fast gripper movement.

**Fix:** Apply the gripper velocity weight to the current *before* comparing against the lockout threshold:

```python
if smooth_current * gripper_vel_weight < CURRENT_LOCKOUT_THRESHOLD:
    raw_torque = 0.0
```

Now only sustained, stationary-gripper current can unlock the feedback. Movement current is discounted and cannot accidentally engage the loop.

**Result:** Excellent stability. No oscillation on release. Clean engagement on contact.

---

## Final Signal Chain

```
Present_Current ──EMA(α=0.2)──► smooth_current ─┐
                                                  ├─ * gripper_vel_weight ──► lockout check
Present_Load ────EMA(α=0.2)──► smooth_load     ─┘                               │
                                                                            if below threshold:
gripper.pos ──── diff*60 ──► gripper_velocity                                 torque = 0
                    │                                                           │
                    └──── gripper_vel_weight = 1/(1 + 0.05*|v|)           else:
                                                                             raw_torque = -copysign(
                                                                               SCALAR * gripper_vel_weight * smooth_current,
                                                                               smooth_load
                                                                             )
                                                                                │
                                                                           EMA(α=0.3)──► gimbal torque
```

## Constants (current values)

```python
CURRENT_TORQUE_SCALAR     = 0.3
CURRENT_LOCKOUT_THRESHOLD = 2.0   # raw counts
SIGNAL_SMOOTH_ALPHA       = 0.2   # input EMA
TORQUE_SMOOTH_ALPHA       = 0.3   # output EMA
GRIPPER_VELOCITY_K        = 0.05  # velocity gate steepness
```

---

## Comparison with Force Sensor Approach

From earlier correlation analysis (`to_sense_or_not_to_sense.md`):
- Force sensor SNR: 41.6 dB vs motor current SNR: 3.6 dB
- Motor current cannot cleanly distinguish contact force from motion force without velocity context
- Force sensor is zero-baseline; motor current requires baseline tracking

The sensorless approach compensates for these limitations through velocity weighting and the current lockout, but is fundamentally working with a noisier, less direct signal. The force sensor approach supports more sophisticated control (closed-loop P controller, force setpoint regulation). The sensorless approach is simpler hardware and surprisingly functional for basic contact awareness.
