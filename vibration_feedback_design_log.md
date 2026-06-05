# Vibration Feedback Design — `main` branch

## Overview

The original `send_feedback` implementation was a simple proportional torque write. Over time it grew into a richer vibration-based feedback system living in `GripFeedbackController`. This documents the design of that system as it exists on `main`.

---

## Signal Structure

The vibration magnitude sent to the gimbal motor is the sum of two terms:

```python
steady_term = GRIP_FEEDBACK_SCALAR * sqrt(max(0, force - VIBRATION_ONSET_FORCE))
derivative_term = CONTACT_DERIVATIVE_SCALAR * derivative_envelope
magnitude = steady_term + derivative_term
```

### Steady Term

Proportional to `sqrt(force - onset)`. The square root keeps the response from becoming overwhelming at high forces while still scaling with load. `VIBRATION_ONSET_FORCE = 5` creates a small deadband below which the steady term is zero — light touches don't vibrate.

### Derivative Term

Captures the *onset* of contact — the sudden jump in force when the gripper first touches an object. Uses a leaky-peak envelope:

```python
d_force = force - last_force
derivative_envelope = max(max(0, d_force), derivative_envelope * DERIVATIVE_DECAY)
```

`DERIVATIVE_DECAY = 0.9` lets the envelope decay smoothly after the spike rather than cutting off abruptly. This gives a distinct "thud" feel at contact that fades as force stabilizes.

---

## Two Gripping Regimes

### Normal Gripping (`force > SENSOR_DEADBAND_THRESHOLD`)

Vibrate with `magnitude`. No position restore, no clamp.

### Above Force Limit (`force > FORCE_LIMIT_THRESHOLD`)

Three additional behaviors activate when force exceeds 30:

1. **Grip clamp:** `_grip_clamp_position` is set to the gimbal position at the moment force first exceeds the limit. `get_action` enforces `gripper.pos >= clamp_position`, preventing the robot's jaw from opening further.

2. **Gimbal restore:** Computes `gimbal_drift = clamp_position - gimbal_pos`. If the gimbal has drifted more open than the clamp position, a restore force `center = -GIMBAL_RESTORE_SCALAR * gimbal_drift` is passed to `vibrate()` as an offset, nudging the feedback motor back toward the clamp position.

3. **Continued vibration:** Same `steady_term + derivative_term` magnitude as normal gripping.

---

## No-Contact / Jaw-Open Spring

When force is below `SENSOR_DEADBAND_THRESHOLD`, all gripping state is reset and a jaw-open spring activates:

```python
error = gimbal_pos - gripper_pos
if error > TELEOP_EFFECTOR_TOO_OPEN_THRESHOLD:
    feedback_motor.write(JAW_OPEN_SCALAR * error)
```

When the teleop is opened significantly wider than the robot's gripper, a restoring torque pushes the gimbal back toward closed. This prevents the operator from accidentally leaving the teleop far open relative to the follower.

---

## Motor Interface

`FeedbackMotor.vibrate(magnitude, center=0.0)` sends a `VIBRATE <magnitude> <center>` serial command to the Arduino, which drives the gimbal motor with an oscillating torque offset by `center`. `FeedbackMotor.write(value)` sends a steady torque.

The `FeedbackCommand` dataclass carries the decision out of `GripFeedbackController`:

```python
@dataclass
class FeedbackCommand:
    value: float
    vibrate: bool = False
    center: float = 0.0
```

`send_feedback` dispatches accordingly:

```python
if cmd.vibrate:
    feedback_motor.vibrate(cmd.value, center=cmd.center)
else:
    feedback_motor.write(cmd.value)
```

---

## Constants

```python
SENSOR_DEADBAND_THRESHOLD = 2
VIBRATION_ONSET_FORCE = 5
FORCE_LIMIT_THRESHOLD = 30
GRIP_FEEDBACK_SCALAR = 1 / 30
CONTACT_DERIVATIVE_SCALAR = 3
DERIVATIVE_DECAY = 0.9
GIMBAL_RESTORE_SCALAR = 0.05
TELEOP_EFFECTOR_TOO_OPEN_THRESHOLD = 15
JAW_OPEN_SCALAR = 0.01
```

---

## Status

Vibration was disabled on `spike_automatic` and the vibration code (`_compute_vibration_magnitude`, `FeedbackCommand.vibrate`/`.center`, related constants) was removed when that branch merged to `main`. The design above is preserved here and in git history for reference.
