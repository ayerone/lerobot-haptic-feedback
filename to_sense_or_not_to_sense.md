# To Sense or Not to Sense: Force Sensor vs Motor Signals for Haptic Feedback

## Question

Is there value in using a dedicated force sensor for haptic feedback teleoperation, or could the gripper motor's own reported signals (Present_Current, Present_Load) serve as a proxy?

---

## Signal Characteristics

### Force Sensor
- Measures contact force directly at the jaw — zero until contact, then rises with squeeze pressure
- Independent of motor speed, gear friction, temperature, or control effort
- Normalized to 0–100 scale

### Present_Current
- Proportional to motor torque output (~6.1 mA/count empirically)
- Reflects combined load from: actual gripping force, gear friction, mechanical preload, inertia during motion, temperature drift
- Nonzero even when not gripping (servo holding position against friction)
- Noisy idle baseline

### Present_Load
- Signed PID PWM output (percentage of max torque, directional)
- Represents control effort, not force
- Extremely noisy at idle — swings widely even while holding position

---

## Experimental Results

**Setup:** SO-101 follower arm teleoperated with FeedbackLeader. Force sensor, motor current, and motor load logged simultaneously at ~60 Hz during 43 seconds of active gripping (9 contact episodes).

```
SAMPLES: 2515   DURATION: 43.0s   RATE: 61.0 Hz
IDLE: 66%   CONTACT: 34%   Episodes: 9

BASIC STATS
                   force   curr_mA      load
  min                0.0       0.0    -292.0
  max               59.4     274.5     196.0
  mean              16.2      44.5      24.6
  std               23.0      44.0      75.8

IDLE BASELINE (force < 2)
  curr_mA:  25.4 ± 29.3   (CV > 100%)
  load:      3.3 ± 78.6

GRIPPING (force > 5)
  force     47.8 ± 8.5
  curr_mA   81.8 ± 44.2  (baseline-sub: 56.3 ± 44.2)
  load      66.8 ± 46.5

CORRELATION (Pearson, gripping only)
  force vs curr_mA:  r = 0.423
  force vs load:     r = 0.193

SNR (gripping_variance / idle_variance)
  force:    41.6 dB
  curr_mA:   3.6 dB
  load:     -4.6 dB

CONTACT ONSET TIMING (motor signal vs force threshold crossing)
  curr_mA  median: -0.119s  mean: -0.218s  min: -0.496s  max: 0.000s
  load     median: +3.571s  mean: +6.508s  (unreliable — noise-driven)
```

---

## Conclusions

### Force sensor is essential for closed-loop control

The force sensor has 41.6 dB SNR vs 3.6 dB for motor current — approximately 38 dB cleaner. The idle current baseline has a coefficient of variation over 100% (std=29.3 on mean=25.4), meaning it would introduce enormous noise into a P controller. Motor load is even worse: negative SNR means it is *more* variable at idle than during active gripping. The P controller for force regulation requires the dedicated force sensor.

### Motor load is not useful

Negative SNR and wildly noisy idle behavior (std=78.6 on mean=3.3) make it unreliable for any purpose. The PID's PWM output swings continuously just to hold position. Contact onset timing was median +3.57s with a max of +16.7s — those are noise-triggered false detections, not real contact signals.

### Motor current shows early movement, not early contact

Motor current crossed its threshold before the force sensor in every episode (median -119ms, never after). However, this lead is almost entirely **motion current** — the motor draws current as soon as it starts closing the jaw, whether approaching an object or moving in free air. This always precedes contact regardless of whether an object is present.

To extract a genuine early-contact signal from current, you would need to isolate the *load component* — the excess current above what the motor is already drawing for motion. This requires a model of expected current given gripper velocity, and given the noisy baseline, would likely not be reliable enough to act on.

### What the force sensor provides that motor current cannot

The force sensor measures contact force independently of what the motor is doing. There is no clean way to separate load current from motion current without a significantly more sophisticated motor model. The force sensor is doing something fundamentally different, not just doing the same thing more precisely.

---

## Implication for Haptic Feedback Design

The dedicated force sensor is justified and should remain the primary feedback signal. The automatic P controller (`GripFeedbackController`) that regulates grip force at a setpoint of 20 (0–100 scale) depends on a clean, zero-baseline force signal that only the sensor provides.

A potential future enhancement: use a rising motor current (above a rolling idle baseline) as a soft "contact imminent" signal to slow gripper closure speed before the force sensor registers — reducing the initial force spike that causes bounce-back. This would be a pre-contact slowdown heuristic, not a replacement for force feedback.
