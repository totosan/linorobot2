# G3 Raw-Odometry Yaw Bias: Analysis (09.10.2026)

Scope: bound the documented G3 raw wheel-odometry yaw deviation using existing
runtime reports and the model equations. No product parameter was changed; one
bounded hypothesis probe (`wheel_mu=2.0`) was run as a G2 test variant.

## Measured evidence

| Evidence | Value | Source |
| --- | --- | --- |
| G3 turn ratio raw/GT (4 turns) | **1.21431**, spread < 10⁻⁵ | `g3-safety-20261006-121252/phase3.json` |
| G2 turn ratio, +0.30 rad/s | 1.2153 | `g2-motion-20261006-112608/motion.json` |
| G2 turn ratio, −0.30 rad/s | 1.2136 | same |
| G2 straight translation | raw/GT ≈ 0.9997 (no linear bias) | same |
| IMU integrated yaw vs GT | matches within 0.02 rad | phase3.json |
| Filtered EKF yaw vs GT | matches (EKF uses IMU yaw rate) | phase3.json |
| Mean joint velocity in turns | 0.559 rad/s = commanded 0.56 rad/s | phase3.json |
| Probe with `wheel_mu=2.0` | ratio **unchanged** (1.2136–1.2153) | `g2-motion-20261009-161221/motion.json` |

Analysis outputs are reproducible via
`scripts/analyze_g3_yaw_bias.py` and stored under `log/g3_analysis/`
(`yaw-bias-baseline.json`, `yaw-bias-wheel-mu-2.json`).

## Model-equation check (Fortress `ign-gazebo6` DiffDrive)

The versioned plugin source
([DiffDrive.cc](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/src/systems/diff_drive/DiffDrive.cc))
confirms: command mapping `jointSpeed = (v ± w·S/2)/r`, joints driven through
`JointVelocityCmd`, odometry integrated from **joint positions** with
`DiffDriveOdometry.SetWheelParams(S=0.126, r=0.03375, r)`. The 2WD model places
the wheel joints at y=±0.063 m (separation 0.126 m) and the odometry child frame
`base_drive` sits on the wheel axle; a longitudinal axle offset cannot create a
yaw bias, and the measured joint speeds track the commands.

Consequently the raw path is internally consistent: raw yaw rate ≈ commanded
0.30 rad/s, while the body physically achieves ≈0.245 rad/s (0.817×) during
scrub turns. The "bias" is the **body under-rotating relative to wheel
kinematics**, so encoder odometry overestimates yaw by 1/0.8235 = 1.2143.

## Ruled out / remaining hypotheses

- **Ruled out — encoder/software/unit/frame error:** IMU and EKF match GT;
  joints track commands; odometry integrates joints with the documented
  geometry.
- **Ruled out — wheel Coulomb friction:** doubling `wheel_mu` (1.0→2.0) changed
  the ratio by <0.2% (within run-to-run scatter).
- **Ruled out — caster drag:** the caster sphere already runs at mu 0.01.
- **Remaining hypothesis:** the Fortress/DART (DART 6.12, ODE collision)
  contact-constraint solution for velocity-enforced cylinder wheels during skid
  turns permits a constant slip rate (constraint softness), independent of mu.
  A geometry-only alternative (effective separation 0.153 m vs 0.126 m) has no
  counterpart in the model and is inconsistent with mu-independence.

## Hardware-transfer implication

The factor is a **simulator artifact candidate**, not a hardware calibration
value. It must not be exported to firmware or used as a physical expectation.
Hardware wheel-odometry yaw requires its own measured calibration in G8.

## Discriminating follow-up experiments

1. **Physics-step probe run 15.10.2026:** Baseline `0.001 s` produced raw/GT
   `1.213581…1.215269`; `0.0005 s` produced `1.213636…1.215400`. No measurable
   shift occurred in this bounded run.
2. **Caster-removed probe run 15.10.2026 — inconclusive:** The base dropped to
   `z≈−0.0125 m`, pitched `0.176…0.189 rad`, and achieved zero Ground-Truth yaw
   in the first turn. Removing the support introduced chassis/ground contact,
   so the result does not isolate wheel constraints.
3. **Revised contact probe:** retain the caster geometry but set its already
   low friction from `0.01` to `0.0`. Run `g2-motion-20261015-081539` passed
   all bounded motion and watchdog checks. Raw/GT remained
   `1.213363…1.215087`, versus baseline `1.213581…1.215269`. Roll/Pitch stayed
   below `1.9×10⁻⁶ rad`; the base-Z range remained
   `−0.000861…0.000000 m`, so support geometry remained valid. Caster
   tangential friction does not explain the yaw deficit.
4. If needed, instrument contact slip velocities in a DART-level debug build.

## Follow-up conclusion

The bounded discriminators strengthen the Fortress/DART wheel-contact
constraint hypothesis: halving the physics step and removing caster friction
did not measurably change the factor, while removing the support geometry was
physically invalid. No simulator factor is transferred to hardware. Further
DART-level slip instrumentation is optional research, not an open G3 gate.
