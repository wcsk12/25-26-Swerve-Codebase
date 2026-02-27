# Operator Cheat Sheet — Swerve Robot (one page)

Quick reference for drivers & operators. Use this during practice/competition. All mappings come from `src/main/java/frc/robot/RobotContainer.java`.

## Driving (Driver Xbox / Driver Joystick fallback)
- Left stick (Axis 1): Forward / Back
- Left stick (Axis 0): Strafe left / right
- Right stick (Axis 4): Rotation
- Deadband: `OIConstants.kDriveDeadband` (~0.05)
- Default drive mode: field-relative (joystick input interpreted relative to the field)

Speed modes (driver bumpers)
- Right bumper pressed: 0.5× speed (hold)
- Right bumper released: back to 1.0×
- Left bumper pressed: 0.25× speed (hold)
- Left bumper released: back to 1.0×

Auto-align
- Driver A (while held): run `AutoAlignCommand(m_robotDrive, 0.6)` — vision-assisted align to AprilTag (0.6 m target).

Fallback joystick (if not using Xbox controller)
- Joystick button 1 mapped to same `AutoAlignCommand` as Driver A (while held).

## Operator (Operator Xbox / Operator Joystick fallback)
- A (while held): Intake in (`IntakeCMD(miscSubsystem, intakeMotorSpeed)`)
- B (while held): Intake out (reverse)
- Left Trigger (≥0.5) (while held): Shooter using limelight-proportional speed (`ShooterCMD(..., Robot.limelight_range_proportional())`)
- Left Bumper (while held): Shooter fixed speed fallback (`DriveConstants.shooterMotorSpeed`)
- Right Trigger (≥0.5) (while held): Launcher (and temporarily indexer) — sends fuel to shooter
- X (toggle): Retract pos-intake (toggle on → set position zero; toggle off → stop)
- Y (toggle): Lower pos-intake (toggle on → set position low; toggle off → stop)

Fallback operator joystick buttons (non-Xbox)
- Button 2: Intake in
- Button 3: Intake out
- Button 7: Shooter (limelight)
- Button 5: Shooter (fixed)
- Button 8: Launcher & Indexer (temp)
- Button 1: pos-intake toggle zero
- Button 4: pos-intake toggle low

## Autonomous / PathPlanner
- PathPlanner autos are provided in `src/main/deploy/pathplanner/paths` and `autos/`.
- Small action commands used in autos are registered in `RobotContainer` via `NamedCommands.registerCommand("Name", command)` (e.g., `IndexerCMD`, `IntakeCMD`, `ShooterCMD`, `LauncherCMD`).
- The Auto chooser is on SmartDashboard as `Auto Chooser` and filters autos optionally for competition mode.

## Vision & Shooter notes
- Limelight is read from NetworkTables table named `limelight`.
- Shooter auto-speed uses `Robot.limelight_range_proportional()`; left trigger runs vision-proportional control, left bumper runs fixed speed as fallback.

## CAN checks & safety
- `CANChecker.runChecks()` is available to probe Spark MAX CAN IDs.
- CAN Checker triggers:
  - Shuffleboard toggle/button on the "CAN Checks" tab (must be disabled to run — checker enforces robot disabled state).
  - Operator controller Start button (fallback) — also requires robot disabled.
  - A scheduled manual toggle is polled periodically by the robot and will run the check when true (also requires disabled state).

Important: The Pigeon2 gyro uses CAN ID 12. Do NOT assign Spark MAXs to CAN ID 12.

## Quick troubleshooting tips
- If odometry/field pose looks flipped, check gyro sign usage: code uses `Rotation2d.fromDegrees(-m_Pigeon2.getYaw().getValueAsDouble())` (negated yaw).
- If PathPlanner behavior is wrong, ensure `pathConfig` settings in GUI match the deployed `src/main/deploy/pathplanner/settings.json` and robot dimensions in `Constants`.
- If controllers aren't responding, check driver/operator port mapping in `Constants.OIConstants` (driver 0, operator 1) and fallback Joystick bindings.

## Where to find more
- Button mappings & default drive: `src/main/java/frc/robot/RobotContainer.java`
- Swerve implementation & odometry: `src/main/java/frc/robot/subsystems/DriveSubsystem.java`
- CAN IDs & constants: `src/main/java/frc/robot/Constants.java`

---
If you'd like a printable PDF version or a version with team-specific tuning numbers filled in (max speeds, shooter RPM), tell me which values to include and I'll generate it.
