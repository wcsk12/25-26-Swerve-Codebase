# Team 8430 — Controller Button Layout

This document describes the controller mappings implemented in `RobotContainer.java` (extracted directly from the code). Use this as the canonical reference for driver/operator bindings and fallbacks.

## Ports
- Driver controller port: `OIConstants.kDriverControllerPort` = 0
- Operator controller port: `OIConstants.kOperatorControllerPort` = 1
- Deadbands: `OIConstants.kDriveDeadband` = 0.05 (drive axes), `kEndEffectorDeadband` = 0.5

## Default drive (driver Xbox / joystick fallback)
- Default command: `DriveSubsystem.drive(...)` is the default command (field-relative = true).
- Axes used (Xbox controller):
  - Left stick Y (axis 1): forward/backwards (throttle) — inverted in code (`-getRawAxis(1)`).
  - Left stick X (axis 0): strafe (left/right).
  - Right stick X (axis 4): rotation.
- Deadband applied: `MathUtil.applyDeadband(..., OIConstants.kDriveDeadband)`
- Drive call uses a 0.02 time step hard-coded in the default `RunCommand`.

## Driver controller (Xbox) — port 0
- A (button A)
  - while held: `AutoAlignCommand(m_robotDrive, 0.6)` — auto-align to AprilTag (0.6m target distance)
  - NOTE: the PathPlanner `NamedCommand` `"Align"` is registered to use `AutoAlignCommand.Mode.FULL_ALIGN` (autos will do full translation+rotation alignment). The driver A binding still uses the rotation-only default constructor.
- B (button B)
  - toggleOnTrue: `LowerSpeedCMD(m_robotDrive, 2)` — toggle a lower-speed mode (toggle behaviour).
- Left bumper (LB)
  - while held: `IntakeCMD(m_OtherMotorsSubsystem, DriveConstants.IntakeMotorSpeed, true)` — intake forwards.
- Right bumper (RB)
  - while held: `IntakeCMD(..., false)` — intake reversed / outtake.

## Operator controller (Xbox) — port 1
- Left bumper (LB)
  - while held: `ReleaseCMD(m_OtherMotorsSubsystem, DriveConstants.ReleaseMotorSpeed)` — run release/feeder motor.
- Right bumper (RB)
  - toggleOnTrue: currently bound to `new InstantCommand(() -> new ShooterCMD(m_ShooterSubsystem, DriveConstants.ShooterMotorSpeed, false))` in code; joystick fallback toggles `ShooterCMD` directly which schedules properly.
- Start (menu) button
  - onTrue: triggers `CANChecker.runChecks()` if robot is disabled (safety guarded) — useful for hardware probing when Shuffleboard isn't available.

## Joystick fallbacks (raw Joystick on same ports)
These provide compatibility for flight-sim or non-Xbox controllers connected to the same ports.
- Driver Joystick (port 0)
  - Button 1: whileTrue `AutoAlignCommand(m_robotDrive, 0.6)` (A fallback)
  - Button 2: toggleOnTrue `LowerSpeedCMD` (B fallback)
  - Button 5: whileTrue `IntakeCMD(..., true)` (left button — intake forwards)
  - Button 6: whileTrue `IntakeCMD(..., false)` (right button — intake backwards)
- Operator Joystick (port 1)
  - Button 5: whileTrue `ReleaseCMD(... )` (release)
  - Button 6: toggleOnTrue `ShooterCMD(...)` (shoot)

## PathPlanner NamedCommands (for autos / PathPlanner integration)
The following NamedCommands are registered and therefore usable from PathPlanner autos and the AutoChooser:
- `"Align"` → `AutoAlignCommand(m_robotDrive, 0, AutoAlignCommand.Mode.FULL_ALIGN).withTimeout(1)`
- `"Intake!"` → `IntakeCMD(...).withTimeout(2)`
- `"Shoot!"` → `getShootSequence()` (a sequential command group handling spin-up, release, and stop)

Note: Because `"Align"` now uses `Mode.FULL_ALIGN`, autos that call `Align` will attempt to translate and rotate to the target automatically. The driver A binding still uses rotation-only behavior by default.

## Notes / Gotchas discovered in code
- Hardware singletons: subsystems instantiate motor controllers/hardware in constructors. Do not create SparkMax/Pigeon objects outside subsystems — use `RobotContainer` getters to reuse instances.
- CAN Checker safety: `CANChecker.runChecks()` is guarded to only run while the robot is disabled; Shuffleboard and controller triggers both enforce this.
- Potential bug: operator Xbox RB uses an InstantCommand that simply constructs a `ShooterCMD` instead of scheduling it. The operator joystick fallback toggles the `ShooterCMD` directly and is likely the intended behavior. Consider updating the Xbox binding to schedule the `ShooterCMD` directly (e.g., `m_operatorController.rightBumper().toggleOnTrue(new ShooterCMD(...));`) if you want consistent behaviour.

## Want changes?
If you want this as an image or printable cheat-sheet for drivers, or want me to change the Xbox `ShooterCMD` binding to the working pattern used in the joystick fallback, tell me which change to make and I can apply the code edit and run a quick build.
