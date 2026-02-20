# FRC Team 8116 Hatchet Robotics and FRC Team 8430 The Hatch Batch

## Project Overview
This repository contains the 2026 swerve drive codebase used by FRC Team 8116 (Hatchet Robotics) and FRC Team 8430 (The Hatch Batch). It provides the robot-side software to control a swerve-drive chassis during teleoperated and autonomous operation, including kinematics, odometry, trajectory following, and utilities for calibration and tuning.

## What this code does for an FRC robot
This project implements a complete swerve-drive software stack enabling independent steering and driving for each wheel. High-level features:
- Convert driver/auto commands into wheel speeds & azimuth angles
- Estimate robot pose (x, y, heading) on the field in real time
- Execute field-relative teleop and precise autonomous trajectories
- Abstract hardware so code can be reused across motor/controller types
- Provide tooling for calibration, safety limits, and simulation

## Subsystems — what each one does
Below are the typical subsystems found in `src/` and an explanation of their responsibilities. File/class names in your repo may vary; use these descriptions to map to your implementation.

- SwerveModule (per-module)
  - Controls one wheel module: drive motor, steer motor, and encoders (absolute & relative).
  - Exposes `setDesiredState(speedMetersPerSecond, angleRadians)` to command module velocity and heading.
  - Handles low-level conversion (encoder ticks ↔ meters/radians), motor inversion, and angle wrap/offset calibration.
  - Supports calibration routines to detect and store absolute encoder offsets.
  - Safety: handles soft-stops, idle modes, and angle normalization to avoid unnecessary rotation.

- Drivetrain / SwerveSubsystem
  - Aggregates all SwerveModules and implements WPILib SwerveKinematics and SwerveDriveOdometry.
  - Provides higher-level APIs: `drive(chassisSpeeds, fieldRelative)`, `setModuleStates(...)`, `getPose()`, `setPose()`, `zeroGyro()`.
  - Coordinates periodic updates: compute module states, apply feedforward + PID to drive motors, and run odometry.
  - Contains safety helpers such as module locking (X-lock), neutral modes, soft speed limits, and emergency-stop handling.
  - Exposes helpers for trajectory following (resetting odometry, setting heading setpoints, running path-follow loops).

- Gyro / IMU Subsystem
  - Reads heading/yaw/pitch/roll from the robot’s IMU (NavX, Pigeon, etc.).
  - Exposes `getYaw()`, `getRotation2d()`, `reset()` and filtered values used by odometry and balance routines.
  - Provides utilities for fused heading, quick-zero routines, and pitch-based balancing (e.g., auto-balance).

- Vision Subsystem (if present)
  - Integrates cameras, LimeLight, or other vision systems.
  - Provides target detection, distance/angle offsets, and estimated robot pose corrections (vision pose estimator or AprilTag support).
  - Supplies data used by alignment and auto-aim commands; can provide measured pose deltas used to correct odometry.

- Intake Subsystem
  - Controls motors and solenoids used to pick up game pieces.
  - Exposes simple commands: `intakeIn()`, `intakeOut()`, `stop()`.
  - Uses sensors (beam-break, current-sensing) to detect possession and protect motors from jams.

- Indexer / Conveyor Subsystem
  - Manages staged movement of game pieces from intake to shooter or storage.
  - Reads sensors to sequence motion, prevent double-feeds, and coordinate with shooter spin-up.

- Shooter Subsystem
  - Controls flywheels, hood actuators, and RPM closed-loop control.
  - Supports `spinUpToRPM(rpm)`, `holdRPM()`, `setHoodAngle(angle)`.
  - Integrates with indexer: readiness checks, timed feeds, and sensor-driven shoot cycles.

- Arm / Elevator Subsystem
  - Provides position control (profiled PID + feedforward) for manipulators.
  - Safety interlocks, limit switches, and soft limits to prevent hardware damage.
  - Often includes manual override while disabled or in maintenance modes.

- Climber Subsystem
  - Controls mechanisms used for endgame climbing (deploy, extend, lock).
  - Includes multi-step state machines, limit switch checks, and motion profiles for safe sequencing.

- LED / Status Subsystem
  - Displays system state (disabled, teleop, auto, fault) using LED strips or indicators.
  - Helpful for quick diagnostics on the field (errors, connection state, shooter ready).

- Simulation / Test Utilities
  - Simulation hooks mirror hardware calls so kinematics, odometry, and controllers can be exercised off-robot.
  - Unit tests target math utilities, conversions, and controller behavior to prevent regressions.

## Commands — purpose and typical behavior
This project uses (or is designed to support) the WPILib command-based framework. Typical commands and what they do:

- DriveWithController (default)
  - Default command for the Drivetrain. Reads joystick/gamepad axes, computes chassis speeds, and calls `drive(...)`.
  - Supports toggles: field-relative vs robot-relative, precision/slow mode, deadband, and throttle scaling.

- FollowTrajectoryCommand / SwerveAuto (auto sequences)
  - Wraps WPILib `SwerveControllerCommand` to follow generated trajectories with translation and heading control.
  - Uses feedforward + PID on module speeds and rotation controllers for heading control.
  - Composed into sequential/parallel command groups for complex autos (drive → aim → score → retreat).

- AlignToTarget / VisionAlign
  - Uses vision processing to generate lateral and angular setpoints.
  - Runs PID on heading and optionally lateral translation until within target tolerances; can finish with a small hold.

- ZeroGyroCommand
  - Resets IMU heading and optionally zeroes odometry; typically run at robot init or a dedicated button.

- CalibrateModuleAngles (per-module)
  - Drives modules to a reference, reads absolute encoder, computes offset, and writes that offset to persistent storage or constants for future runs.

- XLock / LockWheels
  - Sets module azimuths to form an X (or other pattern) to maximize resistance to external pushes (useful when defending or during endgame).

- RunIntake / EjectIntake
  - Runs intake motors to pick up or expel game pieces; often run as button-hold commands tied to operator inputs.

- SpinUpShooter / ShootSequence
  - Spins flywheel to target RPM, verifies stable speed, then runs indexer in short bursts to feed; may include hood adjustment and sensor-based confirmation.

- AutoBalance
  - Drives onto a charging station and uses pitch to actively balance the robot; typically a stateful controller that approaches, enters, oscillation-damps, and holds.

- EmergencyStop / SafeStop
  - Immediately commands zero outputs to motion-critical subsystems, engages brakes/neutral modes, and sets a safe state for testing or fault conditions.

## Command-based organization notes
- RobotContainer (or equivalent) sets up subsystem instances, button bindings, and the autonomous chooser.
- Subsystems should expose small, testable actions (e.g., `setModuleStates`, `setTargetRPM`) while commands compose those into behaviors.
- Keep tuning constants in `Constants` or a similarly named central file for easy adjustments between practice/competition robots.

## Project Structure
- `src/` - Main robot code: subsystems, commands, constants, and the robot entry point.
- `docs/` - Project docs, wiring diagrams, and calibration notes.
- `tests/` - Unit and integration tests.
- `bin/` - Binary artifacts or dev scripts.
- `build.gradle`, `settings.gradle`, `gradlew` - Gradle build and wrapper files.
- `WPILib-License.md` - WPILib license information.
- `USE START BRANCH` - Team branching guidance.

## Prerequisites & Getting Started
- Java 11 (or WPILib-required version)
- Gradle wrapper included: `./gradlew`
- WPILib installed for your robot and driver station
- RoboRIO and hardware configured

Quick start:
```bash
git clone https://github.com/wcsk12/25-26-Swerve-Codebase.git
cd 25-26-Swerve-Codebase
./gradlew build
./gradlew deploy -ProborioAddress=<robot_ip>
```

## Calibration & Tuning Checklist
1. Confirm drivetrain physical constants (wheelbase, trackwidth, wheel radius).
2. Verify motor/encoder directions and inversion.
3. Calibrate each module's absolute encoder offset.
4. Tune drive & steer PID loops; start conservative and increase gradually.
5. Tune feedforward (kS, kV, kA) for velocity profiles.
6. Validate odometry by driving known paths and comparing pose estimates.
7. Test safety limits (current, velocity) and emergency stop behavior.

## Notes & Team Files
- `USE START BRANCH` contains branching guidance—follow it for feature and release workflow.
- `WPILib-License.md` includes WPILib licensing details.
- Avoid committing private credentials or robot-specific secret files.

## License
This project follows WPILib licensing guidance and team-specific licensing decisions. See `WPILib-License.md` and check with team leads for distribution policies.
