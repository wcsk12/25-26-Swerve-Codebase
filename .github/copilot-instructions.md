## Purpose
This repo is an FRC robot project (WPILib + PathPlanner + CTRE/REV vendor libraries). These instructions give focused, actionable guidance for an AI coding agent to be productive immediately.

## Big picture (what to know first)
- Command-based robot (WPILib): subsystems live under `src/main/java/frc/robot/subsystems/` and commands under `src/main/java/frc/robot/commands/`.
- `RobotContainer.java` wires subsystems, default commands, and trigger bindings. See examples of button wiring, default drive RunCommand, and PathPlanner NamedCommands registration.
- `DriveSubsystem.java` implements a MAX-based swerve drive, uses a `Pigeon2` gyro (CAN ID 12 reserved), and integrates PathPlanner's AutoBuilder for path following.
- `Constants.java` centralizes robot-wide values (CAN IDs, kinematics, speeds). Prefer updating constants here for hardware/config changes.

## Key files to reference
- `src/main/java/frc/robot/RobotContainer.java` — bindings, default drive command, PathPlanner NamedCommands registration.
- `src/main/java/frc/robot/subsystems/DriveSubsystem.java` — swerve implementation, odometry, field visuals, PathPlanner AutoBuilder hooks.
- `src/main/java/frc/robot/Constants.java` — CAN IDs and kinematics (note: DO NOT use CAN ID 12 for Spark MAX; pigeon is on 12).
- `src/main/deploy/pathplanner/` — PathPlanner JSONs and autos (named paths referenced by `NamedCommands.registerCommand`).
- `build.gradle` — GradleRIO + WPILib configuration; jar and deploy are configured here.

## Project-specific patterns and conventions
- Command-based with heavy use of inline `InstantCommand`, `RunCommand`, `.whileTrue()` and `.toggleOnTrue()`. Follow the same style when adding small commands.
- PathPlanner integration: register small action commands with `NamedCommands.registerCommand("Name", command)` in `RobotContainer` so path JSONs can reference them.
- CAN checker: `CANChecker.runChecks()` is used at startup or via Shuffleboard/Controller. The project exposes a Shuffleboard toggle and a controller Start-button fallback — prefer those for hardware probes.
- Shuffleboard/SmartDashboard are used extensively for telemetry; add new telemetry keys in the same pattern (e.g., `SmartDashboard.putNumber("Pose X (m)", pose.getX())`).

## Build / test / deploy (PowerShell examples)
- Build: `.\\gradlew.bat build` — compiles, creates the fat jar.
- Run tests: `.\\gradlew.bat test` (JUnit 5 is configured).
- Deploy to RoboRIO: `.\\gradlew.bat deploy` (GradleRIO reads the team number from WPILib preferences; override with `-Pteam=####` if needed).
- The `deploy` block in `build.gradle` places files from `src/main/deploy` to `/home/lvuser/deploy` on the roboRIO.

## Editing hardware/config
- Update `Constants.DriveConstants` for CAN IDs, kinematics and max speeds. Many subsystems read values from `Constants` at construction.
- PathPlanner robot config is loaded from `pathConfig` — changing robot physical dimensions should be coordinated between `Constants` and `src/main/deploy/pathplanner` assets.

## Integration points & external deps
- WPILib and GradleRIO (see `build.gradle`). Vendor libs are under `vendordeps/` and referenced via JSON files in `vendordeps` (PathPlanner, Phoenix/REV libs).
- NetworkTables/Shuffleboard for telemetry. Limelight tables are referenced by name "limelight".

## Examples to copy/paste
- Registering a small action so PathPlanner can call it from a .auto file:
  NamedCommands.registerCommand("IndexerCMD", new IndexerCMD(miscSubsystem, DriveConstants.indexerMotorSpeed).withTimeout(1));
- Default drive wiring (use same deadband/axis mapping):
  m_robotDrive.setDefaultCommand(new RunCommand(() -> m_robotDrive.drive(-MathUtil.applyDeadband(m_driverController.getRawAxis(1), OIConstants.kDriveDeadband), ...), m_robotDrive));

## Do / Don't
- DO update `Constants` for hardware changes and register small PathPlanner actions in `RobotContainer`.
- DO use the existing Shuffleboard/CANChecker widgets for hardware probing instead of inserting ad-hoc probes.
- DON'T change `Main.java` or add static initialization there — WPILib entry is centralized via `RobotBase.startRobot(Robot::new)`.

## Where to ask for human help
- If you need current team number, roboRIO IP, or physical wiring (which CAN IDs are actually populated), ask a human: this repo encodes defaults but hardware can differ.

If any of these sections are unclear or you want more examples (unit-tests, sample PathPlanner .auto snippets, or preferred telemetry keys), tell me which part to expand and I will iterate.
