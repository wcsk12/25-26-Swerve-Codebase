## Purpose
Short, actionable guidance for an AI coding agent working on this WPILib + PathPlanner FRC robot project.

## Big picture (must-know)
- Command-based WPILib robot. Subsystems: `src/main/java/frc/robot/subsystems/`. Commands: `src/main/java/frc/robot/commands/`.
- `RobotContainer.java` wires controllers, default commands, and registers PathPlanner action commands via `NamedCommands.registerCommand(...)`.
- `DriveSubsystem.java` is the swerve implementation: MAX controllers, odometry, field visuals, and PathPlanner AutoBuilder hooks.
- `Constants.java` (especially `DriveConstants`) centralizes CAN IDs, kinematics, and speed limits — change hardware here, not across files.

## Key files to inspect
- `src/main/java/frc/robot/RobotContainer.java` — bindings, default drive `RunCommand`, PathPlanner registrations.
- `src/main/java/frc/robot/subsystems/DriveSubsystem.java` — swerve kinematics/odometry and AutoBuilder integration.
- `src/main/java/frc/robot/Constants.java` — CAN IDs (note: Pigeon2 gyro uses CAN ID 12; DO NOT assign Spark MAX on 12).
- `src/main/deploy/pathplanner/` — path and auto JSONs; PathPlanner .auto files reference named commands from `RobotContainer`.
- `src/main/java/frc/robot/CANChecker.java` — hardware check utilities used from Shuffleboard / controller.

## Project-specific conventions
- Favor small inline commands: `new InstantCommand(...)`, `new RunCommand(..., subsystem)` and chaining `.whileTrue()` / `.toggleOnTrue()`.
- Register path actions in `RobotContainer` so PathPlanner autos can call them (example below).
- Use `SmartDashboard` keys consistently; add telemetry using the same patterns (e.g., `SmartDashboard.putNumber("Pose X (m)", pose.getX())`).

## Examples (copy/paste)
- Register a PathPlanner action in `RobotContainer`:
  NamedCommands.registerCommand("IndexerCMD", new IndexerCMD(miscSubsystem, DriveConstants.indexerMotorSpeed).withTimeout(1));
- Drive default command pattern (deadband + axis mapping):
  m_robotDrive.setDefaultCommand(new RunCommand(() -> m_robotDrive.drive(-MathUtil.applyDeadband(m_driverController.getRawAxis(1), OIConstants.kDriveDeadband), ...), m_robotDrive));

## Build / test / deploy (PowerShell)
- Build: `.\\gradlew.bat build` — compiles and creates the deployable jar.
- Tests: `.\\gradlew.bat test` (JUnit 5 configured).
- Deploy to RoboRIO: `.\\gradlew.bat deploy` (override team with `-Pteam=####`). Deploy block copies `src/main/deploy` to `/home/lvuser/deploy`.

## Hardware/config editing notes
- Update `Constants.DriveConstants` for CAN IDs, physical dims, and max speeds; many classes read values at construction.
- PathPlanner robot dimensions are in `src/main/deploy/pathplanner` + `pathConfig` — update both consistently.
- Use `CANChecker.runChecks()` (Shuffleboard/controller) for hardware probes — prefer that over ad-hoc CAN scans.

## Integration & deps
- External libs managed via `vendordeps/` JSONs (PathPlanner, Phoenix, REV). WPILib + GradleRIO configured in `build.gradle`.
- NetworkTables / Shuffleboard used for telemetry; Limelight appears as table `limelight`.

## Do / Don't (short)
- DO update `Constants` for hardware changes and register small PathPlanner actions in `RobotContainer`.
- DO use `CANChecker` and Shuffleboard widgets for hardware probes.
- DON'T modify `Main.java` or add static initialization; WPILib entry is `RobotBase.startRobot(Robot::new)`.

If you want this expanded into a longer developer guide (examples of PathPlanner .auto snippets, recommended telemetry keys, or unit-test examples), tell me which area to expand.
