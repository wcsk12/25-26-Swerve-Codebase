<!--
Purpose: concise, repo-specific guidance for AI coding agents.
Keep this short, concrete, and anchored to files/patterns that exist in this codebase.
-->
## Copilot instructions for 25-26-Swerve-Codebase

This is a Java 17 WPILib command-based FRC robot project centered on a REV MAXSwerve drive with PathPlanner autos, Limelight-assisted alignment, and Shuffleboard diagnostics.

- Read these first:
  - `src/main/java/frc/robot/Robot.java` owns mode transitions and reuses the `DriveSubsystem` instance from `RobotContainer`.
  - `src/main/java/frc/robot/RobotContainer.java` is the real wiring hub: subsystem construction, default drive command, controller bindings, `NamedCommands`, auto chooser, CAN-check widgets, and dashboard init.
  - `src/main/java/frc/robot/subsystems/DriveSubsystem.java` contains odometry, gyro usage, field-relative drive, PathPlanner `AutoBuilder.configure(...)`, dashboard telemetry, and the slow-mode speed divisor.
  - `src/main/java/frc/robot/Constants.java` and `src/main/java/frc/robot/Configs.java` hold CAN IDs, module offsets, motor speeds, and Spark MAX closed-loop configuration.

- Architecture and data flow:
  - Treat subsystems as single hardware owners. `RobotContainer` constructs them once; `Robot` explicitly calls `m_robotContainer.getDriveSubsystem()` instead of creating another drive instance.
  - Teleop drive flow is split: the default `RunCommand` in `RobotContainer` drives continuously from joystick axes, while `AutoAlignCommand` updates the static `AutoAlignCommand.rCmd` rotation term or directly drives in full-align mode.
  - `DriveSubsystem` wraps four `MAXSwerveModule` objects and a CTRE `Pigeon2` (`CAN ID 12`). Module state order matters; check `drive(...)` before changing kinematics or state mapping.
  - PathPlanner is configured inside `DriveSubsystem`, but event hooks come from `NamedCommands.registerCommand(...)` in `RobotContainer`. Renaming one of those commands breaks matching `.auto` files under `src/main/deploy/pathplanner/autos/`.

- Project-specific patterns and gotchas:
  - Controller bindings use `CommandXboxController`, with raw `Joystick` fallbacks also present in `RobotContainer`; follow the existing `whileTrue(...)`, `toggleOnTrue(...)`, and `withTimeout(...)` style.
  - The `Shoot!` auto action is not a long-running command class; `RobotContainer.getShootSequence()` uses `Commands.sequence(...)` with `InstantCommand` + `WaitCommand` so PathPlanner sequences can continue.
  - `AutoAlignCommand` uses Limelight fiducials first, then falls back to NetworkTables `tv/tx/ta`, and publishes `SmartDashboard` status keys used for debugging.
  - `CANChecker` is meant to run only while disabled; `RobotContainer` already guards Shuffleboard-triggered checks with `DriverStation.isDisabled()`.
  - There are two robot-config loaders: `Configs.fromGUISettings()` and `pathConfig.fromGUISettings()`. `DriveSubsystem` currently uses `pathConfig`, so verify which loader is active before changing PathPlanner config code.

- External integrations:
  - REV Spark MAX + encoders are configured in `MAXSwerveModule` using shared `Configs.MAXSwerveModule` settings.
  - CTRE Phoenix provides the `Pigeon2`; vendor JSONs live in `vendordeps/`.
  - PathPlanner deploy assets live in `src/main/deploy/pathplanner/` (`autos/`, `paths/`, `settings.json`, `navgrid.json`) and are copied to the roboRIO by GradleRIO.
  - Limelight access is centralized through `LimelightHelpers.java`; `DriveSubsystem.periodic()` also reads raw values from the `limelight` NetworkTable for dashboard telemetry.

- Common edit locations:
  - CAN IDs, module angular offsets, and mechanism speeds: `Constants.java`.
  - Spark MAX PID/current limits and encoder conversion factors: `Configs.java`.
  - Default drive behavior, auto chooser filtering, dashboard widgets, and `NamedCommands`: `RobotContainer.java`.
  - Swerve control math, odometry resets, and auto path following: `DriveSubsystem.java`.

- Build and safety workflow:
  - Use `./gradlew.bat build` for the main compile/test/package check on Windows PowerShell.
  - Use `./gradlew.bat deploy` to deploy code plus `src/main/deploy/` assets to the roboRIO.
  - The project uses `edu.wpi.first.GradleRIO` `2026.1.1`, Java 17, and JUnit 5; desktop support is disabled in `build.gradle` (`includeDesktopSupport = false`).
  - Do not add a second scheduler loop; `Robot.robotPeriodic()` already runs `CommandScheduler.getInstance().run()`.
  - If you rename `NamedCommands` like `Align`, `Intake!`, or `Shoot!`, update the matching PathPlanner autos in the same change.

If any section feels unclear or incomplete, tell me which part to tune next—architecture, workflows, or subsystem conventions—and I’ll iterate on the file.
