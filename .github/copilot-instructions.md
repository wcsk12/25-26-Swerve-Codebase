## Quick context (what this project is)

- Java WPILib command-based robot project (GradleRIO). Main robot entry: `frc.robot.Robot` (see `src/main/java/frc/robot/Robot.java`).
- Primary subsystems live under `src/main/java/frc/robot/subsystems` (notably `DriveSubsystem` and `MAXSwerveModule`).
- This code uses REV SparkMAX motor controllers, a Pigeon2 gyro (CTRE), and PathPlanner for autonomous paths.

## Big-picture architecture

- Robot lifecycle: `Main` -> `Robot` -> `RobotContainer`. `RobotContainer` wires subsystems, default commands and the SmartDashboard auto chooser (`RobotContainer.getAutonomousCommand()`).
- Drive stack:
  - `DriveSubsystem` is the high-level swerve subsystem (odometry, PathPlanner AutoBuilder configuration, `drive()` and `setModuleStates()`).
  - `MAXSwerveModule` is a per-wheel wrapper that encapsulates SparkMAX configs, encoders, and closed-loop controllers (see `src/main/java/frc/robot/subsystems/MAXSwerveModule.java`).
  - Kinematics constants are in `Constants.java` (`DriveConstants`, `ModuleConstants`, etc.). CAN IDs and chassis offsets are defined here and must match the robot wiring.

## Important implementation patterns

- Command-based default command pattern: `RobotContainer` sets a RunCommand on `DriveSubsystem` using `CommandXboxController` axis values. Look at the deadband usage in `RobotContainer`.
- PathPlanner integration: `DriveSubsystem` calls `AutoBuilder.configure(...)` and uses `pathConfig.fromGUISettings()` / `Configs.fromGUISettings()` to read PathPlanner robot settings. Note: there is a hard-coded /deploy path in `Configs.fromGUISettings()` that references a user-specific path — treat that as a known fragile spot when debugging path loading.
- SparkMAX configuration: `Configs.MAXSwerveModule` builds `SparkMaxConfig` objects and these are applied/persisted in `MAXSwerveModule` constructor. Search `Configs.MAXSwerveModule` for PID and feed-forward values.

## Developer workflows and commands

- Common Gradle tasks (Windows PowerShell examples):
  - Build: 
    ```powershell
    .\gradlew build
    ```
  - Run unit tests: 
    ```powershell
    .\gradlew test
    ```
  - Deploy to RoboRIO (requires network/robot reachable and team number):
    ```powershell
    .\gradlew deploy -PteamNumber=8116
    ```
    If the deploy fails with "Missing Target" or discovery timeouts, verify the RoboRIO is on the network and accessible. The build file uses GradleRIO; the deploy task will fail fast if the RoboRIO can't be found.

## Project-specific gotchas and conventions

- Hard-coded PathPlanner settings path: `Configs.fromGUISettings()` reads a settings JSON via a full absolute path in `Filesystem.getDeployDirectory()` usage — this will break when run from other machines. Look in `pathConfig` for a safer fallback (`pathConfig` contains sensible defaults and comments).
- CAN ID mapping and chassis offsets live in `Constants.DriveConstants`. Any hardware changes must update those constants first.
- The project uses WPILib simulation flags in `build.gradle` (see `wpi.sim.addGui()` and `wpi.sim.addDriverstation()`). Simulation is enabled by configuration, but desktop support is disabled by default (`includeDesktopSupport = false`).

## Where to look for changes / examples

- Default drive wiring & controls: `RobotContainer.java` (controller bindings, default RunCommand).
- Swerve module implementation and how closed-loop control is applied: `src/main/java/frc/robot/subsystems/MAXSwerveModule.java` and `src/main/java/frc/robot/Configs.java`.
- PathPlanner/autonomous setup and AutoBuilder: `src/main/java/frc/robot/subsystems/DriveSubsystem.java` and `src/main/java/frc/robot/pathConfig.java`.
- Constants that affect physics/odometry and CAN IDs: `src/main/java/frc/robot/Constants.java`.

## Guidance for an AI agent working on this repo

- When changing CAN IDs, update `Constants.DriveConstants` and run a quick search for usages. Tests won't catch mismatch with physical wiring.
- When editing PathPlanner loading, prefer using `Filesystem.getDeployDirectory()` or `Filesystem.getSourceDirectory()` patterns; treat the existing absolute path as a red flag and add defensive fallback behavior.
- Tune SparkMAX configurations in `Configs.MAXSwerveModule` rather than scattering PID constants elsewhere—this file centralizes Spark-specific tuning.
- Prefer adding new commands under `src/main/java/frc/robot/commands/*`. When introducing new subsystem state, add accessors in the subsystem and update `RobotContainer` wiring.

## Quick references

- Main entry: `src/main/java/frc/robot/Main.java` and `Robot.java`
- Subsystems: `src/main/java/frc/robot/subsystems/*`
- Commands: `src/main/java/frc/robot/commands/*`
- Constants/Configs: `src/main/java/frc/robot/Constants.java`, `Configs.java`, `pathConfig.java`
- Build: `build.gradle` (GradleRIO plugin configured for WPILib 2026)

If any section is unclear or you want this to enforce stricter rules (linting, tests, CI tasks), tell me which areas to expand and I will iterate.
