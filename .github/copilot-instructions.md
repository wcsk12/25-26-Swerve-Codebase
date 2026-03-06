<!--
Purpose: provide concise, actionable guidance for AI coding agents working on this repo.
Keep this short (20–50 lines). Reference concrete files and patterns the agent will see.
-->
## Copilot instructions for 25-26-Swerve-Codebase

This repository is a Java WPILib (command-based) robot project for an FRC swerve-drive robot. Keep guidance short and concrete — below are the most useful, repo-specific facts an AI agent needs to be productive.

- Big picture (what to read first)
  - Entry points: `src/main/java/frc/robot/Robot.java` (TimedRobot) and `src/main/java/frc/robot/RobotContainer.java` (wires subsystems, default commands, and controller bindings).
  - Drive: `src/main/java/frc/robot/subsystems/DriveSubsystem.java` builds swerve modules from `MAXSwerveModule` (REV SparkMax + encoders) and reads a CTRE `Pigeon2` gyro. PathPlanner integration is performed during auto configuration (look for `AutoBuilder.configure` usages).
  - Config and constants: `src/main/java/frc/robot/Constants.java` (DriveConstants nested class) holds CAN IDs and module offsets. `src/main/java/frc/robot/Configs.java` contains static hardware config objects (e.g., `Configs.MAXSwerveModule`).
  - Commands: inspect `src/main/java/frc/robot/commands/` for concrete commands like `AutoAlignCommand`, `IntakeCMD`, `ShooterCMD`. `RobotContainer` registers PathPlanner `NamedCommands` used by autos.

- Developer workflows (concrete commands)
  - Build (Windows PowerShell):
    - Build: `.
      \gradlew.bat build`
    - Deploy to roboRIO (uploads jar and deploy folder files): `.
      \gradlew.bat deploy`
    - Note: the roboRIO/target requires Java 17 (project Gradle/JRE assumptions). Use VSCode WPILib tasks if preferred.
  - Runtime checks: `CANChecker.runChecks()` can be triggered from Shuffleboard or via the controller Start button. `RobotContainer` exposes a `RUN_CAN_CHECKER` flag to control startup checks.

- Project-specific conventions & gotchas
  - Single hardware instances: subsystems are singletons owned by `RobotContainer`. Do not construct hardware objects (SparkMax, Pigeon) outside subsystems—use `RobotContainer.getDriveSubsystem()` to reuse the single instance.
  - PathPlanner assets: static autos, paths, and settings live in `src/main/deploy/pathplanner/` (check `autos/` and `paths/`). Changing `NamedCommands` names in `RobotContainer` affects those `.auto` files.
  - Hard-coded path: `Configs.fromGUISettings()` currently contains an absolute developer path. Replace with deploy-aware loading: `new File(Filesystem.getDeployDirectory(), "pathplanner/settings.json")`.
  - Controller style: uses `CommandXboxController` with joystick fallbacks; bindings follow the `m_driverController.a().whileTrue(new AutoAlignCommand(...))` pattern.

- Integration points & external deps
  - Hardware libs: REV SparkMax libraries and CTRE Phoenix are used (check `tools/vendordeps/*.json`).
  - PathPlanner integration: PathPlanner auto runner calls `NamedCommands.registerCommand(...)` from `RobotContainer`. Keep command names stable unless you update deploy `.auto` files.
  - Deploy folder: `src/main/deploy/` is copied to the robot during `deploy` task—changes to paths/autos/settings must be placed here.

- Where to change common things (concrete file targets)
  - CAN IDs/module offsets: `Constants.java` → `DriveConstants`
  - SparkMax PID/encoder factors: `Configs.java` → `MAXSwerveModule` static configs
  - Default drive behavior and PathPlanner registrations: `RobotContainer.configureBindings()` / `RobotContainer` constructor
  - Add a new auto: implement a `SequentialCommandGroup` under `commands/`, then `NamedCommands.registerCommand("MyAuto", myCommand)` and add the corresponding `.auto` in `src/main/deploy/pathplanner/autos/`.

- Safety & scheduler
  - `Robot.robotPeriodic()` already calls `CommandScheduler.getInstance().run()` — do not add a second scheduler loop.
  - Wrap hardware probing utilities with `DriverStation.isDisabled()` if they run at boot.

- Quick actionable examples for edits
  - Fix PathPlanner settings loader: replace hard-coded path in `Configs.fromGUISettings()` with `new File(Filesystem.getDeployDirectory(), "pathplanner/settings.json")` and verify `deploy/` contains the settings file.
  - Registering a PathPlanner command example (in `RobotContainer`):
    - `NamedCommands.registerCommand("Align", new AutoAlignCommand(m_robotDrive, AutoAlignCommand.Mode.FULL_ALIGN));`

If anything is unclear or you want me to include more specific examples (e.g., exact lines to edit in `Configs.fromGUISettings()` or a small test auto), tell me which area to expand and I'll iterate.
