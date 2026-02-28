<!--
Purpose: provide concise, actionable guidance for AI coding agents working on this repo.
Keep this short (20–50 lines). Reference concrete files and patterns the agent will see.
-->
# Copilot instructions for 25-26-Swerve-Codebase

This is a Java WPILib (command-based) robot project for an FRC swerve-drive robot. Focus on these facts and patterns when making changes.

- Big picture
  - Entry points: `src/main/java/frc/robot/Robot.java` (TimedRobot) and `RobotContainer.java` (wires subsystems, default commands, and bindings).
  - Drive: `DriveSubsystem.java` uses `MAXSwerveModule` (SparkMax + encoders) and a CTRE `Pigeon2` gyro. PathPlanner is integrated via `AutoBuilder.configure` in `DriveSubsystem`.
  - Hardware/config: CAN IDs and module offsets live in `Constants.java` (see `DriveConstants`). SparkMax PID/encoder settings live in `Configs.java` (`Configs.MAXSwerveModule`).
  - Commands: see `src/main/java/frc/robot/commands/` for `AutoAlignCommand`, `IntakeCMD`, `ShooterCMD`, etc. `RobotContainer` registers PathPlanner `NamedCommands` and sets a default `RunCommand` for driving.

- Developer workflows (how to build, deploy, debug)
  - Build locally (Windows PowerShell): `.
    \gradlew.bat build` (or run from VSCode WPILib tasks). Use `build` to catch compile problems.
  - Deploy to roboRIO: `.
    \gradlew.bat deploy` (WPILib gradle tasks are configured; the project deploys jar and static PathPlanner files). The repo's gradle logs indicate JRE 17 is required on the target.
  - Logs: the project prints important runtime messages to stdout (e.g., CAN checker, controller bindings). `CommandScheduler.getInstance().run()` is already called in `Robot.robotPeriodic()`.

- Project-specific conventions and gotchas
  - Single hardware instances: subsystems are singletons created in `RobotContainer`. Avoid creating hardware objects (SparkMax/Pigeon) outside subsystems—use `RobotContainer.getDriveSubsystem()` to reuse the instance.
  - PathPlanner settings: deployment folder `src/main/deploy/pathplanner/` contains settings, paths, and autos; PathPlanner integration uses `Configs.fromGUISettings()` — currently this method contains a hard-coded absolute path to a developer machine. Prefer `Filesystem.getDeployDirectory()` + `pathplanner/settings.json` instead of absolute paths.
  - CANChecker: a Shuffleboard toggle and controller Start button can run `CANChecker.runChecks()`. There is also a `RUN_CAN_CHECKER` flag in `RobotContainer` for optional startup checks.
  - Controller bindings: code uses `CommandXboxController` with a joystick fallback. Example binding style: `m_driverController.a().whileTrue(new AutoAlignCommand(m_robotDrive, 0.6));`.
  - PathPlanner commands: `NamedCommands.registerCommand("Align", new AutoAlignCommand(...))` — updating names affects autos in `src/main/deploy/pathplanner/autos/`.

- Where to change common things
  - Change CAN IDs or module offsets: `Constants.java` → `DriveConstants`.
  - Adjust SparkMax PID or encoder factors: `Configs.java` → `MAXSwerveModule` static configs.
  - Add/register PathPlanner commands: `RobotContainer.configureBindings()` (use `NamedCommands.registerCommand(...)`).
  - Default drive behavior: `RobotContainer` sets a `RunCommand` as the default for `DriveSubsystem`.

- Safety and scheduler
  - `Robot.robotPeriodic()` calls `CommandScheduler.getInstance().run()` — do not duplicate scheduling calls. When creating new commands prefer Command-based patterns (commands, sequences, parallel, onTrue/whileTrue triggers).
  - Use `DriverStation.isDisabled()` checks around hardware-probing utilities (the repo already does this for the CAN checker).

- Quick examples for the agent
  - Fix PathPlanner config loader: replace the hard-coded path in `Configs.fromGUISettings()` with `new File(Filesystem.getDeployDirectory(), "pathplanner/settings.json")`.
  - To add a new autonomous routine: implement a Command/SequentialCommandGroup in `commands/`, then register with `NamedCommands.registerCommand("MyAuto", myCommand)` and confirm `src/main/deploy/pathplanner/autos/` contains a matching auto (.auto file).

If anything in this summary is unclear or missing details you need (e.g., hardware mapping not in Constants, or how the team uses Shuffleboard), tell me which area to expand and I will iterate.
