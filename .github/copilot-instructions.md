## Purpose
This is an FRC robot codebase (WPILib + PathPlanner + vendor libs). These notes give an AI coding agent the minimum, actionable repo knowledge to be productive quickly.

## Big picture (quick)
- Command-based robot: subsystems in `src/main/java/frc/robot/subsystems/`, commands in `src/main/java/frc/robot/commands/`.
- `RobotContainer.java` wires subsystems, default commands, button bindings and registers PathPlanner small-action commands (via `NamedCommands`).
- `DriveSubsystem.java` implements the swerve drive, odometry and PathPlanner AutoBuilder hooks; `Constants.java` holds CAN IDs, kinematics and speed limits.
- PathPlanner assets live under `src/main/deploy/pathplanner/` (paths and autos reference NamedCommands by name).

## Key files to read first
- `src/main/java/frc/robot/RobotContainer.java` — wiring, default RunCommands, and NamedCommands registration.
- `src/main/java/frc/robot/subsystems/DriveSubsystem.java` — swerve kinematics, odometry, gyro (Pigeon2 on CAN ID 12) and field visuals.
- `src/main/java/frc/robot/Constants.java` — update hardware IDs and kinematics here (DON'T reuse CAN ID 12 for Spark MAX; it's the pigeon).
- `src/main/deploy/pathplanner/` — JSON path/autos; names must match `NamedCommands.registerCommand("Name", ...)` used in `RobotContainer`.
- `src/main/java/frc/robot/CANChecker.java` and `src/main/java/frc/robot/Dashboard.java` — examples of hardware probes and telemetry patterns.

## Repo-specific conventions (copyable patterns)
- Small actions for PathPlanner: register in `RobotContainer` so autos can reference them. Example: NamedCommands.registerCommand("IndexerCMD", new IndexerCMD(...).withTimeout(1));
- Prefer inline command usage: `new RunCommand(() -> ..., subsystem)` and `InstantCommand` for short operations.
- Telemetry: add SmartDashboard/Shuffleboard keys in `Dashboard.java` style (use consistent key names, e.g., `Pose X (m)`).

## Build / test / deploy (PowerShell)
Use the project wrapper in the repo root. Typical commands (PowerShell):
```
.\gradlew.bat build
.\gradlew.bat test
.\gradlew.bat deploy            # reads WPILib prefs for team number
.\gradlew.bat deploy -Pteam=1234  # override team
```

## Hardware/config edits checklist
1. Update `Constants` (DriveConstants, CAN IDs, kinematics) before touching hardware code.
2. Update `src/main/deploy/pathplanner/pathConfig` and regenerate/adjust PathPlanner assets if robot dims change.
3. Register any new PathPlanner action commands in `RobotContainer` so they can be referenced from .auto files.

## Integration & external deps
- WPILib + GradleRIO (configured in `build.gradle`). Vendor dependencies are in `vendordeps/` (PathPlanner, Phoenix, REV).
- NetworkTables/Shuffleboard used heavily; Limelight entries use table name `limelight`.

## Do / Don't (short)
- DO: update `Constants` for hardware changes; register PathPlanner actions in `RobotContainer`.
- DO: use `CANChecker.runChecks()` and existing Shuffleboard toggles for hardware probing.
- DON'T: change `Main.java` or add global static init — WPILib startup is via `RobotBase.startRobot(Robot::new)`.

## If stuck
- Ask a human for team number, roboRIO IP, or real wiring/CAN topology (these are not reliably discoverable from source alone).

If you'd like, I can expand small sections (unit-test patterns, example NamedCommands usage, or PathPlanner .auto snippets).
