# Operator Cheat Sheet — Swerve Robot

Quick reference for drivers & operators. Current mappings are based on `src/main/java/frc/robot/RobotContainer.java`.

## Driver controls (Xbox controller)

- **Left stick Y (Axis 1)**: Forward / backward drive
- **Left stick X (Axis 0)**: Strafe left / right
- **Right stick X (Axis 4)**: Rotate robot
- **Drive mode**: Field-relative by default
- **Deadband**: `OIConstants.kDriveDeadband`

### Driver buttons

- **A (hold)**: Auto-align to AprilTag using `AutoAlignCommand(m_robotDrive, 0.6)`
- **Left bumper (hold)**: 0.5× drive speed
- **Right bumper (hold)**: 0.25× drive speed
- **Back / Select (press)**: Zero gyro heading

## Operator controls (button board / fallback joystick on port 1)

These are the active bindings in the code right now.

- **Button 1 (hold)**: Spin shooter to **soft shot RPM** (`DriveConstants.softShooterTargetRPM`)
- **Button 2 (hold)**: **Shoot when ready**
  - Spins shooter to soft shot RPM
  - Waits until shooter is at speed
  - Runs launcher while held
- **Button 3 (hold)**: Intake game piece
  - Runs intake motor
  - Moves pos-intake to bumper position
- **Button 4 (hold)**: Feed / launch
  - Runs launcher motor
  - Runs pos-intake shake/jiggle
- **Button 5 (hold)**: Move pos-intake to bumper position
- **Button 6 (hold)**: Move pos-intake to zero / home position
- **Button 7 (hold)**: Spin shooter to **1000 RPM**
- **Button 8 (toggle)**: Toggle shooter preset `ShooterSetSpeed.SlowSpeed`

## Shooter tuning / diagnostics

- **Shuffleboard → Tuning tab → "Run Shooter Tuner"**: Runs the shooter PID tuning test
- **Shuffleboard → Tuning tab → "Reload Shooter Gains"**: Reloads persisted gains from `/home/lvuser/shooter_gains.properties`
- **L3 binding**: removed — shooter tuner is now started from Shuffleboard only

## CAN checks / maintenance

- **Shuffleboard → CAN Checks tab → "Run CAN Checker"**: Run CAN probe (robot must be disabled)
- **Shuffleboard → CAN Checks tab → "Run CAN Checker (Manual)"**: Manual polled version of CAN probe (robot must be disabled)

## Autonomous / PathPlanner

- Auto chooser appears on SmartDashboard as **`Auto Chooser`**
- PathPlanner small actions are registered in `RobotContainer` via `NamedCommands`
- Shooter-related named commands use closed-loop RPM control

## Notes

- Shooter is configured as:
  - **`shooterMotor2`** = closed-loop master with encoder
  - **`shooterMotor1`** = inverted follower
- Important CAN note: **Pigeon2 gyro uses CAN ID 12** — do not assign a Spark MAX to ID 12

## Files to check if something changes

- `src/main/java/frc/robot/RobotContainer.java` — button mappings
- `src/main/java/frc/robot/subsystems/ShooterSubsystem.java` — shooter closed-loop/follower config
- `src/main/java/frc/robot/Constants.java` — RPM targets, CAN IDs, speed constants
