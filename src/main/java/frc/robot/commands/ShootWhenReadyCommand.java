package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * While held: spin shooter to softShooterTargetRPM using closed-loop, wait until at-speed,
 * then run the launcher. On release: stop both.
 */
public class ShootWhenReadyCommand extends Command {
  private final ShooterSubsystem shooter;
  private final LauncherSubsystem launcher;
  private final double targetRpm;
  private final double rpmTolerance;

  public ShootWhenReadyCommand(ShooterSubsystem shooter, LauncherSubsystem launcher) {
    this(shooter, launcher, DriveConstants.midShooterTargetRPM, 0.05); // 5% tolerance
  }

  public ShootWhenReadyCommand(ShooterSubsystem shooter, LauncherSubsystem launcher, double targetRpm, double toleranceFraction) {
    this.shooter = shooter;
    this.launcher = launcher;
    this.targetRpm = targetRpm;
    this.rpmTolerance = toleranceFraction;
    addRequirements(shooter, launcher);
  }

  @Override
  public void initialize() {
    // Start closed-loop ramp toward target RPM
    shooter.setClosedLoopTargetRPM(targetRpm);
  }

  @Override
  public void execute() {
    double currentRpm = shooter.getMeasuredRPM();
    double lower = targetRpm * (1.0 - rpmTolerance);
    double upper = targetRpm * (1.0 + rpmTolerance);

    boolean atSpeed = currentRpm >= lower && currentRpm <= upper;

    if (atSpeed) {
      launcher.setLauncherSpeed(DriveConstants.launcherMotorSpeed);
    } else {
      launcher.setLauncherSpeed(0.0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Stop both shooter and launcher on release
    shooter.setClosedLoopTargetRPM(0.0);
    launcher.setLauncherSpeed(0.0);
  }

  @Override
  public boolean isFinished() {
    // This is intended as a whileHeld command; it never finishes on its own.
    return false;
  }
}
