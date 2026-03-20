package frc.robot.commands;

import java.io.File;
import java.io.FileInputStream;
import java.util.Properties;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Diagnostic command: on initialize, load persisted shooter gains from /home/lvuser,
 * apply them immediately, then run closed-loop to the same soft target used by the tuner.
 * This lets us compare B directly to L3 while guaranteeing gain application happens right before spin-up.
 */
public class ShooterSpinupWithPersistedGainsCommand extends Command {
  private final ShooterSubsystem shooter;
  private final double targetRpm;
  private double startTime;
  private double lastLogTime;

  public ShooterSpinupWithPersistedGainsCommand(ShooterSubsystem shooter) {
    this(shooter, DriveConstants.softShooterTargetRPM);
  }

  public ShooterSpinupWithPersistedGainsCommand(ShooterSubsystem shooter, double targetRpm) {
    this.shooter = shooter;
    this.targetRpm = targetRpm;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    lastLogTime = 0.0;
    try {
      File f = new File("/home/lvuser/shooter_gains.properties");
      if (f.exists()) {
        Properties p = new Properties();
        try (FileInputStream fis = new FileInputStream(f)) {
          p.load(fis);
        }
        double pGain = Double.parseDouble(p.getProperty("p", "0"));
        double iGain = Double.parseDouble(p.getProperty("i", "0"));
        double dGain = Double.parseDouble(p.getProperty("d", "0"));
        double ff = Double.parseDouble(p.getProperty("ff", "0"));
        System.out.println(String.format("[ShooterSpinupWithPersistedGains] Applying persisted gains p=%.8f i=%.8f d=%.8f ff=%.8f", pGain, iGain, dGain, ff));
        shooter.applyClosedLoopGains(pGain, iGain, dGain, ff);
      } else {
        System.out.println("[ShooterSpinupWithPersistedGains] No persisted gains file found; using current in-memory gains.");
      }
    } catch (Exception e) {
      System.out.println("[ShooterSpinupWithPersistedGains] Failed to reload persisted gains: " + e);
    }

    shooter.setClosedLoopTargetRPM(targetRpm);
    System.out.println(String.format("[ShooterSpinupWithPersistedGains] targetRPM=%.1f", targetRpm));
  }

  @Override
  public void execute() {
    double now = Timer.getFPGATimestamp();
    double elapsed = now - startTime;
    double rpm = shooter.getMeasuredRPM();

    SmartDashboard.putNumber("Shooter/B Test Elapsed (s)", elapsed);
    SmartDashboard.putNumber("Shooter/B Test RPM", rpm);
    SmartDashboard.putNumber("Shooter/B Test Target RPM", targetRpm);

    if (elapsed - lastLogTime >= 0.25) {
      System.out.println(String.format("[ShooterSpinupWithPersistedGains] t=%.2fs rpm=%.1f target=%.1f", elapsed, rpm, targetRpm));
      lastLogTime = elapsed;
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setClosedLoopTargetRPM(0.0);
    double elapsed = Timer.getFPGATimestamp() - startTime;
    double rpm = shooter.getMeasuredRPM();
    System.out.println(String.format("[ShooterSpinupWithPersistedGains] end interrupted=%s t=%.2fs rpm=%.1f", interrupted, elapsed, rpm));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
