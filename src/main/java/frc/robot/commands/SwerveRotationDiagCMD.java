package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Diagnostic command: for a pure rotation request, publish expected module
 * angles vs actual chassis-relative module angles and suggest fixes.
 */
public class SwerveRotationDiagCMD extends Command {
  private final DriveSubsystem drive;
  private final double rot; // rad/s (relative magnitude only)
  private final double durationSeconds;
  private long endTimeMs;

  public SwerveRotationDiagCMD(DriveSubsystem drive, double rot, double durationSeconds) {
    this.drive = drive;
    this.rot = rot;
    this.durationSeconds = durationSeconds;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    endTimeMs = System.currentTimeMillis() + (long)(durationSeconds * 1000.0);
    SmartDashboard.putString("RotDiag/Status", "Started");
  }

  @Override
  public void execute() {
    // Compute expected module states for pure rotation (robot-relative)
    ChassisSpeeds cs = new ChassisSpeeds(0.0, 0.0, rot);
    SwerveModuleState[] expected = DriveConstants.kDriveKinematics.toSwerveModuleStates(cs);

    double expFL = expected[0].angle.getRadians();
    double expFR = expected[1].angle.getRadians();
    double expBL = expected[2].angle.getRadians();
    double expBR = expected[3].angle.getRadians();

    double actFL = drive.getFrontLeftChassisAngle();
    double actFR = drive.getFrontRightChassisAngle();
    double actBL = drive.getRearLeftChassisAngle();
    double actBR = drive.getRearRightChassisAngle();

    SmartDashboard.putNumber("RotDiag/ExpFL_deg", Math.toDegrees(expFL));
    SmartDashboard.putNumber("RotDiag/ExpFR_deg", Math.toDegrees(expFR));
    SmartDashboard.putNumber("RotDiag/ExpBL_deg", Math.toDegrees(expBL));
    SmartDashboard.putNumber("RotDiag/ExpBR_deg", Math.toDegrees(expBR));

    SmartDashboard.putNumber("RotDiag/ActFL_deg", Math.toDegrees(actFL));
    SmartDashboard.putNumber("RotDiag/ActFR_deg", Math.toDegrees(actFR));
    SmartDashboard.putNumber("RotDiag/ActBL_deg", Math.toDegrees(actBL));
    SmartDashboard.putNumber("RotDiag/ActBR_deg", Math.toDegrees(actBR));

    // Differences and quick suggestions (normalize to [-PI, PI])
    suggestFix("FL", expFL, actFL);
    suggestFix("FR", expFR, actFR);
    suggestFix("BL", expBL, actBL);
    suggestFix("BR", expBR, actBR);

    // Compute whether each expected wheel is pointing generally outward or inward
    // relative to the module's radial vector from the robot center. This helps
    // determine "outward/inward" behavior during rotation.
    double halfWB = DriveConstants.kWheelBase / 2.0;
    double halfTW = DriveConstants.kTrackWidth / 2.0;

    double radFL = Math.atan2(halfTW, halfWB);
    double radFR = Math.atan2(-halfTW, halfWB);
    double radBL = Math.atan2(halfTW, -halfWB);
    double radBR = Math.atan2(-halfTW, -halfWB);

    boolean expOutFL = isOutward(expFL, radFL);
    boolean expOutFR = isOutward(expFR, radFR);
    boolean expOutBL = isOutward(expBL, radBL);
    boolean expOutBR = isOutward(expBR, radBR);

    boolean actOutFL = isOutward(actFL, radFL);
    boolean actOutFR = isOutward(actFR, radFR);
    boolean actOutBL = isOutward(actBL, radBL);
    boolean actOutBR = isOutward(actBR, radBR);

    SmartDashboard.putBoolean("RotDiag/ExpOut_FL", expOutFL);
    SmartDashboard.putBoolean("RotDiag/ExpOut_FR", expOutFR);
    SmartDashboard.putBoolean("RotDiag/ExpOut_BL", expOutBL);
    SmartDashboard.putBoolean("RotDiag/ExpOut_BR", expOutBR);

    SmartDashboard.putBoolean("RotDiag/ActOut_FL", actOutFL);
    SmartDashboard.putBoolean("RotDiag/ActOut_FR", actOutFR);
    SmartDashboard.putBoolean("RotDiag/ActOut_BL", actOutBL);
    SmartDashboard.putBoolean("RotDiag/ActOut_BR", actOutBR);

    // User-specified desired pattern when rotating RIGHT (clockwise):
    // FR outward, BR inward, FL inward, BL outward
    // Note: In WPILib positive rotation (omega) is CCW. A right turn (CW)
    // corresponds to a negative omega. We'll only check the pattern when
    // the requested rotation indicates a right/CW rotation (rot < 0).
    if (rot < 0.0) {
      boolean want_FR = true;
      boolean want_BR = false;
      boolean want_FL = false;
      boolean want_BL = true;

      boolean match = (actOutFR == want_FR) && (actOutBR == want_BR) && (actOutFL == want_FL)
          && (actOutBL == want_BL);
      SmartDashboard.putBoolean("RotDiag/PatternMatch_RightCW", match);

      if (!match) {
        SmartDashboard.putString("RotDiag/PatternSuggestion",
            "Pattern mismatch for RIGHT rotation: check offsets (flip PI?) or wiring");
      } else {
        SmartDashboard.putString("RotDiag/PatternSuggestion", "Pattern OK for RIGHT rotation");
      }
    }

    // Also print a compact summary to stdout so deploy logs / console capture
    // can show the diagnostic when running on the roboRIO.
    System.out.println(String.format(
        "RotDiag Summary: ExpDeg=[%.1f,%.1f,%.1f,%.1f] ActDeg=[%.1f,%.1f,%.1f,%.1f] OutAct=[%b,%b,%b,%b] PatternMatch=%b",
        Math.toDegrees(expFL), Math.toDegrees(expFR), Math.toDegrees(expBL), Math.toDegrees(expBR),
        Math.toDegrees(actFL), Math.toDegrees(actFR), Math.toDegrees(actBL), Math.toDegrees(actBR),
        actOutFL, actOutFR, actOutBL, actOutBR,
        (rot < 0.0) ? SmartDashboard.getBoolean("RotDiag/PatternMatch_RightCW", false) : false));
  }

  private void suggestFix(String name, double expected, double actual) {
    double diff = wrapRadians(actual - expected);
    SmartDashboard.putNumber("RotDiag/Diff_" + name + "_deg", Math.toDegrees(diff));

    String suggestion = "OK";
    // If off by around PI (180 deg), suggest flipping offset by PI
    if (Math.abs(Math.abs(diff) - Math.PI) < Math.toRadians(20)) {
      suggestion = "Suggest: add/sub PI to offset";
    } else if (Math.abs(diff) > Math.toRadians(90)) {
      suggestion = "Suggest: re-check module wiring/offset";
    }
    SmartDashboard.putString("RotDiag/Suggest_" + name, suggestion);
  }

  private double wrapRadians(double r) {
    while (r > Math.PI) r -= 2*Math.PI;
    while (r < -Math.PI) r += 2*Math.PI;
    return r;
  }

  /**
   * Return true if the wheel's angle is generally pointing away from the
   * robot center (outward) compared to the module's radial vector.
   */
  private boolean isOutward(double wheelAngle, double radialAngle) {
    double d = wrapRadians(wheelAngle - radialAngle);
    return Math.abs(d) < (Math.PI / 2.0);
  }

  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() > endTimeMs;
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("RotDiag/Status", interrupted ? "Interrupted" : "Done");
  }
}
