package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;

/** Command that uses Limelight fiducials to align robot to a tag. */
public class AutoAlignCommand extends Command {
  public static double rCmd = 0.0;
  private final DriveSubsystem m_drive;
  private final String m_llName;
  private final double m_targetDist;

  private final double kPX = 0.8;
  private final double kPY = 1.0;
  private final double kPA = 1.5;

  private final double posTol = 0.05;
  private final double angTol = Math.toRadians(3.0);

  public AutoAlignCommand(DriveSubsystem drive, double targetDistanceMeters) {
    m_drive = drive;
    m_llName = "limelight";
    m_targetDist = targetDistanceMeters;
    addRequirements(m_drive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    var results = LimelightHelpers.getLatestResults(m_llName);
    System.out.println("[AutoAlign] execute() called");
    if (results == null) {
      System.out.println("[AutoAlign] no Limelight results (null)");
      //m_drive.drive(0, 0, 0, false, 0.02);
      rCmd = 0.0;
      SmartDashboard.putString("AutoAlign/status", "no_results");
      return;
    }

  int tagCount = results.targets_Fiducials == null ? 0 : results.targets_Fiducials.length;
    SmartDashboard.putNumber("AutoAlign/tagCount", tagCount);

    double forwardError = 0.0;
    double lateralError = 0.0;
    double angError = 0.0;

    if (tagCount == 0) {
      // No fiducials visible — try simple NetworkTables TV/TX/TA fallback
      boolean tv = LimelightHelpers.getTV(m_llName);
      SmartDashboard.putBoolean("AutoAlign/nt_tv", tv);
      if (!tv) {
        SmartDashboard.putString("AutoAlign/status", "no_tag_nt");
        //m_drive.drive(0, 0, 0, false, 0.02);
        rCmd = 0.0;
        return;
      }

      // Use tx for angle (degrees) and ta (area) as a crude distance proxy
      double tx = LimelightHelpers.getTX(m_llName);
      double ta = LimelightHelpers.getTA(m_llName);
      System.out.println("[AutoAlign] fallback tv=true tx=" + tx + " ta=" + ta);
      angError = Math.toRadians(tx);
      // Map area to forward error: larger area -> closer; simple proportional target area
      double targetArea = 5.0; // % of image; tune as needed
      forwardError = (targetArea - ta) * 0.01; // convert percent to meters-scale proxy
      lateralError = 0.0;
    } else {
      var fid = results.targets_Fiducials[0];
      Pose2d rel = fid.getTargetPose_RobotSpace2D();

      forwardError = rel.getX() - m_targetDist;
      lateralError = rel.getY();
      angError = rel.getRotation().getRadians();
    }

  SmartDashboard.putNumber("AutoAlign/forwardErr", forwardError);
  SmartDashboard.putNumber("AutoAlign/lateralErr", lateralError);
  SmartDashboard.putNumber("AutoAlign/angleErr", angError);

    double xCmd = MathUtil.clamp(kPX * forwardError, -1.0, 1.0);
    double yCmd = MathUtil.clamp(kPY * lateralError, -1.0, 1.0);
    rCmd = MathUtil.clamp(kPA * angError, -1.0, 1.0);
    
    System.out.println("Command rCmd: " + rCmd);
    // drive robot-relative
    //m_drive.drive(xCmd, -yCmd, rCmd, false, 0.02);


    SmartDashboard.putString("AutoAlign/status", "running");
  }

  @Override
  public boolean isFinished() {
    var pe = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_llName);
    if (pe == null || pe.tagCount <= 0) {
      return false;
    }
    double fwd = pe.pose.getTranslation().getX();
    double lat = pe.pose.getTranslation().getY();
    double ang = pe.pose.getRotation().getRadians();
    return Math.abs(fwd - m_targetDist) < posTol && Math.abs(lat) < posTol && Math.abs(ang) < angTol;
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, false, 0.02);
    SmartDashboard.putString("AutoAlign/status", interrupted ? "interrupted" : "ended");
  }
}
