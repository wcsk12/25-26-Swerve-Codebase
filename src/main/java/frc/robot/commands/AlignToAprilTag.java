package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Simple P-controller command that uses Limelight v2 fiducial outputs to drive the robot
 * to a desired distance and heading relative to the detected AprilTag.
 *
 * Notes:
 * - This is intentionally simple (P-only) to be easy to reason about and safe for testing.
 * - Tune the gains here for your robot; stop the command if no tag is visible.
 */
public class AlignToAprilTag extends Command {
  private final DriveSubsystem m_drive;
  private final String m_limelightName;
  private final double m_targetDistanceMeters;

  // Simple P gains (tweak for robot)
  private final double kPX = 0.8; // forward/back
  private final double kPY = 1.0; // strafe
  private final double kPAngle = 1.5; // rotation

  // Tolerances
  private final double kPosTol = 0.05; // meters
  private final double kAngleTol = Math.toRadians(3.0); // radians

  public AlignToAprilTag(DriveSubsystem drive, double targetDistanceMeters) {
    m_drive = drive;
    m_limelightName = "limelight"; // change if your camera table uses a different name
    m_targetDistanceMeters = targetDistanceMeters;
  addRequirements(m_drive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults(m_limelightName);
    if (results == null || results.targets_Fiducials == null || results.targets_Fiducials.length == 0) {
      // No tag visible: stop
      m_drive.drive(0, 0, 0, false);
      return;
    }

    // Use the first fiducial detected
    var fid = results.targets_Fiducials[0];
    Pose2d targetInRobot = fid.getTargetPose_RobotSpace2D();

    // targetInRobot translation: x = forward (meters), y = left (meters)
    double forwardError = targetInRobot.getX() - m_targetDistanceMeters;
    double strafeError = targetInRobot.getY();
    double angleError = targetInRobot.getRotation().getRadians();

    // P-controllers -> produce speeds in range roughly [-1,1]
    double xCmd = MathUtil.clamp(kPX * forwardError, -1.0, 1.0);
    double yCmd = MathUtil.clamp(kPY * strafeError, -1.0, 1.0);
    double rotCmd = MathUtil.clamp(kPAngle * angleError, -1.0, 1.0);

    // Drive in robot-relative coordinates
    m_drive.drive(xCmd, -yCmd, -rotCmd, false);
  }

  @Override
  public boolean isFinished() {
    LimelightHelpers.PoseEstimate pe = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_limelightName);
    // When no pose (or no tag) -> don't finish; require tag to determine convergence
    if (pe == null || pe.tagCount <= 0) {
      return false;
    }
    // We can use the raw fiducials distance/pose to decide
    if (pe.rawFiducials == null || pe.rawFiducials.length == 0) {
      return false;
    }
    double forward = pe.pose.getTranslation().getX();
    double lateral = pe.pose.getTranslation().getY();
    double ang = pe.pose.getRotation().getRadians();
    return (Math.abs(forward - m_targetDistanceMeters) < kPosTol
        && Math.abs(lateral) < kPosTol
        && Math.abs(ang) < kAngleTol);
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, false);
  }
}
