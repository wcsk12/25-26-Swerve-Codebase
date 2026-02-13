package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.geometry.Pose2d;

/** Controls a REV Blinkin connected to a RoboRIO PWM port. */
public class LedSubsystem extends SubsystemBase {
  // PWM port on the RoboRIO where the Blinkin is connected
  private static final int BLINKIN_PWM_PORT = 1;

  // These raw values are conservative defaults that map to useful solid-color patterns.
  // They may need tuning for your hardware; keep them configurable.
  // Pattern values are expressed as servo angles (0..180). These are placeholders
  // that map to different Blinkin patterns. You'll likely want to tune these for the
  // specific firmware on your Blinkin.
  private static final double PATTERN_BLUE = 90.0;
  private static final double PATTERN_YELLOW = 120.0;
  private static final double PATTERN_GREEN = 60.0;

  // Thresholds for considering the robot "lined up" with a fiducial
  private static final double LATERAL_TOL_METERS = 0.05; // 5 cm
  private static final double ANG_TOL_RAD = Math.toRadians(3.0); // 3 degrees

  private final Servo m_servo;
  private int m_lastPattern = -1;

  private final String m_limelightName = "limelight";
  private final double m_targetDistanceMeters;

  public LedSubsystem(double targetDistanceMeters) {
    m_targetDistanceMeters = targetDistanceMeters;
  m_servo = new Servo(BLINKIN_PWM_PORT);
  // Set a safe default
  setBlue();
  }

  @Override
  public void periodic() {
    // Check limelight for fiducials and update LED pattern appropriately
    var results = LimelightHelpers.getLatestResults(m_limelightName);
    if (results == null) {
      setBlue();
      return;
    }

    int tagCount = results.targets_Fiducials == null ? 0 : results.targets_Fiducials.length;
    if (tagCount == 0) {
      // No fiducials visible
      setBlue();
      return;
    }

    var fid = results.targets_Fiducials[0];
  Pose2d rel = fid.getTargetPose_RobotSpace2D();
  double forwardError = rel.getX() - m_targetDistanceMeters;
  double lateralError = rel.getY();
  double angError = rel.getRotation().getRadians();

  boolean linedUp = Math.abs(forwardError) < LATERAL_TOL_METERS && Math.abs(lateralError) < LATERAL_TOL_METERS && Math.abs(angError) < ANG_TOL_RAD;
    if (linedUp) {
      setGreen();
    } else {
      setYellow();
    }
  }

  private void setPattern(double angle) {
    int a = (int) Math.round(angle);
    if (a == m_lastPattern) {
      return;
    }
    try {
      m_servo.setAngle(angle);
      m_lastPattern = a;
    } catch (Exception e) {
      System.out.println("[LedSubsystem] Failed to set Blinkin angle: " + e);
    }
  }

  public void setBlue() {
    setPattern(PATTERN_BLUE);
  }

  public void setYellow() {
    setPattern(PATTERN_YELLOW);
  }

  public void setGreen() {
    setPattern(PATTERN_GREEN);
  }

  public void setOff() {
    setPattern(0.0);
  }
}
