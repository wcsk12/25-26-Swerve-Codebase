package frc.robot.subsystems;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

  private final SwerveDrive swerveDrive;
  private final Field2d m_field = new Field2d();

  // Current maximum linear drive speed used by drive(...) for scaling.
  private static double MaxDriveSpeed = DriveConstants.kMaxSpeedMetersPerSecond;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    try {
      swerveDrive = new SwerveParser(
          new File(Filesystem.getDeployDirectory(), "swerve"))
          .createSwerveDrive(DriveConstants.kMaxSpeedMetersPerSecond);
    } catch (Exception e) {
      throw new RuntimeException("Failed to create YAGSL SwerveDrive", e);
    }

    SmartDashboard.putData("Field", m_field);
    zeroHeading();

    // -------------- PathPlanner Configuration -------------- //
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
      config = null;
    }

    if (config != null) {
      AutoBuilder.configure(
          this::getPose,
          this::resetPose,
          this::getRobotRelativeSpeeds,
          (speeds, feedforwards) -> driveRobotRelative(speeds),
          new PPHolonomicDriveController(
              new PIDConstants(0.2, 0.0, 0),
              new PIDConstants(0.2, 0.0, 0)),
          config,
          () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this);
    }
  }

  @Override
  public void periodic() {
    m_field.setRobotPose(getPose());

    SmartDashboard.putNumber("Gyro Angle", getHeading());
    SmartDashboard.putNumber("Gyro Rate", getTurnRate());

    Pose2d pose = getPose();
    SmartDashboard.putNumber("Pose X (m)", pose.getX());
    SmartDashboard.putNumber("Pose Y (m)", pose.getY());
    SmartDashboard.putNumber("Pose Rot (deg)", pose.getRotation().getDegrees());

    var nt = NetworkTableInstance.getDefault().getTable("limelight");
    SmartDashboard.putNumber("limelight_tx", nt.getEntry("tx").getDouble(0.0));
    SmartDashboard.putNumber("limelight_ty", nt.getEntry("ty").getDouble(0.0));
    SmartDashboard.putNumber("limelight_ta", nt.getEntry("ta").getDouble(0.0));

    frc.robot.Dashboard.setSlowModeBanner(isSlowModeActive(), getSpeedDivisor());
  }

  public static boolean isSlowModeActive() {
    return MaxDriveSpeed < DriveConstants.kMaxSpeedMetersPerSecond;
  }

  public static double getSpeedDivisor() {
    return DriveConstants.kMaxSpeedMetersPerSecond / MaxDriveSpeed;
  }

  public static void setDriveSpeed(double divisor) {
    MaxDriveSpeed = DriveConstants.kMaxSpeedMetersPerSecond / divisor;
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public void resetPose(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
  }

  public double GetPigeonDegrees() {
    double wrappedDegrees = MathUtil.inputModulus(
        swerveDrive.getYaw().getDegrees(), -180, 180);
    SmartDashboard.putNumber("Pigeon Degrees", wrappedDegrees);
    return wrappedDegrees;
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return swerveDrive.getRobotVelocity();
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    swerveDrive.drive(robotRelativeSpeeds);
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    double xSpeedDelivered = xSpeed * MaxDriveSpeed;
    double ySpeedDelivered = ySpeed * MaxDriveSpeed;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    swerveDrive.drive(
        new Translation2d(xSpeedDelivered, ySpeedDelivered),
        rotDelivered,
        fieldRelative,
        false);
  }

  public void setX() {
    swerveDrive.lockPose();
  }

  public void zeroHeading() {
    swerveDrive.zeroGyro();
  }

  public double getHeading() {
    return swerveDrive.getYaw().getDegrees();
  }

  public double getTurnRate() {
    return swerveDrive.getGyroRotation3d().getZ();
  }

  public Field2d getField() {
    return m_field;
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }
}
