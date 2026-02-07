package frc.robot;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * Programmatic Shuffleboard layout so every driver station sees the same widgets.
 */
public final class Dashboard {
  private Dashboard() {}

  public static void init(DriveSubsystem drive) {
    // Drive tab
    ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");

    // Field visualization (reuse DriveSubsystem's Field2d instance)
    driveTab.add("Field", drive.getField()).withSize(6, 4).withPosition(0, 0);

    // Gyro / pose
    driveTab.add("Gyro Angle", 0.0).withSize(2, 1).withPosition(6, 0).withWidget(BuiltInWidgets.kTextView);
    driveTab.add("Gyro Rate", 0.0).withSize(2, 1).withPosition(8, 0).withWidget(BuiltInWidgets.kTextView);

    driveTab.add("Pose X (m)", 0.0).withSize(1, 1).withPosition(6, 1).withWidget(BuiltInWidgets.kTextView);
    driveTab.add("Pose Y (m)", 0.0).withSize(1, 1).withPosition(7, 1).withWidget(BuiltInWidgets.kTextView);
    driveTab.add("Pose Rot (deg)", 0.0).withSize(2, 1).withPosition(8, 1).withWidget(BuiltInWidgets.kTextView);

    // Module speeds
    driveTab.add("FL Speed (m/s)", 0.0).withSize(2, 1).withPosition(0, 4);
    driveTab.add("FR Speed (m/s)", 0.0).withSize(2, 1).withPosition(2, 4);
    driveTab.add("BL Speed (m/s)", 0.0).withSize(2, 1).withPosition(4, 4);
    driveTab.add("BR Speed (m/s)", 0.0).withSize(2, 1).withPosition(6, 4);

    // Module angles
    driveTab.add("FL Angle (deg)", 0.0).withSize(2, 1).withPosition(0, 5);
    driveTab.add("FR Angle (deg)", 0.0).withSize(2, 1).withPosition(2, 5);
    driveTab.add("BL Angle (deg)", 0.0).withSize(2, 1).withPosition(4, 5);
    driveTab.add("BR Angle (deg)", 0.0).withSize(2, 1).withPosition(6, 5);

    // Legacy encoder values
    driveTab.add("fLeftDrive", 0.0).withSize(1, 1).withPosition(0, 6);
    driveTab.add("fRightDrive", 0.0).withSize(1, 1).withPosition(1, 6);
    driveTab.add("bLeftDrive", 0.0).withSize(1, 1).withPosition(2, 6);
    driveTab.add("bRightDrive", 0.0).withSize(1, 1).withPosition(3, 6);

    // Vision tab
    ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");
    visionTab.add("limelight_tx", 0.0).withSize(2, 1).withPosition(0, 0);
    visionTab.add("limelight_ty", 0.0).withSize(2, 1).withPosition(2, 0);
    visionTab.add("limelight_ta", 0.0).withSize(2, 1).withPosition(4, 0);

  // AutoAlign telemetry (populated by AutoAlignCommand via SmartDashboard)
  visionTab.add("AutoAlign/tagCount", 0.0).withSize(1, 1).withPosition(0, 1);
  visionTab.add("AutoAlign/forwardErr", 0.0).withSize(2, 1).withPosition(1, 1);
  visionTab.add("AutoAlign/lateralErr", 0.0).withSize(2, 1).withPosition(3, 1);
  visionTab.add("AutoAlign/angleErr", 0.0).withSize(2, 1).withPosition(5, 1);
  visionTab.add("AutoAlign/status", "").withSize(2, 1).withPosition(7, 1).withWidget(BuiltInWidgets.kTextView);

  // Auto tab (Auto chooser is already published by RobotContainer to SmartDashboard as "Auto Chooser")
  ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
  autoTab.add("Auto Chooser", 0.0).withSize(3, 2).withPosition(0, 0);

    // Note: the numeric widgets above will automatically pick up values published to NetworkTables
    // by the DriveSubsystem (SmartDashboard.putNumber) and Limelight entries.
  }
}
