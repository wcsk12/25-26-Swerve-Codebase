package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Simple on-robot sanity check that points the four swerve modules to distinct
 * angles (0, 90, 180, -90) and publishes the module states. Run while disabled
 * first to verify wiring and mapping.
 */
public class SwerveModuleSelfTestCMD extends Command {
  private final DriveSubsystem drive;
  private long endTimeMs;

  public SwerveModuleSelfTestCMD(DriveSubsystem drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // Set distinct angles for quick visual verification (no driving speed)
    SwerveModuleState[] states = new SwerveModuleState[] {
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0)),   // FL -> 0 deg
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(90.0)),  // FR -> 90 deg
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(180.0)), // BL -> 180 deg
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(-90.0))  // BR -> -90 deg
    };

    drive.setModuleStates(states);

    // Allow 2 seconds for modules to move and then sample
    endTimeMs = System.currentTimeMillis() + 2000;
  }

  @Override
  public void execute() {
    // Publish running status and raw absolute encoder values + configured offsets
    SmartDashboard.putString("SelfTest/Status", "Running");
    SmartDashboard.putNumber("SelfTest/FL Absolute (rad)", drive.getFrontLeftAbsoluteAngle());
    SmartDashboard.putNumber("SelfTest/FR Absolute (rad)", drive.getFrontRightAbsoluteAngle());
    SmartDashboard.putNumber("SelfTest/BL Absolute (rad)", drive.getRearLeftAbsoluteAngle());
    SmartDashboard.putNumber("SelfTest/BR Absolute (rad)", drive.getRearRightAbsoluteAngle());

    SmartDashboard.putNumber("SelfTest/FL Offset (rad)", drive.getFrontLeftOffset());
    SmartDashboard.putNumber("SelfTest/FR Offset (rad)", drive.getFrontRightOffset());
    SmartDashboard.putNumber("SelfTest/BL Offset (rad)", drive.getRearLeftOffset());
    SmartDashboard.putNumber("SelfTest/BR Offset (rad)", drive.getRearRightOffset());
  }

  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() > endTimeMs;
  }

  @Override
  public void end(boolean interrupted) {
    // Mark completion so operator can inspect the FL/FR/BL/BR angle values
    SmartDashboard.putString("SelfTest/Status", interrupted ? "Interrupted" : "Done");
  }
}
