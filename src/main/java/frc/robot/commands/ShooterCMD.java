// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.OtherMotorsSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterCMD extends Command {
  private final double speed;
  private final OtherMotorsSubsystem otherMotorsSubsystem;
  /** Creates a new ShooterCMD. */
  public ShooterCMD(OtherMotorsSubsystem m_otherMotorsSubsystem, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.otherMotorsSubsystem = m_otherMotorsSubsystem;
    this.speed = speed;
    addRequirements(otherMotorsSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { //If an apriltag is seen: 10-25 (Hub Tags), then, the motor speed is set to ty by the power of 0.1, else: it is set to 0.2.
    if (Robot.limelight_id() == 10 || Robot.limelight_id() == 25){
    double ty = Robot.limelight_range_proportional();
    if (1.0 > Math.abs(ty)) {
      otherMotorsSubsystem.setShooterSpeed(Math.pow(0.1, ty)); //set shooter's speed to ty by the power of 0.1
      }
    }
    else {
      otherMotorsSubsystem.setShooterSpeed(speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    otherMotorsSubsystem.setIntakeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
