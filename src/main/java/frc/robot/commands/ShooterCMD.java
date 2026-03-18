// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.OtherMotorsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterCMD extends Command {
  private final double speed;
  //private final boolean ON;
  private final ShooterSubsystem shootersubsystem;
  /** Creates a new ShooterCMD. */
  public ShooterCMD(ShooterSubsystem m_ShooterSubsystem, double speed) { //boolean ON
    // Use addRequirements() here to declare subsystem dependencies.
    this.shootersubsystem = m_ShooterSubsystem;
    this.speed = speed;
    //this.ON = ON;
    addRequirements(m_ShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { //If an apriltag is seen: 10-25 (Hub Tags), then, the motor speed is set to ty by the power of 0.1, else: it is set to 0.2.
    // if (Robot.limelight_id() == 10 || Robot.limelight_id() == 26){
    // double ty = Robot.limelight_range_proportional();
    // if (0.95 > Math.abs((ty+20.5)/41) && ty != 0)  {
    //   shootersubsystem.setShooterSpeed(ty*10);
    //   System.out.println("Works");
    //   System.out.println("Ty : " + ty);
    //   //shootersubsystem.setShooterSpeed(Math.pow(0.1, ty)); //set shooter's speed to ty by the power of 0.1
    //   //shootersubsystem.setShooterSpeed(speed);
    // }
    // else {
    //   shootersubsystem.setShooterSpeed(speed);
    //   System.out.println("Ty : " + ty);
    //   }
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //shootersubsystem.setShooterSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
