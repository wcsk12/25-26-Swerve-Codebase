// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PosIntakeSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PosIntakeBumperCMD extends Command {
  private final PosIntakeSubsystem posIntakeSubsystem;
  private final double speed;
  private double position;
  private double distance;

  /** Creates a new PosIntakeCMD. */
  public PosIntakeBumperCMD(PosIntakeSubsystem posIntakeSubsystem, double speed) {
    this.posIntakeSubsystem = posIntakeSubsystem;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(posIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    position = posIntakeSubsystem.getPosIntakePosition(); //position is 0 at starting position, 
      //counts down from .999 to about .600 where .600 is resting on the bumper
    if (position < 0.05) {
      position = 1.0;
    }
    System.out.println("BumperCMD: " + position);
    if (position > .65) {
      posIntakeSubsystem.setPosIntakeSpeed(speed);
    } else {
      posIntakeSubsystem.setPosIntakeSpeed(speed * 0.2);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    posIntakeSubsystem.setPosIntakeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
