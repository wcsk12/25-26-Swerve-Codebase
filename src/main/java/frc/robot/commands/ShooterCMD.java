// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.subsystems.MiscSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterCMD extends Command {
  private final MiscSubsystem miscSubsystem;
  private final double speed;
  private final CommandXboxController controller;
  
  /** Creates a new ShooterCMD. */
  public ShooterCMD(MiscSubsystem miscSubsystem, double speed, CommandXboxController controller) {
    this.miscSubsystem = miscSubsystem;
    this.speed = speed;
    this.controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(miscSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Robot.limelight_id() == 10 || Robot.limelight_id() == 25){
      if (1.0 > Math.abs(speed)){
      miscSubsystem.setShooterSpeed(Math.pow(0.1, speed));
      }
    }
    else{
      miscSubsystem.setShooterSpeed(speed);
    }
    controller.setRumble(RumbleType.kBothRumble, 1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    miscSubsystem.setShooterSpeed(0);
    controller.setRumble(RumbleType.kBothRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
