// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.OtherMotorsSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReleaseCMD extends Command {
  private final double speed;
  private final OtherMotorsSubsystem otherMotorsSubsystem;

  /** Creates a new ReleaseCMD. */
  public ReleaseCMD(OtherMotorsSubsystem m_otherMotorsSubsystem, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.otherMotorsSubsystem = m_otherMotorsSubsystem;
    this.speed = speed;
    addRequirements(m_otherMotorsSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    otherMotorsSubsystem.setReleaseSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    otherMotorsSubsystem.setReleaseSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
