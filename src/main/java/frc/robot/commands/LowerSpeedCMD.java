// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LowerSpeedCMD extends Command {
  /** Creates a new LowerSpeedCMD. */
  private final double divisor;
  public LowerSpeedCMD(DriveSubsystem m_DriveSubsystem, double divisor) {
    this.divisor = divisor;
    addRequirements(m_DriveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  // Apply the lowered speed once when the command starts
  DriveSubsystem.setDriveSpeed(divisor);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // No-op: speed already set in initialize. Keeping execute lightweight.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  // Restore normal speed when the command ends/cancelled
  DriveSubsystem.setDriveSpeed(1);
  }
    
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
