// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PosIntakeSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PosIntakeShakeCMD extends Command {
  private final PosIntakeSubsystem posIntakeSubsystem;
  private final double speed;
  private double position;
  private double timer;

  /** Creates a new PosIntakeCMD. */
  public PosIntakeShakeCMD(PosIntakeSubsystem posIntakeSubsystem, double speed) {
    this.posIntakeSubsystem = posIntakeSubsystem;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(posIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = System.currentTimeMillis() + 1000;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    position = posIntakeSubsystem.getPosIntakePosition();
    // position normally ranges ~0.6..1.0 (wrap near 0). If the encoder is invalid,
    // getPosIntakePosition() returns NaN — treat that as 'unknown' and continue moving
    // so the command can still perform the shake behavior instead of immediately
    // thinking it's at the top.
    if (Double.isNaN(position)) {
      System.out.println("ShakeCMD: encoder invalid, forcing movement");
      // choose a safe working default that causes the command to attempt movement
      position = 0.7;
    } else {
      // handle wrap-around: small values near zero actually represent values near 1.0
      if (position > 0.95) {
        position = 0.0; // normalize wrap-around
      }
    }
    if (System.currentTimeMillis() > timer) {
      if (position > 0.2) { // bring posIntake up
        posIntakeSubsystem.setPosIntakeSpeed(speed);
      } else if (Math.floor(System.currentTimeMillis() / 1000) % 2 == 0) {
        posIntakeSubsystem.setPosIntakeSpeed(0.2 * speed);
      } else {
        posIntakeSubsystem.setPosIntakeSpeed(-0.2 * speed);
      }
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
