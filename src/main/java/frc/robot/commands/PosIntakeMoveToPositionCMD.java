package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PosIntakeSubsystem;

/**
 * Move the positional intake until the encoder reaches a target position (with safety timeout).
 */
public class PosIntakeMoveToPositionCMD extends Command {
  private final PosIntakeSubsystem posIntakeSubsystem;
  //private final double targetPosition;
  private final double speed;
  private final double timeout;
  private final Timer autoTimer = new Timer();
  private double position;
  private double timer;

  /**
   * @param posIntakeSubsystem the subsystem
   * @param targetPosition normalized encoder target (0..1+). If encoder wraps near 0, callers should use values near 1.0
   * @param speed motor speed to apply while moving (negative/up or positive/down per hardware)
   * @param timeout seconds to wait before aborting; pass 0 for no timeout (not recommended)
   */
  public PosIntakeMoveToPositionCMD(PosIntakeSubsystem posIntakeSubsystem, double speed, double timeout) {
    this.posIntakeSubsystem = posIntakeSubsystem;
    //this.targetPosition = targetPosition;
    this.speed = speed;
    this.timeout = timeout;
    addRequirements(posIntakeSubsystem);
  }

  @Override
  public void initialize() {
    autoTimer.reset();
    autoTimer.start();
    timer = System.currentTimeMillis() + 1000;
  }

  @Override
  public void execute() {
    // Command continuously drives the pos intake until isFinished() returns true.
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

  @Override
  public boolean isFinished() {
    // Timeout safety
    if (timeout > 0 && autoTimer.hasElapsed(timeout)) {
      return true;
    }

    // double pos = posIntakeSubsystem.getPosIntakePosition();
    // if (Double.isNaN(pos)) {
    //   // Encoder invalid/uninitialized — keep trying until timeout
    //   return false;
    // }

    // Normalize wrap-around: very small values near 0 represent values near 1.0
    // if (pos < 0.05) {
    //   pos += 1.0;
    // }

    return false;
  }

  @Override
  public void end(boolean interrupted) {
    posIntakeSubsystem.setPosIntakeSpeed(0);
    autoTimer.stop();
  }
}
