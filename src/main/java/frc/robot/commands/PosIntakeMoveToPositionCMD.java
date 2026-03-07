package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PosIntakeSubsystem;

/**
 * Move the positional intake until the encoder reaches a target position (with safety timeout).
 */
public class PosIntakeMoveToPositionCMD extends Command {
  private final PosIntakeSubsystem posIntakeSubsystem;
  private final double targetPosition;
  private final double speed;
  private final double timeout;
  private final Timer timer = new Timer();

  /**
   * @param posIntakeSubsystem the subsystem
   * @param targetPosition normalized encoder target (0..1+). If encoder wraps near 0, callers should use values near 1.0
   * @param speed motor speed to apply while moving (negative/up or positive/down per hardware)
   * @param timeout seconds to wait before aborting; pass 0 for no timeout (not recommended)
   */
  public PosIntakeMoveToPositionCMD(PosIntakeSubsystem posIntakeSubsystem, double targetPosition, double speed, double timeout) {
    this.posIntakeSubsystem = posIntakeSubsystem;
    this.targetPosition = targetPosition;
    this.speed = speed;
    this.timeout = timeout;
    addRequirements(posIntakeSubsystem);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    // Command continuously drives the pos intake until isFinished() returns true.
    posIntakeSubsystem.setPosIntakeSpeed(speed);
  }

  @Override
  public boolean isFinished() {
    // Timeout safety
    if (timeout > 0 && timer.hasElapsed(timeout)) {
      return true;
    }

    double pos = posIntakeSubsystem.getPosIntakePosition();
    if (Double.isNaN(pos)) {
      // Encoder invalid/uninitialized — keep trying until timeout
      return false;
    }

    // Normalize wrap-around: very small values near 0 represent values near 1.0
    if (pos < 0.05) {
      pos += 1.0;
    }

    return pos >= targetPosition;
  }

  @Override
  public void end(boolean interrupted) {
    posIntakeSubsystem.setPosIntakeSpeed(0);
    timer.stop();
  }
}
