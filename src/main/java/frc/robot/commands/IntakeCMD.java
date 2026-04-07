package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OtherMotorsSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCMD extends Command {
  private final IntakeSubsystem IntakeSubsystem;
  private final double speed;
  private final boolean forward;

  /** Creates a new IntakeCommand. */
  public IntakeCMD(IntakeSubsystem m_IntakeSubsystem, double speed, boolean forward) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.IntakeSubsystem = m_IntakeSubsystem;
    this.speed = speed;
    this.forward = forward;
    addRequirements(m_IntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (forward)
      IntakeSubsystem.setIntakeSpeed(speed, forward);
    else {
      IntakeSubsystem.setIntakeSpeed(-speed, forward);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    IntakeSubsystem.setIntakeSpeed(0, forward);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}