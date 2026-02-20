package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.OtherMotorsSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCMD extends Command {
  private final OtherMotorsSubsystem otherMotorsSubsystem;
  private final double speed;
  private final boolean forward;

  /** Creates a new IntakeCommand. */
  public IntakeCMD(OtherMotorsSubsystem m_otherMotorsSubsystem, double speed, boolean forward) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.otherMotorsSubsystem = m_otherMotorsSubsystem;
    this.speed = speed;
    this.forward = forward;
    addRequirements(m_otherMotorsSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (forward == true)
      otherMotorsSubsystem.setIntakeSpeed(speed);
    else {
      otherMotorsSubsystem.setIntakeSpeed(speed *-1);
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