package frc.robot.commands;
 
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MiscSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCMD extends Command {
  private final MiscSubsystem miscSubsystem;
  private final double speed;

  /** Creates a new ExampleCommand. */
  public IntakeCMD(MiscSubsystem miscSubsystem, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.miscSubsystem = miscSubsystem;
    this.speed = speed;
    addRequirements(miscSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    miscSubsystem.setIntakeSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    miscSubsystem.setIntakeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}