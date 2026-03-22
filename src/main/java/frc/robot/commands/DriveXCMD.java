package frc.robot.commands;
 
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveXCMD extends Command {
  private final DriveSubsystem m_robotDrive;

  /** Creates a new ExampleCommand. */
  public DriveXCMD(DriveSubsystem m_robotDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_robotDrive = m_robotDrive;
    addRequirements(m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_robotDrive.setX();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}