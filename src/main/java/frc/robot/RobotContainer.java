package frc.robot;
//Pathplanner Imports\\
import com.pathplanner.lib.auto.AutoBuilder;
//Math Imports\\
import edu.wpi.first.math.MathUtil;
//SmartDashboard Imports\\
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//Commands and Controllers\\
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController; 
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//Constants Imports\\
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.AlignToAprilTag;
//Subsystems
import frc.robot.subsystems.DriveSubsystem;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
import frc.robot.subsystems.ExampleSubsystem;
public class RobotContainer {
  //Intialize the Autochooser for selecting autos in SmartDashboard\\
  private final SendableChooser<Command> autoChooser;
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_robotDrive;
  private final ExampleSubsystem exampleSubsystem;
  
  // Initializes the controller (Xbox)
  private final CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Initializes the subsystems
    m_robotDrive = new DriveSubsystem();
    exampleSubsystem = new ExampleSubsystem();
    // Gets controller binding
    configureBindings();
    // Sets joystick to drive
    m_robotDrive.setDefaultCommand(
      new RunCommand(
        () -> m_robotDrive.drive(
        -MathUtil.applyDeadband(m_driverController.getRawAxis(1), OIConstants.kDriveDeadband), 
        -MathUtil.applyDeadband(m_driverController.getRawAxis(0), OIConstants.kDriveDeadband), 
        MathUtil.applyDeadband(m_driverController.getRawAxis(4), OIConstants.kDriveDeadband), 
        true),
      m_robotDrive));
    // -------------------------------- PathPlanner Code -------------------------------- \\
    // For convenience a programmer could change this when going to competition.
      boolean isCompetition = false;
    // Build an auto chooser. This will use Commands.none() as the default option.
        // As an example, this will only show autos that start with "comp" while at
        // competition as defined by the programmer
        autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
            (stream) -> isCompetition
            ? stream.filter(auto -> auto.getName().startsWith("comp"))
            : stream
        );

        //Put the Auto Chooser on SmartDashboard so we can select autos.
        SmartDashboard.putData("Auto Chooser", autoChooser);
  }
  // Sets up controller bindings
  private void configureBindings() {
    // Initiallizyng Buttons
    // Example buttons (uncomment and tune as needed)
    // Run example command while A is held
    //m_driverController.a().whileTrue(new ExampleCommand(exampleSubsystem, 0.5));
    // Align to an AprilTag while A is held (0.6 m target distance)
    m_driverController.a().whileTrue(new AlignToAprilTag(m_robotDrive, 0.6));
    //m_driverController.leftTrigger(0.5).whileTrue(new ExampleCommand(exampleSubsystem, 0.3));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // The chosen auto in autoChooser is returned
    return autoChooser.getSelected();
  }
}
