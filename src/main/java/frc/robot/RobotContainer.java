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
//Subsystems
import frc.robot.subsystems.DriveSubsystem;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
public class RobotContainer {
  //Intialize the Autochooser for selecting autos in SmartDashboard\\
  private final SendableChooser<Command> autoChooser;
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_robotDrive;
  private final ExampleSubsystem exampleSubsystem;
  
  // Initializes the controller (Xbox)
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OIConstants.kOperatorControllerPort);

  // Set this to true to run the CAN checker at startup (probes SparkMax IDs).
  // Default is false — use the Shuffleboard button to run on demand.
  private static final boolean RUN_CAN_CHECKER = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Optionally run the CAN checker before instantiating subsystems that create SparkMax
    if (RUN_CAN_CHECKER) {
      CANChecker.runChecks();
    }

    // Initializes the subsystems
    m_robotDrive = new DriveSubsystem();
    // Initialize programmatic dashboard layout (creates Shuffleboard tabs/widgets)
    Dashboard.init(m_robotDrive);
  // Ensure CAN Checks widgets are present on Shuffleboard (doesn't probe hardware)
  CANChecker.createWidgets();
    exampleSubsystem = new ExampleSubsystem();
    // Gets controller binding
    configureBindings();
    // Sets joystick to drive\
    

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
    
      // Add an on-demand Shuffleboard toggle/button to run the CAN checker.
      var canTab = Shuffleboard.getTab("CAN Checks");
    var runEntry = canTab.add("Run CAN Checker", false)
      .withWidget(BuiltInWidgets.kToggleButton)
      .withPosition(7, 0)
      .withSize(1, 1)
      .getEntry();

      // When the button becomes true, run CANChecker once and reset the toggle.
      // Require the robot to be disabled for safety; if not disabled, write a helpful
      // message and reset the toggle so the user gets immediate feedback.
      new Trigger(() -> runEntry.getBoolean(false))
        .onTrue(new InstantCommand(() -> {
          boolean disabled = DriverStation.isDisabled();
          if (!disabled) {
            System.out.println("[RobotContainer] CAN Checker requested while robot enabled — aborting. Disable robot and press the button again.");
            SmartDashboard.putString("CAN Checks/lastRunError", "Probe aborted: robot must be disabled");
            runEntry.setBoolean(false);
            return;
          }
          System.out.println("[RobotContainer] Running CAN Checker (robot disabled).");
          CANChecker.runChecks();
          runEntry.setBoolean(false);
        }));

      // Fallback manual entry: add a manual boolean entry the user can toggle
      // from Shuffleboard/SmartDashboard if the widget above isn't interactive.
      var manualEntry = canTab.add("Run CAN Checker (Manual)", false)
        .withWidget(BuiltInWidgets.kToggleButton)
        .withPosition(7, 1)
        .withSize(1, 1)
        .getEntry();

      // Start a small scheduled task to poll the manualEntry. When it becomes true
      // and robot is disabled, run the checker and reset the entry.
      ScheduledExecutorService scheduler = Executors.newSingleThreadScheduledExecutor();
      scheduler.scheduleAtFixedRate(() -> {
        try {
          boolean requested = manualEntry.getBoolean(false);
          if (requested) {
            if (!DriverStation.isDisabled()) {
              System.out.println("[RobotContainer] Manual CAN Checker requested while robot enabled — aborting.");
              SmartDashboard.putString("CAN Checks/lastRunError", "Manual probe aborted: robot must be disabled");
              manualEntry.setBoolean(false);
              return;
            }
            System.out.println("[RobotContainer] Running CAN Checker (manual entry detected).");
            CANChecker.runChecks();
            manualEntry.setBoolean(false);
          }
        } catch (Exception e) {
          System.out.println("[RobotContainer] Error polling manual CAN entry: " + e);
        }
      }, 0, 300, TimeUnit.MILLISECONDS);
  }

  // Sets up controller bindings
  private void configureBindings() {
    // Initiallizyng Buttons
    //m_operatorController.a().whileTrue(new ExampleCommand(exampleSubsystem, 0.5));
    //m_operatorController.leftTrigger(0.5).whileTrue(new ExampleCommand(exampleSubsystem, 0.3));
    // Bind the operator controller Start button as a fallback to run the CAN checker
    // while the robot is disabled. This is useful when Shuffleboard widgets are not
    // allowing writes from the client.
    try {
      m_operatorController.start().onTrue(new InstantCommand(() -> {
        if (!DriverStation.isDisabled()) {
          System.out.println("[RobotContainer] Controller-triggered CAN check aborted: robot must be disabled");
          SmartDashboard.putString("CAN Checks/lastRunError", "Controller probe aborted: robot must be disabled");
          return;
        }
        System.out.println("[RobotContainer] Controller-triggered CAN check starting.");
        CANChecker.runChecks();
      }));
    } catch (Exception e) {
      // Defensive: if controller library changes or no controller connected, log and continue.
      System.out.println("[RobotContainer] Failed to bind controller CAN check: " + e);
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // The chosen auto in autoChooser is returned
    return autoChooser.getSelected();
  }

  /**
   * Expose the DriveSubsystem instance so Robot (and tests) can access it without creating
   * duplicate hardware objects.
   */
  public DriveSubsystem getDriveSubsystem() {
    return m_robotDrive;
  }
}
