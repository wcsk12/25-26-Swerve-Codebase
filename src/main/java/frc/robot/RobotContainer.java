package frc.robot;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

//Pathplanner Imports\\
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
//Math Imports\\
import edu.wpi.first.math.MathUtil;
//SmartDashboard Imports\\
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//Commands and Controllers\\
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController; 
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//Constants
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
//Commands
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.ClimberCMD;
import frc.robot.commands.IntakeCMD;
import frc.robot.commands.LowerSpeedCMD;
import frc.robot.commands.ReleaseCMD;
import frc.robot.commands.ShooterCMD;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//Subsystems
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OtherMotorsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.SetShooterSpeed;
//Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.networktables.NetworkTableEntry;
//Other
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class RobotContainer {
  //Intialize the Autochooser for selecting autos in SmartDashboard\\
  private final SendableChooser<Command> autoChooser;
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_robotDrive;
  private final OtherMotorsSubsystem m_OtherMotorsSubsystem;
  private final IntakeSubsystem m_IntakeSubsystem;
  private final ShooterSubsystem m_ShooterSubsystem;
  private final ClimberSubsystem m_ClimberSubsystem;
  
  // Initializes the controller (Xbox)
  private final CommandXboxController m_operatorController = //Operator Controller
      new CommandXboxController(OIConstants.kOperatorControllerPort);
  private final CommandXboxController m_driverController = //Driver Controller -Change to FlightSim
    new CommandXboxController(OIConstants.kDriverControllerPort);
  // Fallback raw joystick (in case client uses a non-Xbox joystick on the driver port)
  private final Joystick m_driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
  private final Joystick m_operatorJoystick = new Joystick(OIConstants.kOperatorControllerPort);
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
    m_OtherMotorsSubsystem = new OtherMotorsSubsystem();
    m_IntakeSubsystem = new IntakeSubsystem();
    m_ShooterSubsystem = new ShooterSubsystem();
    m_ClimberSubsystem = new ClimberSubsystem();
    // Initialize programmatic dashboard layout (creates Shuffleboard tabs/widgets)
    Dashboard.init(m_robotDrive);
  // Ensure CAN Checks widgets are present on Shuffleboard (doesn't probe hardware)
  CANChecker.createWidgets();
    
    // Gets controller binding
    configureBindings();
    // Sets joystick to drive
    m_robotDrive.setDefaultCommand(
      new RunCommand(
        () -> m_robotDrive.drive(
        -MathUtil.applyDeadband(m_driverController.getRawAxis(1), OIConstants.kDriveDeadband), 
        MathUtil.applyDeadband(m_driverController.getRawAxis(0), OIConstants.kDriveDeadband), 
        MathUtil.applyDeadband(m_driverController.getRawAxis(4), OIConstants.kDriveDeadband), 
        true, 0.02),
      m_robotDrive));
    // -------------------------------- PathPlanner Code -------------------------------- \\
    // For convenience a programmer could change this when going to competition.
      boolean isCompetition = false;
    // Build an auto chooser. This will use Commands.none() as the default option.
        // As an example, this will only show autos that start with "comp" while at
        // competition as defined by the programmer
        autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
            (stream) -> isCompetition
            ? stream.filter(auto -> auto.getName().startsWith("BLU"))
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

// (previous toggle state removed — ReleaseandShoot is a one-shot sequence now)

// Shooter Command for Autos \\
  public Command getShootSequence() {
    // ShooterCMD is a persistent command (isFinished() == false) and would block a sequence.
    // Use InstantCommands to set/clear speeds so the sequence can progress.
    return Commands.sequence(
        // 1. Start shooter motor, wait for it to reach speed
        new InstantCommand(() -> m_ShooterSubsystem.setShooterSpeed(DriveConstants.ShooterMotorSpeed)),
        new WaitCommand(1.1), // Adjust wait time for spin-up //Takes 0.8 sec for other motor to start.
        // 2. Run feeder/release motor to fire
        new InstantCommand(() -> m_OtherMotorsSubsystem.setReleaseSpeed(-DriveConstants.ReleaseMotorSpeed)),
        new InstantCommand(() -> m_IntakeSubsystem.setIntakeSpeed(-DriveConstants.IntakeMotorSpeed)),
        new WaitCommand(6),
        // 3. Stop both
        new InstantCommand(() -> m_IntakeSubsystem.setIntakeSpeed(0)),
        new InstantCommand(() ->  m_ShooterSubsystem.setShooterSpeed(DriveConstants.ShootMotorSpeedOFF)),
        new InstantCommand(() -> m_OtherMotorsSubsystem.setReleaseSpeed(0))
        );
}
// Limelight Release and shoot simultaneously \\
  // public Command ReleaseandShootWithLimelight() { //shooter and release combined.
  //   // Run a one-shot sequence: start shooter, wait to spin up, run release, then stop both.
  //     return Commands.sequence(
  //       new InstantCommand(() -> m_ShooterSubsystem.setShooterSpeed(DriveConstants.ShooterMotorSpeed), m_ShooterSubsystem),
  //       new WaitCommand(1.1), // headstart for shooter spin-up
  //       new InstantCommand(() -> m_OtherMotorsSubsystem.setReleaseSpeed(-DriveConstants.ReleaseMotorSpeed), m_OtherMotorsSubsystem)
  //       );
  //     }

    
  // public Command ReleaseandShootWithoutLimelight(int Negative) {
  //   return Commands.sequence( 
  //       new InstantCommand(() -> m_ShooterSubsystem.setShooterSpeed(Negative * DriveConstants.ShooterMotorSpeed), m_ShooterSubsystem),
  //       new WaitCommand(1.1), // headstart for shooter spin-up
  //       new InstantCommand(() -> m_OtherMotorsSubsystem.setReleaseSpeed(Negative * -DriveConstants.ReleaseMotorSpeed), m_OtherMotorsSubsystem)
  //       );
  // }
  // public Command ReleaseandShootOFF() {
  //   return Commands.sequence(
        
  //       new InstantCommand(() -> m_ShooterSubsystem.setShooterSpeed(DriveConstants.ShootMotorSpeedOFF), m_ShooterSubsystem),
  //       new InstantCommand(() -> m_OtherMotorsSubsystem.setReleaseSpeed(0), m_OtherMotorsSubsystem)
  //   );
  // }
      //   new InstantCommand(() -> m_ShooterSubsystem.setShooterSpeed(DriveConstants.ShooterMotorSpeed)),
      //   new WaitCommand(0.5), // Adjust wait time for spin-up //Takes 0.8 sec for other motor to start.
      //   // 2. Run feeder/release motor to fire
      //   new InstantCommand(() -> m_OtherMotorsSubsystem.setReleaseSpeed(-DriveConstants.ReleaseMotorSpeed));
      //     })
      //   );
      // }
        //50/50 w way to talj lol
        
  // Sets up controller bindings
  
  private void configureBindings() {
    // Initiallizyng Buttons
        // Commands that go to PathPlanner
    //NamedCommands.registerCommand("[Pathplanner Name]", [Command to run]);
    
  NamedCommands.registerCommand("Align", new AutoAlignCommand(m_robotDrive, 0, AutoAlignCommand.Mode.FULL_ALIGN).withTimeout(1)); // ALign
  //NamedCommands.registerCommand("Shoot!", new ShooterCMD(m_ShooterSubsystem, DriveConstants.ShooterMotorSpeed).withTimeout(1)); // Shoot
  //NamedCommands.registerCommand("Release", new ReleaseCMD(m_OtherMotorsSubsystem, DriveConstants.ReleaseMotorSpeed).withTimeout(1)); // Release up to shooter
  NamedCommands.registerCommand("Intake!", new IntakeCMD(m_IntakeSubsystem, DriveConstants.IntakeMotorSpeed, true)); // Intake the Fuel! - Potentially remove withTimeout due to parallel deadline command.
  NamedCommands.registerCommand("Shoot!", getShootSequence()); // Release + Shoot!

        // Driver A button: while held, run auto-align to AprilTag (0.6m target distance)
    try {
      // Debug: log when A is pressed
      m_driverController.a().onTrue(new InstantCommand(() -> System.out.println("[RobotContainer] Driver A pressed")));
      m_driverController.a().whileTrue(new AutoAlignCommand(m_robotDrive, 0.6));
  // Driver X button: one-shot full autonomous alignment (translation + rotation)
  // Use a short timeout as a safety net so it doesn't run forever if pose estimates fail.
  m_driverController.x().toggleOnTrue(new AutoAlignCommand(m_robotDrive, 0.6, frc.robot.commands.AutoAlignCommand.Mode.FULL_ALIGN).withTimeout(5));
      // ------------------------------------------ Intake ------------------------------------------ \\
  m_driverController.leftBumper().whileTrue(new IntakeCMD(m_IntakeSubsystem, DriveConstants.IntakeMotorSpeed, true)); //Intake speed (Forwards)
  m_driverController.rightBumper().whileTrue(new IntakeCMD(m_IntakeSubsystem, DriveConstants.IntakeMotorSpeed, false));
      // ------------------------------------------ Shooter ------------------------------------------ \\
  // Toggle the shooter command (start/stop) directly. Do NOT wrap command creation in an InstantCommand.
      //m_operatorController.leftBumper().whileTrue(new ShooterCMD(m_ShooterSubsystem, DriveConstants.ShooterMotorSpeed, m_operatorController));
      //EXPERIMENTAL
      m_operatorController.leftBumper().toggleOnTrue(new InstantCommand(() -> m_ShooterSubsystem.setSpeed(SetShooterSpeed.SlowSpeed))).toggleOnFalse(new InstantCommand(() -> m_ShooterSubsystem.setSpeed(SetShooterSpeed.ZeroSpeed)));
  //  m_operatorController.rightBumper().toggleOnTrue((ReleaseandShootWithoutLimelight(1)));
  //  m_operatorController.rightBumper().toggleOnFalse((ReleaseandShootOFF()));
  //   m_operatorController.leftBumper().toggleOnTrue(ReleaseandShootWithoutLimelight(-1));
  //  m_operatorController.leftBumper().toggleOnFalse(ReleaseandShootOFF());
   // LIMELIGHT SHOOTER \\
   //m_operatorController.b().whileTrue((ReleaseandShootWithLimelight()));
   //m_operatorController.b().whileFalse((ReleaseandShootOFF()));
      // ------------------------------------------ Release ------------------------------------------ \\
      m_operatorController.rightBumper().whileTrue(new ReleaseCMD(m_OtherMotorsSubsystem, DriveConstants.ReleaseMotorSpeed));
      // ------------------------------------------ Climber ------------------------------------------ \\
      m_operatorController.y().whileTrue(new ClimberCMD(m_ClimberSubsystem, DriveConstants.ClimberSpeed));
      // ------------------------------------------ LowerSpeed ------------------------------------------ \\
  // Toggle the LowerSpeedCMD directly so press-on -> schedule the command, press-again -> cancel it
  m_driverController.b().toggleOnTrue(new frc.robot.commands.LowerSpeedCMD(m_robotDrive, 2)); //set to two when pressed! -B button
      // ------------------------------------------ Reset Pigeon ------------------------------------------ \\
      m_operatorController.a().whileTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive)); //Pigeon Reset

      //Back up Buttons! (Regular Controller) \\

      // Also bind raw joystick button 1 as a fallback for non-Xbox controllers
      new JoystickButton(m_driverJoystick, 1).onTrue(new InstantCommand(() -> System.out.println("[RobotContainer] Joystick button 1 pressed")));
      new JoystickButton(m_driverJoystick, 1).whileTrue(new AutoAlignCommand(m_robotDrive, 0.6)); //Potentially just use onTrue - A BUTTON
      // ------------------------------------------ Intake ------------------------------------------ \\
  new JoystickButton(m_driverJoystick, 5).whileTrue(new IntakeCMD(m_IntakeSubsystem, DriveConstants.IntakeMotorSpeed, true)); //Intake speed (Forwards) - LEFT BUTTON
   new JoystickButton(m_driverJoystick, 6).whileTrue(new IntakeCMD(m_IntakeSubsystem, DriveConstants.IntakeMotorSpeed, false)); //Intake speed (Backwards) - RIGHT BUTTON
        // ------------------------------------------ Shooter ------------------------------------------ \\
  // Run the combined one-shot shoot sequence when the operator presses button 6.
  // new JoystickButton(m_operatorJoystick, 6).toggleOnTrue(ReleaseandShootWithoutLimelight(1)); // Shooter - RIGHT BUTTON --Shoot!
  // new JoystickButton(m_operatorJoystick, 6).toggleOnFalse(ReleaseandShootOFF());
  // new JoystickButton(m_operatorJoystick, 5).toggleOnTrue(ReleaseandShootWithoutLimelight(-1));
  // new JoystickButton(m_operatorJoystick, 5).toggleOnFalse(ReleaseandShootOFF());
       // LIMELIGHT SHOOTER \\
   //new JoystickButton(m_operatorJoystick, 1).whileTrue((ReleaseandShootWithLimelight()));
   //new JoystickButton(m_operatorJoystick, 1).whileFalse((ReleaseandShootOFF()));
      // ------------------------------------------ Climber ------------------------------------------ \\
      //Climber button binding goes here.
      // ------------------------------------------ LowerSpeed ------------------------------------------ \\
  // Toggle LowerSpeedCMD directly rather than constructing it inside an InstantCommand.
  new JoystickButton(m_driverJoystick, 2).toggleOnTrue(new LowerSpeedCMD(m_robotDrive, 2)); //set to two when pressed! - B Button

    } catch (Exception e) {
      System.out.println("[RobotContainer] Failed to bind AutoAlignCommand to A button: " + e);
    }

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
