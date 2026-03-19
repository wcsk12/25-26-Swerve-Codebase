package frc.robot;
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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController; 
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//Constants Imports\\
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.IntakeCMD;
import frc.robot.commands.LED_command;
import frc.robot.commands.LauncherCMD;
import frc.robot.commands.PosIntakeBumperCMD;
import frc.robot.commands.PosIntakeMoveToPositionCMD;
import frc.robot.commands.PosIntakeShakeCMD;
import frc.robot.commands.PosIntakeZeroCMD;
import frc.robot.commands.ShooterCMD;
//Subsystems
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterSetSpeed;
import frc.robot.subsystems.ShooterSubsystem.ShooterSetSpeed;
import frc.robot.subsystems.PosIntakeSubsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  private final ShooterSubsystem shooterSubsystem;
  private final LauncherSubsystem launcherSubsystem;
  private final PosIntakeSubsystem posIntakeSubsystem;
  private final IntakeSubsystem intakeSubsystem;

  final LEDSubsystem ledSubsystem = new LEDSubsystem(1);
  
  // Initializes the controller (Xbox)
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OIConstants.kOperatorControllerPort);
  private final CommandXboxController m_driverController =
    new CommandXboxController(OIConstants.kDriverControllerPort);

  public static double speedMode = 1.0;

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


    // Initialize programmatic dashboard layout (creates Shuffleboard tabs/widgets)
    Dashboard.init(m_robotDrive);
  // Ensure CAN Checks widgets are present on Shuffleboard (doesn't probe hardware)
  CANChecker.createWidgets();
    shooterSubsystem = new ShooterSubsystem();
    launcherSubsystem = new LauncherSubsystem();
    posIntakeSubsystem = new PosIntakeSubsystem(); 
    intakeSubsystem = new IntakeSubsystem(); 
    //NamedCommand for Auto \\  //NamedCommands.registerCommand("[Pathplanner Name]", [Command to run]);
    //NamedCommands.registerCommand("IndexerCMD", new IndexerCMD(miscSubsystem, DriveConstants.indexerMotorSpeed).withTimeout(1));
    NamedCommands.registerCommand("IntakeCMD", getIntakeCommand());
    NamedCommands.registerCommand("ShootAndLaunch", getShootSequence());
    //NamedCommands.registerCommand("LauncherCMD", new LauncherCMD(miscSubsystem, DriveConstants.launcherMotorSpeed).withTimeout(1));
    NamedCommands.registerCommand("ShooterCMD", new ShooterCMD(shooterSubsystem, Robot.limelight_range_proportional(), m_operatorController).withTimeout(1));
    NamedCommands.registerCommand("LowerIntake", new PosIntakeBumperCMD(posIntakeSubsystem, DriveConstants.posIntakeMotorSpeed * 2.5).withTimeout(1.5));
    NamedCommands.registerCommand("ShootAndLaunchwithShake", getShootShakeCommand());
    //NamedCommands.registerCommand("RaiseIntake", new InstantCommand(() -> posIntakeSubsystem.setPosition(IntakePositions.zero)));
    // These were causing the robot to not instantiate ^
    // Gets controller binding
    configureBindings();
    // Sets joystick to drive
    m_robotDrive.setDefaultCommand(
      new RunCommand(
        () -> m_robotDrive.drive(
        -MathUtil.applyDeadband(m_driverController.getRawAxis(1) * speedMode, OIConstants.kDriveDeadband), 
        MathUtil.applyDeadband(m_driverController.getRawAxis(0) * speedMode, OIConstants.kDriveDeadband), 
        MathUtil.applyDeadband(-m_driverController.getRawAxis(4) * speedMode, OIConstants.kDriveDeadband), 
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

      // Changes LED depending on distance
      ledSubsystem.setDefaultCommand(new LED_command(ledSubsystem));
  }

  public Command getShootSequence() {
      return Commands.sequence(
        new InstantCommand(() -> shooterSubsystem.setShooterSpeed(DriveConstants.hardShooterTargetRPM)),
        new WaitCommand(1),
        new InstantCommand(() -> launcherSubsystem.setLauncherSpeed(DriveConstants.launcherMotorSpeed)),
        //new PosIntakeShakeCMD(posIntakeSubsystem, DriveConstants.posIntakeMotorSpeed).withTimeout(1.5),
        new WaitCommand(3),
        new InstantCommand(() -> shooterSubsystem.setShooterSpeed(0)),
        new InstantCommand(() -> launcherSubsystem.setLauncherSpeed(0))
        //new PosIntakeShakeCMD(posIntakeSubsystem, 0)
      );
  } 

  public Command getShootShakeCommand() {
    return Commands.sequence(
        new InstantCommand(() -> shooterSubsystem.setShooterSpeed(0.65)),
        new WaitCommand(1),
        new InstantCommand(() -> launcherSubsystem.setLauncherSpeed(DriveConstants.launcherMotorSpeed)),
  // Move posIntake until encoder reaches target (normalize wrap-around in the command)
  new PosIntakeMoveToPositionCMD(posIntakeSubsystem, 0.85, DriveConstants.posIntakeMotorSpeed, 1.5),
        new WaitCommand(3),
        new InstantCommand(() -> shooterSubsystem.setShooterSpeed(0)),
        new InstantCommand(() -> launcherSubsystem.setLauncherSpeed(0)),
        // Ensure we stop the posIntake and finish the sequence instead of scheduling
        // another PosIntakeShakeCMD (which never finishes). Use an InstantCommand to
        // explicitly stop the motor so the NamedCommand completes reliably.
        new InstantCommand(() -> posIntakeSubsystem.setPosIntakeSpeed(0))
      );
  }

  public Command getIntakeCommand() {
    return Commands.sequence(
      new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(DriveConstants.intakeMotorSpeed))
      //new PosIntakeBumperCMD(posIntakeSubsystem, DriveConstants.posIntakeMotorSpeed * 1.5)
    );
  }

  // Sets up controller bindings
  private void configureBindings() {
    // Initializing Buttons
      // Driver A button: while held, run auto-align to AprilTag (0.6m target distance)
    try {
      // Debug: log when A is pressed
      m_driverController.a().onTrue(new InstantCommand(() -> System.out.println("[RobotContainer] Driver A pressed " + speedMode)));
      m_driverController.a().whileTrue(new AutoAlignCommand(m_robotDrive, 0.6));
      //.5 speed mode
      m_driverController.leftBumper().onTrue(new InstantCommand(() -> speedMode = .5));
      m_driverController.leftBumper().onFalse(new InstantCommand(() -> speedMode = 1.0));
      //.25 speed mode
      m_driverController.rightBumper().onTrue(new InstantCommand(() -> speedMode = .25));
      m_driverController.rightBumper().onFalse(new InstantCommand(() -> speedMode = 1.0));
      m_driverController.back().onTrue(new InstantCommand(()-> DriveSubsystem.zeroHeading()));
      // Also bind raw joystick button 1 as a fallback for non-Xbox controllers
      /*new JoystickButton(m_driverJoystick, 1).onTrue(new InstantCommand(() -> System.out.println("[RobotContainer] Joystick button 1 pressed")));
      new JoystickButton(m_driverJoystick, 1).whileTrue(new AutoAlignCommand(m_robotDrive, 0.6));*/
    } catch (Exception e) {
      System.out.println("[RobotContainer] Failed to bind AutoAlignCommand to A button: " + e);
    }

    //m_operatorController.a().whileTrue(new ExampleCommand(exampleSubsystem, 0.5));
    //m_operatorController.leftTrigger(0.5).whileTrue(new ExampleCommand(exampleSubsystem, 0.3));
    // Bind the operator controller Start button as a fallback to run the CAN checker
    // while the robot is disabled. This is useful when Shuffleboard widgets are not
    // allowing writes from the client.
    // HAVING BOTH XBOX AND BUTTON BOARD CODE UNCOMMENTED CAN LEAD TO ISSUES, RECOMMENDED TO
    // COMMENT OUT WHICHEVER ONE YOU AREN'T USING
    
    /*try {
      m_operatorController.start().onTrue(new InstantCommand(() -> {
        if (!DriverStation.isDisabled()) {
          System.out.println("[RobotContainer] Controller-triggered CAN check aborted: robot must be disabled");
          SmartDashboard.putString("CAN Checks/lastRunError", "Controller probe aborted: robot must be disabled");
          return;
        }
        System.out.println("[RobotContainer] Controller-triggered CAN check starting.");
        CANChecker.runChecks();
      }));
      m_operatorController.a().whileTrue(new IntakeCMD(miscSubsystem, DriveConstants.intakeMotorSpeed)); // Takes in fuel
      m_operatorController.a().whileTrue(new PosIntakeBumperCMD(posIntakeSubsystem, DriveConstants.posIntakeMotorSpeed * 1.0)); // Moves posIntake into position when using intake
      //m_operatorController.leftTrigger(0.5).toggleOnTrue(new ShooterCMD(miscSubsystem, Robot.limelight_range_proportional())); // Sets the speed of shooter based on distance of apriltag
      m_operatorController.leftTrigger(0.5).whileTrue(new ShooterCMD(miscSubsystem, DriveConstants.softShooterTargetRPM, m_operatorController));
      m_operatorController.leftBumper().whileTrue(new ShooterCMD(miscSubsystem, DriveConstants.hardShooterTargetRPM, m_operatorController)); // Use if limelight starts to fail
      // When operator right trigger is held, run both launcher and indexer together.
      // Previously these were two separate commands that both required the same
      // `miscSubsystem`, causing a conflict where only one would run. Use a
      // single RunCommand so both motors are commanded simultaneously.
      m_operatorController.rightTrigger(0.5).whileTrue(new LauncherCMD(launcherSubsystem, DriveConstants.launcherMotorSpeed));
      m_operatorController.rightTrigger(0.5).whileTrue(new PosIntakeShakeCMD(posIntakeSubsystem, DriveConstants.posIntakeZeroMotorSpeed)); // Jiggles posIntake when using launcher
      m_operatorController.x().whileTrue(new PosIntakeBumperCMD(posIntakeSubsystem, DriveConstants.posIntakeMotorSpeed)); // Move posIntake to bumper
      m_operatorController.y().whileTrue(new PosIntakeZeroCMD(posIntakeSubsystem, DriveConstants.posIntakeZeroMotorSpeed)); // Move posIntake to zero
    } catch (Exception e) {
      // Defensive: if controller library changes or no controller connected, log and continue.
      System.out.println("[RobotContainer] Failed to bind controller CAN check: " + e);
    }
    */

    // Also bind raw joystick button 1 as a fallback for non-Xbox controllers
    new JoystickButton(m_operatorJoystick, 1).whileTrue(new ShooterCMD(shooterSubsystem, DriveConstants.softShooterTargetRPM)); // Soft shooter
    new JoystickButton(m_operatorJoystick, 2).whileTrue(new ShooterCMD(shooterSubsystem, DriveConstants.hardShooterTargetRPM)); // Hard shooter
    new JoystickButton(m_operatorJoystick, 3).whileTrue(new IntakeCMD(intakeSubsystem, DriveConstants.intakeMotorSpeed)); // Intake
    new JoystickButton(m_operatorJoystick, 3).whileTrue(new PosIntakeBumperCMD(posIntakeSubsystem, DriveConstants.posIntakeMotorSpeed)); // posIntake to bumper when intaking
    new JoystickButton(m_operatorJoystick, 4).whileTrue(new LauncherCMD(launcherSubsystem, DriveConstants.launcherMotorSpeed)); // Fuel to shooter
    new JoystickButton(m_operatorJoystick, 4).whileTrue(new PosIntakeShakeCMD(posIntakeSubsystem, DriveConstants.posIntakeZeroMotorSpeed)); // Jiggles posIntake when using launcher
    //new JoystickButton(m_operatorJoystick, 4).whileTrue(new IntakeCMD(miscSubsystem, DriveConstants.intakeMotorSpeed)); // Intake while agitating
    new JoystickButton(m_operatorJoystick, 5).whileTrue(new PosIntakeBumperCMD(posIntakeSubsystem, DriveConstants.posIntakeMotorSpeed)); // posIntake to bumper
    new JoystickButton(m_operatorJoystick, 6).whileTrue(new PosIntakeZeroCMD(posIntakeSubsystem, DriveConstants.posIntakeZeroMotorSpeed)); // posIntake to zero    
    new JoystickButton(m_operatorJoystick, 7).whileTrue(new ShooterCMD(shooterSubsystem, 1000));
    new JoystickButton(m_operatorJoystick, 8).toggleOnTrue(new InstantCommand(() -> shooterSubsystem.setSpeed(ShooterSetSpeed.SlowSpeed))).toggleOnFalse(new InstantCommand(() -> shooterSubsystem.stopShooterSpeed()));
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