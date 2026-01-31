package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final XboxController m_driverController = 
    new XboxController(OIConstants.kDriverControllerPort);

  private DriveSubsystem m_swerve;
  private final RobotContainer m_robotContainer;

  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    // Use the DriveSubsystem instance constructed by RobotContainer to avoid
    // creating duplicate hardware objects (which can cause duplicate CAN IDs).
    m_swerve = m_robotContainer.getDriveSubsystem();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    drive(false);
    m_swerve.periodic();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    drive(true);
  }

  public static double limelight_aim_proportional()
  {
    double kP = .035;

    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

    targetingAngularVelocity *= -1.0;
    System.out.println(targetingAngularVelocity);
    return targetingAngularVelocity;
  }

  public static double limelight_range_proportional()
  {
    double kP = .035;
    
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
    targetingForwardSpeed *= 0.1;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }

  private void drive(boolean fieldRelative){
    var xSpeed = 
      -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_driverController.getLeftY(), 0.02))
        * DriveConstants.kMaxSpeedMetersPerSecond;

    var ySpeed = 
      -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_driverController.getLeftX(), 0.02))
        * DriveConstants.kMaxSpeedMetersPerSecond;

    var rot = 
      -m_rotLimiter.calculate(MathUtil.applyDeadband(m_driverController.getRightY(), 0.02))
        * DriveConstants.kMaxSpeedMetersPerSecond;

    if (m_driverController.getAButton()){
      final var rot_limelight = limelight_aim_proportional();
      rot = rot_limelight;

      final var foward_limelight = limelight_range_proportional();
      xSpeed = foward_limelight;

      fieldRelative = false;
    }

    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative, getPeriod());
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
