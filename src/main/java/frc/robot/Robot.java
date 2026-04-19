package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
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

  public static double rot = 0.0;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    // Reuse DriveSubsystem from RobotContainer
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
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    //drive(false);
    m_swerve.periodic();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    //m_swerve.setHeading(m_swerve.getHeading());
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }
  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Drive normally; OI bindings in RobotContainer will schedule alignment command when A is pressed
    if (m_driverController.getAButton()) {
      //if (limelight_id() == 18 || limelight_id() == 26 || limelight_id() == 24){
        drive(true);
      //}
    }
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

  public static double limelight_distance_proportional()
  {
    double dist = 0;

    Pose3d targetCam = LimelightHelpers.getTargetPose3d_CameraSpace("limelight");
      if (targetCam != null) {
        Translation3d tr = targetCam.getTranslation();
        dist = Math.sqrt(tr.getX() * tr.getX() + tr.getY() * tr.getY() + tr.getZ() * tr.getZ());
      }
    return dist;
  }

  public static int limelight_id() {
    RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("limelight");
    int aprilTagId = 0;
    for (RawFiducial fiducial : fiducials) {
      aprilTagId = fiducial.id;
    }
    System.out.println("id: " + aprilTagId);
    return aprilTagId;
  }

  private void drive(boolean fieldRelative){
    var xSpeed = -MathUtil.applyDeadband(m_driverController.getRawAxis(1), OIConstants.kDriveDeadband);
      /*-m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_driverController.getLeftY(), 0.02))
        * DriveConstants.kMaxSpeedMetersPerSecond;*/

    var ySpeed = MathUtil.applyDeadband(m_driverController.getRawAxis(0), OIConstants.kDriveDeadband);
      /*-m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_driverController.getLeftX(), 0.02))
        * DriveConstants.kMaxSpeedMetersPerSecond;*/

    // Teleop alignment moved to command: Robot.teleopPeriodic schedules AutoAlignCommand when A is down.

    m_swerve.drive(xSpeed * RobotContainer.speedMode, ySpeed * RobotContainer.speedMode, AutoAlignCommand.rCmd, fieldRelative, getPeriod()); //rot was set to negative, but was changed to positive
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
