package frc.robot.subsystems;
// CTRE Imports
import com.ctre.phoenix6.hardware.Pigeon2;
// Pathplanner Imports
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
// Hal Imports
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
//Limelight Imports
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
// Math Imports
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
// DriverStation and SmartDashboard Imports
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableInstance;
// Subsystem Imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// Config and Constant Imports
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.pathConfig;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset,
      DriveConstants.kFrontLeftEncoderOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset,
      DriveConstants.kFrontRightEncoderOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset,
      DriveConstants.kBackLeftEncoderOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset,
      DriveConstants.kBackRightEncoderOffset);

  // The gyro sensor
  private final Pigeon2 m_Pigeon2 = new Pigeon2(12);
  // 2d Field in SmartDashboard
  private final Field2d m_field = new Field2d(); 

  // Odometry class for tracking robot pose -- Odometry means position
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(-m_Pigeon2.getYaw().getValueAsDouble()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });
  // Initializing kinematics
  SwerveDriveKinematics m_kinematics;
  // Sets up exception messages
  public static RobotConfig config;{
    try{
        config = pathConfig.fromGUISettings();
      } catch (Exception e) {
        e.printStackTrace();  
      }
    }

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    // Places the field in SmartDashboard
    SmartDashboard.putData("Field", m_field);
    // Calls zero heading
    zeroHeading();
    // Set up robot's kinematics
    m_kinematics = new SwerveDriveKinematics(
            new Translation2d(Units.inchesToMeters(12.5), Units.inchesToMeters(12.5)), // Front Left
            new Translation2d(Units.inchesToMeters(12.5), Units.inchesToMeters(-12.5)), // Front Right
            new Translation2d(Units.inchesToMeters(-12.5), Units.inchesToMeters(12.5)), // Back Left
            new Translation2d(Units.inchesToMeters(-12.5), Units.inchesToMeters(-12.5))); // Back Right

    // Change the camera pose relative to robot center (x forward, y left, z up, degrees) <-- Not sure this is in the right place.
    
    // -------------- PathPlanner Code -------------- \\
    // Configure AutoBuilder last
    AutoBuilder.configure(
      this::getPose, // Robot pose supplier
      this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
      new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
            new PIDConstants(0.2, 0.0, 0), // Translation PID constants
            new PIDConstants(0.2, 0.0, 0) // Rotation PID constants
      ),
      config, // The robot configuration
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
     },
      this // Reference to this subsystem to set requirements
    );
    
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    
    m_odometry.update(
        Rotation2d.fromDegrees(-m_Pigeon2.getYaw().getValueAsDouble()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
/* Limelight Code
    if (DriverStation.isAutonomous()){
      LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      if (limelightMeasurement.tagCount >= 2) {  // Only trust measurement if we see multiple tags
        m_PoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
        m_PoseEstimator.addVisionMeasurement(
          limelightMeasurement.pose,
          limelightMeasurement.timestampSeconds
      );

        m_field.setRobotPose(m_PoseEstimator.getEstimatedPosition());
    
      }
      }
    */
  

  // Publish common telemetry to SmartDashboard / Shuffleboard
  SmartDashboard.putNumber("Gyro Angle", getHeading());
  SmartDashboard.putNumber("Gyro Rate", getTurnRate());

  Pose2d pose = getPose();
  SmartDashboard.putNumber("Pose X (m)", pose.getX());
  SmartDashboard.putNumber("Pose Y (m)", pose.getY());
  SmartDashboard.putNumber("Pose Rot (deg)", pose.getRotation().getDegrees());

  // Module speeds (m/s) and angles (deg)
  var flState = m_frontLeft.getState();
  var frState = m_frontRight.getState();
  var blState = m_rearLeft.getState();
  var brState = m_rearRight.getState();

  SmartDashboard.putNumber("FL Speed (m/s)", flState.speedMetersPerSecond);
  SmartDashboard.putNumber("FR Speed (m/s)", frState.speedMetersPerSecond);
  SmartDashboard.putNumber("BL Speed (m/s)", blState.speedMetersPerSecond);
  SmartDashboard.putNumber("BR Speed (m/s)", brState.speedMetersPerSecond);

  SmartDashboard.putNumber("FL Angle (deg)", Math.toDegrees(flState.angle.getRadians()));
  SmartDashboard.putNumber("FR Angle (deg)", Math.toDegrees(frState.angle.getRadians()));
  SmartDashboard.putNumber("BL Angle (deg)", Math.toDegrees(blState.angle.getRadians()));
  SmartDashboard.putNumber("BR Angle (deg)", Math.toDegrees(brState.angle.getRadians()));

  // Keep the original relative encoder velocity values for backwards compatibility
  SmartDashboard.putNumber("fRightDrive", m_frontRight.getRelativeEncoder());
  SmartDashboard.putNumber("fLeftDrive", m_frontLeft.getRelativeEncoder());
  SmartDashboard.putNumber("bRightDrive", m_rearRight.getRelativeEncoder());
  SmartDashboard.putNumber("bLeftDrive", m_rearLeft.getRelativeEncoder());

  // Limelight values (if a Limelight is present and named "limelight")
  var nt = NetworkTableInstance.getDefault().getTable("limelight");
  SmartDashboard.putNumber("limelight_tx", nt.getEntry("tx").getDouble(0.0));
  SmartDashboard.putNumber("limelight_ty", nt.getEntry("ty").getDouble(0.0));
  SmartDashboard.putNumber("limelight_ta", nt.getEntry("ta").getDouble(0.0));
  // Publish slow mode state and current divisor so Dashboard can update the banner
  frc.robot.Dashboard.setSlowModeBanner(isSlowModeActive(), getSpeedDivisor());
    
  }

  /** Returns true when a speed divisor greater than 1 is active (i.e., slow mode). */
  public static boolean isSlowModeActive() {
    return MaxDriveSpeed < DriveConstants.kMaxSpeedMetersPerSecond;
  }

  /** Returns the effective divisor applied to max speed (>=1). */
  public static double getSpeedDivisor() {
    return DriveConstants.kMaxSpeedMetersPerSecond / MaxDriveSpeed;
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
    //return m_PoseEstimator.getEstimatedPosition();
  }

  //public Pose2d getAutoPose(){
  //  return m_PoseEstimator.getEstimatedPosition();
  // }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetPose(Pose2d pose) {
    
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(-m_Pigeon2.getYaw().getValueAsDouble()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
      }
      
  public double GetPigeonDegrees(){
    Rotation2d currentRotation = m_Pigeon2.getRotation2d();
    // Wrap the degrees to [-180, 180)
    double wrappedDegrees = MathUtil.inputModulus(currentRotation.getDegrees(), -180, 180);
    SmartDashboard.putNumber("Pigeon Degrees", wrappedDegrees);
    return wrappedDegrees;
    
  }
/* 
    m_PoseEstimator.resetPosition(
        Rotation2d.fromDegrees(m_Pigeon2.getYaw().getValueAsDouble()), // This is what pathplanner uses for position
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        },
        getAutoPose());
  
  }
*/

  public ChassisSpeeds getRobotRelativeSpeeds(){
    ChassisSpeeds measured = DriveConstants.kDriveKinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());
    // Invert Y to compensate for swapped kinematics Y-signs
    return new ChassisSpeeds(measured.vxMetersPerSecond, -measured.vyMetersPerSecond, measured.omegaRadiansPerSecond);
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds){
    // Invert Y to compensate for swapped kinematics Y-signs
    ChassisSpeeds adjusted = new ChassisSpeeds(
        robotRelativeSpeeds.vxMetersPerSecond,
        -robotRelativeSpeeds.vyMetersPerSecond,
        robotRelativeSpeeds.omegaRadiansPerSecond);
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(adjusted, 0.02);

    SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }

  /**
   * Method to drive the robot using joystick info.
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
   
  /**
   * Set the scaling divisor for the X-axis drive speed.
   * Example: divisor=2 halves the maximum delivered X speed.
   */
  public static void setDriveSpeed(double divisor) { //set speed to half
    MaxDriveSpeed = DriveConstants.kMaxSpeedMetersPerSecond / divisor; //divide by divisor.
  }
  
  // Current maximum linear drive speed used by `drive(...)` for X axis scaling.
  // Default to the configured maximum in DriveConstants.
  private static double MaxDriveSpeed = DriveConstants.kMaxSpeedMetersPerSecond;

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    // Convert the commanded (normalized -1..1) speeds into physical units
    double xSpeedDelivered = xSpeed * MaxDriveSpeed;
    double ySpeedDelivered = ySpeed * MaxDriveSpeed;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    // DEBUG: Publish raw inputs to help diagnose teleop issues
    SmartDashboard.putNumber("Drive/xSpeed_raw", xSpeed);
    SmartDashboard.putNumber("Drive/ySpeed_raw", ySpeed);
    SmartDashboard.putNumber("Drive/rot_raw", rot);
    SmartDashboard.putNumber("Drive/xSpeed_delivered", xSpeedDelivered);
    SmartDashboard.putNumber("Drive/ySpeed_delivered", ySpeedDelivered);
    SmartDashboard.putNumber("Drive/rotDelivered", rotDelivered);
    SmartDashboard.putNumber("Drive/MaxDriveSpeed", MaxDriveSpeed);

    // Build chassis speeds using the delivered (scaled) values. Use field-relative
    // conversion when requested so joystick inputs are interpreted relative to the
    // field rather than the robot.
    ChassisSpeeds chassisSpeeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
            Rotation2d.fromDegrees(-m_Pigeon2.getYaw().getValueAsDouble()))
        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);

    ChassisSpeeds discretized = ChassisSpeeds.discretize(chassisSpeeds, periodSeconds);

    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics
        .toSwerveModuleStates(discretized);

    // Ensure wheel speeds are within the configured maximum
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    // DEBUG: Publish computed chassis speeds
    SmartDashboard.putNumber("Drive/chassisVx", discretized.vxMetersPerSecond);
    SmartDashboard.putNumber("Drive/chassisVy", discretized.vyMetersPerSecond);
    SmartDashboard.putNumber("Drive/chassisOmega", discretized.omegaRadiansPerSecond);

    // Apply via centralized canonical mapping (FL, FR, BL, BR)
    setModuleStates(swerveModuleStates);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /*public void changeModeValue(int value){
    DriveConstants.modeValue = value;
  }*/

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    
// -------------- SMARTDASHBOARD MODULE STATES --------------  \\
    // DEBUG: Show commanded states for each module
    //SmartDashboard.putNumber("Mod/FL_speed", desiredStates[0].speedMetersPerSecond);
    //SmartDashboard.putNumber("Mod/FL_angle", desiredStates[0].angle.getDegrees());
    //SmartDashboard.putNumber("Mod/FR_speed", desiredStates[1].speedMetersPerSecond);
    //SmartDashboard.putNumber("Mod/FR_angle", desiredStates[1].angle.getDegrees());
    //SmartDashboard.putNumber("Mod/BL_speed", desiredStates[2].speedMetersPerSecond);
    //SmartDashboard.putNumber("Mod/BL_angle", desiredStates[2].angle.getDegrees());
    //SmartDashboard.putNumber("Mod/BR_speed", desiredStates[3].speedMetersPerSecond);
    //SmartDashboard.putNumber("Mod/BR_angle", desiredStates[3].angle.getDegrees());

    // Apply module states in canonical WPILib/kinematics order:
    // Front Left, Front Right, Back Left, Back Right.
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_Pigeon2.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_Pigeon2.getYaw().getValueAsDouble()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_Pigeon2.getAngularVelocityZWorld().getValueAsDouble() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Expose the Field2d used for visualization so Dashboard can add the same instance.
   */
  public Field2d getField() {
    return m_field;
  }

}