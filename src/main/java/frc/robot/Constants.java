package frc.robot;
import com.revrobotics.spark.SparkMax;

// Math Imports 
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
// Miscellaneous
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
  // Sets up simulation
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */ 
    REAL,
    /** Running a physics simulator. */
    SIM,
    /** Replaying from a log file. */
    REPLAY
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 6.0;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    //--- Motor Speeds of other motors ---\\
    //public static final double indexerMotorSpeed = 0.75; indexer uses launcherMotorSpeed
    public static final double intakeMotorSpeed = 1.0;
    public static final double launcherMotorSpeed = 0.75;
    public static final double softShooterTargetRPM = 2800;
    public static final double hardShooterTargetRPM = 3800;
    public static final double shooterRPMDeadzone = 150;
    public static final double posIntakeMotorSpeed = -0.30;
    public static final double posIntakeZeroMotorSpeed = 0.40;

    // Current Regulation Constants
    public static final boolean regulateShooter = false;
    public static final boolean regulateIntake = true;
    public static final boolean regulatePosIntake = true;
    public static final boolean regulateIndexer = true;
    public static final boolean regulateLauncher = true;
    // Default SparkMax current limit is 80A
    // Recommended current limit for NEO motors is 40-60A
    public static final int shooterMaxCurrent = 60;
    public static final int intakeMaxCurrent = 40;
    public static final int posIntakeMaxCurrent = 40;
    public static final int indexerMaxCurrent = 40;
    public static final int launcherMaxCurrent = 40;

    // Elevator Constants
    public static final double kStartPose = -44.4;
    public static double h = 0;// Elevator Encoder Value
    // Distance Sensor Constants
    public static final double endEffectorDist = 0.08;

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(22.5); // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(22.5); // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = Math.PI;
    public static final double kFrontRightChassisAngularOffset = -Math.PI / 2;
    public static final double kBackLeftChassisAngularOffset = Math.PI / 2;
    public static final double kBackRightChassisAngularOffset = 0;

    // SWERVE SPARK MAX CAN IDs  CANNOT USE ID 12 DUE TO PIGEON BEING SET TO ID 12
    // Driving Motors
    public static final int kFrontLeftDrivingCanId = 2;
    public static final int kFrontRightDrivingCanId = 4;
    public static final int kRearLeftDrivingCanId = 6;
    public static final int kRearRightDrivingCanId = 8;
    // Turning Motors 
    public static final int kFrontLeftTurningCanId = 1;
    public static final int kFrontRightTurningCanId = 3;
    public static final int kRearLeftTurningCanId = 5;
    public static final int kRearRightTurningCanId = 7;

    public static int modeValue = 0;

    //--- OTHER SPARK MAX CAN IDs (These will likely be changed each year) ---\\
    public static final int intakeId = 14;
    public static final int posIntakeId = 15;
    public static final int shooter1Id = 16;
    public static final int shooter2Id = 17;
    public static final int launcherId = 18;
    public static final int indexerId = 19; // removed launcher2 and indexer2 because there are only one motor per each

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }
//------OI-----//
  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
    public static final double kEndEffectorDeadband = 0.5;
  }
//--------AUTO-------//
  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
//------NEO-------//
  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}