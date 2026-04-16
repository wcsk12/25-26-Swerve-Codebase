package frc.robot;
// SparkMax Imports
import com.revrobotics.spark.config.SparkMaxConfig;
// Math Imports
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;

// Pathplanner Imports
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// Constant Imports
import frc.robot.Constants.ModuleConstants;
// Miscellaneous
import edu.wpi.first.wpilibj.Filesystem;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

public final class Configs {
    public static final class MAXSwerveModule {
      // Initializes variables
  public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
  public static final SparkMaxConfig turningConfig = new SparkMaxConfig();
  public static final SparkMaxConfig shooterConfig = new SparkMaxConfig();
  // Small motor configs used by shooter/launcher/indexer/intake subsystems
  public static final SparkMaxConfig launcherConfig = new SparkMaxConfig();
  public static final SparkMaxConfig indexerConfig = new SparkMaxConfig();
  public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();
  public static final SparkMaxConfig posIntakeConfig = new SparkMaxConfig();

        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            double turningFactor = 2 * Math.PI;
            double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);
            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor) // meters
                    .velocityConversionFactor(drivingFactor / 60.0); // meters per second
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.04, 0, 0)
                    .velocityFF(drivingVelocityFeedForward)
                    .outputRange(-1, 1);

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);
            turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(true)
                    .positionConversionFactor(turningFactor) // radians
                    .velocityConversionFactor(turningFactor / 60.0); // radians per second
            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(1, 0, 0)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);

          shooterConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50);
          // Conservative small-motor configs
          launcherConfig.apply(drivingConfig);
          launcherConfig.smartCurrentLimit(40);

          indexerConfig.apply(drivingConfig);
          indexerConfig.smartCurrentLimit(30);

          intakeConfig.apply(drivingConfig);
          intakeConfig.smartCurrentLimit(40);

          posIntakeConfig.apply(drivingConfig);
          posIntakeConfig.smartCurrentLimit(20);
        }
    }

    /**
   * Load the robot config from the shared settings file created by the GUI
   *
   * @return RobotConfig matching the robot settings in the GUI
   * @throws IOException if an I/O error occurs
   * @throws ParseException if a JSON parsing error occurs
   */
  public static RobotConfig fromGUISettings() throws IOException, ParseException {
    BufferedReader br =
        new BufferedReader(
            new FileReader(new File(Filesystem.getDeployDirectory(), "C:////Users////hatchetrobotics////Desktop////2025_SwerveRobot-main////2025_SwerveRobot-main////src////main////deploy////pathplanner////settings.json")));

    StringBuilder fileContentBuilder = new StringBuilder();
    String line;
    while ((line = br.readLine()) != null) {
      fileContentBuilder.append(line);
    }
    br.close();
    // Configuration of the robot
    String fileContent = fileContentBuilder.toString();
    JSONObject json = (JSONObject) new JSONParser().parse(fileContent);

    boolean isHolonomic = true;
    double massKG = ((Number) json.get("robotMass")).doubleValue();
    double MOI = ((Number) json.get("robotMOI")).doubleValue();
    double wheelRadius = ((Number) json.get("driveWheelRadius")).doubleValue();
    double gearing = ((Number) json.get("driveGearing")).doubleValue();
    double maxDriveSpeed = ((Number) json.get("maxDriveSpeed")).doubleValue();
    double wheelCOF = ((Number) json.get("wheelCOF")).doubleValue();
    String driveMotor = (String) json.get("driveMotorType");
    double driveCurrentLimit = ((Number) json.get("driveCurrentLimit")).doubleValue();

    int numMotors = isHolonomic ? 1 : 2;
    DCMotor gearbox =
        switch (driveMotor) {
          case "krakenX60" -> DCMotor.getKrakenX60(numMotors);
          case "krakenX60FOC" -> DCMotor.getKrakenX60Foc(numMotors);
          case "falcon500" -> DCMotor.getFalcon500(numMotors);
          case "falcon500FOC" -> DCMotor.getFalcon500Foc(numMotors);
          case "vortex" -> DCMotor.getNeoVortex(numMotors);
          case "NEO" -> DCMotor.getNEO(numMotors);
          case "CIM" -> DCMotor.getCIM(numMotors);
          case "miniCIM" -> DCMotor.getMiniCIM(numMotors);
          default -> throw new IllegalArgumentException("Invalid motor type: " + driveMotor);
        };
    gearbox = gearbox.withReduction(gearing);

    ModuleConfig moduleConfig =
        new ModuleConfig(
            wheelRadius, maxDriveSpeed, wheelCOF, gearbox, driveCurrentLimit, numMotors);

    if (isHolonomic) {
      Translation2d[] moduleOffsets =
          new Translation2d[] {
            new Translation2d(
                ((Number) json.get("flModuleX")).doubleValue(),
                ((Number) json.get("flModuleY")).doubleValue()),
            new Translation2d(
                ((Number) json.get("frModuleX")).doubleValue(),
                ((Number) json.get("frModuleY")).doubleValue()),
            new Translation2d(
                ((Number) json.get("blModuleX")).doubleValue(),
                ((Number) json.get("blModuleY")).doubleValue()),
            new Translation2d(
                ((Number) json.get("brModuleX")).doubleValue(),
                ((Number) json.get("brModuleY")).doubleValue())
          };

      return new RobotConfig(massKG, MOI, moduleConfig, moduleOffsets);
    } else {
      double trackwidth = ((Number) json.get("robotTrackwidth")).doubleValue();

      return new RobotConfig(massKG, MOI, moduleConfig, trackwidth);
    }
  }
}