package frc.robot;
//Pathplanner Imports\\
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
//Math Imports\\
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
//IO Imports\\
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
//Simple Imports\\
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

public class pathConfig {
    /**
   * Load the robot config from the shared settings file created by the GUI
   * @return RobotConfig matching the robot settings in the GUI
   * @throws IOException if an I/O error occurs
   * @throws ParseException if a JSON parsing error occurs
   */
  public static RobotConfig fromGUISettings() {//throws IOException, ParseException {
    //File file = new File("C:\\\\Users\\\\hatchetrobotics\\\\Desktop\\\\2025_SwerveRobot-main\\\\2025_SwerveRobot-main\\\\src\\\\main\\\\deploy\\\\pathplanner\\\\settings.json");
    //BufferedReader br =
    //    new BufferedReader(
    //        new FileReader(file)); // File(Filesystem.getDeployDirectory(), "C:////Users////hatchetrobotics////Desktop////2025_SwerveRobot-main////2025_SwerveRobot-main////src////main////deploy////pathplanner////settings.json"))

    //StringBuilder fileContentBuilder = new StringBuilder();
    //String line;
    //while ((line = br.readLine()) != null) {
    //  fileContentBuilder.append(line);
    //}
    //br.close();

    //String fileContent = fileContentBuilder.toString();
    //JSONObject json = (JSONObject) new JSONParser().parse(fileContent);

    boolean isHolonomic = true;
    double massKG = 61.2; // Robot mass in kg
    double MOI = 6.883; // Moment of inertia
    double wheelRadius = 0.0381; // 3" diameter = 0.0762m, radius = 0.0381m
    double gearing = 4.714; // (45*22)/(14*15) for 14T pinion
    double maxDriveSpeed = 5.45; // Max drive speed m/s
    double wheelCOF = 1.2; // Wheel coefficient of friction
    String driveMotor = "NEO";
    double driveCurrentLimit = 60.0;

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
      // Module positions: 22.5" trackwidth = 0.5715m, half = 0.286m from center
      // Standard WPILib convention (Y-inversion handled in driveRobotRelative)
      Translation2d[] moduleOffsets =
          new Translation2d[] {
            new Translation2d(0.286, 0.286),   // FL: +x, +y
            new Translation2d(0.286, -0.286),  // FR: +x, -y
            new Translation2d(-0.286, 0.286),  // BL: -x, +y
            new Translation2d(-0.286, -0.286)  // BR: -x, -y
          };

      return new RobotConfig(massKG, MOI, moduleConfig, moduleOffsets);
    } else {
      double trackwidth = 0.5715; // 22.5" in meters

      return new RobotConfig(massKG, MOI, moduleConfig, trackwidth);
    }
  }
}