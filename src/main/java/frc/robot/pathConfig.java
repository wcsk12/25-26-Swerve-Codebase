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
    double massKG = 50.03; //((Number) json.get("robotMass")).doubleValue();
    double MOI = 6.883; //((Number) json.get("robotMOI")).doubleValue();
    double wheelRadius = 0.051; //((Number) json.get("driveWheelRadius")).doubleValue();
    double gearing = 5.143; //((Number) json.get("driveGearing")).doubleValue();
    double maxDriveSpeed = 4.8; //((Number) json.get("maxDriveSpeed")).doubleValue();
    double wheelCOF = 1.4; //((Number) json.get("wheelCOF")).doubleValue();
    String driveMotor = "NEO"; //(String) json.get("driveMotorType");
    double driveCurrentLimit = 60.0; //((Number) json.get("driveCurrentLimit")).doubleValue();

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
              0.381,//((Number) json.get("flModuleX")).doubleValue(),
              0.279),//((Number) json.get("flModuleY")).doubleValue()),
            new Translation2d(
              0.381,//((Number) json.get("frModuleX")).doubleValue(),
               -0.279),//((Number) json.get("frModuleY")).doubleValue()),
            new Translation2d(
               -0.381,//((Number) json.get("blModuleX")).doubleValue(),
              0.279),//((Number) json.get("blModuleY")).doubleValue()),
            new Translation2d(
               -0.381,//((Number) json.get("brModuleX")).doubleValue(),
               -0.279)//((Number) json.get("brModuleY")).doubleValue())
          };

      return new RobotConfig(massKG, MOI, moduleConfig, moduleOffsets);
    } else {
      double trackwidth = 0.546;//((Number) json.get("robotTrackwidth")).doubleValue();

      return new RobotConfig(massKG, MOI, moduleConfig, trackwidth);
    }
  }
}
