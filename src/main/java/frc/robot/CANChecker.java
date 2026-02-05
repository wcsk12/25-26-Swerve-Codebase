package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.time.Instant;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import frc.robot.Constants.DriveConstants;

/**
 * Lightweight, opt-in CAN checker for SparkMax devices.
 *
 * Usage: call CANChecker.runChecks() before constructing subsystems that create
 * SparkMax objects. The checker will attempt to instantiate a temporary SparkMax
 * for each expected ID and then close it. Results are published to SmartDashboard
 * under keys like "CAN/2/present" and "CAN/2/error".
 */
public final class CANChecker {
  private CANChecker() {}

  private static boolean widgetsCreated = false;

  /**
   * Create the Shuffleboard widgets used by the CAN checker without probing hardware.
   * Safe to call at startup; this only creates UI entries and sets initial values.
   */
  public static void createWidgets() {
    ShuffleboardTab tab = Shuffleboard.getTab("CAN Checks");
    if (!widgetsCreated) {
      for (int id : expectedIds()) {
        String baseKey = "CAN/" + id + "/";
        tab.add(baseKey + "present", false).withWidget(BuiltInWidgets.kBooleanBox);
        tab.add(baseKey + "status", "idle").withWidget(BuiltInWidgets.kTextView);
        tab.add(baseKey + "error", "").withWidget(BuiltInWidgets.kTextView);
      }
      tab.add("CAN/pigeon/12/present", false).withWidget(BuiltInWidgets.kBooleanBox);
      tab.add("CAN/pigeon/12/status", "idle").withWidget(BuiltInWidgets.kTextView);
      tab.add("CAN/pigeon/12/yaw", 0.0).withWidget(BuiltInWidgets.kTextView);
      // Summary widgets
      tab.add("CAN Summary/okCount", 0).withWidget(BuiltInWidgets.kTextView).withPosition(8, 0).withSize(1,1);
      tab.add("CAN Summary/total", 0).withWidget(BuiltInWidgets.kTextView).withPosition(8, 1).withSize(1,1);
      tab.add("CAN Checks/lastRunError", "").withWidget(BuiltInWidgets.kTextView).withPosition(8, 2).withSize(2,1);
      widgetsCreated = true;
    }
  }

  // List expected SparkMax IDs (drive + turning + example motor). Update if needed.
  private static int[] expectedIds() {
    return new int[] {
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kRearRightTurningCanId,
      // add any other SparkMax IDs you expect (ExampleSubsystem uses 10 in code)
      10
    };
  }

  public static void runChecks() {
    // Ensure the Shuffleboard widgets exist (this is safe and doesn't probe hardware).
    createWidgets();

    // Mark the start time so we can see when a probe ran in Shuffleboard/console.
    String startTime = Instant.now().toString();
    SmartDashboard.putString("CAN Checks/lastRunTime", startTime);
    SmartDashboard.putString("CAN Checks/lastRunError", "");
    System.out.println("[CANChecker] Starting CAN probe at " + startTime);

    int total = 0;
    int ok = 0;

    for (int id : expectedIds()) {
      String baseKey = "CAN/" + id + "/";
      try {
        SmartDashboard.putString(baseKey + "status", "probing");
        SparkMax sm = new SparkMax(id, MotorType.kBrushless);
        // If we successfully constructed the object, consider it present.
        SmartDashboard.putBoolean(baseKey + "present", true);
        SmartDashboard.putString(baseKey + "status", "ok");
        System.out.println("[CANChecker] SparkMax " + id + " present");
        ok++;
        total++;
        // Try to close the instance to avoid reserving the ID in this process.
        try {
          sm.close();
        } catch (Exception e) {
          // If close() isn't supported or fails, note it but continue.
          SmartDashboard.putString(baseKey + "closeError", e.toString());
        }
      } catch (Exception e) {
        SmartDashboard.putBoolean(baseKey + "present", false);
        SmartDashboard.putString(baseKey + "error", e.toString());
        SmartDashboard.putString(baseKey + "status", "error");
        SmartDashboard.putString("CAN Checks/lastRunError", "Error with ID " + id + ": " + e.toString());
        System.out.println("[CANChecker] SparkMax " + id + " error: " + e.toString());
        total++;
      }
    }

    // Check CTRE Pigeon2 (gyro) presence. This uses the same CAN bus but a different API.
    int pigeonId = 12; // matches where DriveSubsystem constructs Pigeon2
    String pigKey = "CAN/pigeon/" + pigeonId + "/";
    try {
      SmartDashboard.putString(pigKey + "status", "probing");
      Pigeon2 pig = new Pigeon2(pigeonId);
      try {
        // Attempt to read a value; if it throws or times out we'll catch it.
        double yaw = pig.getYaw().getValueAsDouble();
        SmartDashboard.putBoolean(pigKey + "present", true);
        SmartDashboard.putNumber(pigKey + "yaw", yaw);
        SmartDashboard.putString(pigKey + "status", "ok");
        ok++;
        total++;
      } finally {
        try {
          pig.close();
        } catch (Exception ignored) {
        }
      }
    } catch (Exception e) {
      SmartDashboard.putBoolean(pigKey + "present", false);
      SmartDashboard.putString(pigKey + "error", e.toString());
      SmartDashboard.putString(pigKey + "status", "error");
      SmartDashboard.putString("CAN Checks/lastRunError", "Pigeon error: " + e.toString());
      System.out.println("[CANChecker] Pigeon error: " + e.toString());
      total++;
    }

    // Publish summary
    SmartDashboard.putNumber("CAN Summary/okCount", ok);
    SmartDashboard.putNumber("CAN Summary/total", total);
    String endTime = Instant.now().toString();
    SmartDashboard.putString("CAN Checks/lastRunTime", endTime);
    System.out.println("[CANChecker] CAN probe finished at " + endTime + ": " + ok + "/" + total + " OK");
  }
}
