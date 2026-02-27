package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OtherMotors;

/**
 * Placeholder ShooterSubsystem left in the tree for compatibility during the revert.
 * This class intentionally does not own hardware. The project has been reverted to use
 * OtherMotorsSubsystem for motor ownership; this file is a harmless placeholder so
 * leftover references (if any) won't break the build.
 */
public class ShooterSubsystem extends SubsystemBase {
  public ShooterSubsystem() {}

    public static SparkMax ShooterMotor = new SparkMax(OtherMotors.ShooterMotorId, MotorType.kBrushless);

    public void setShooterSpeed(double speed){
    ShooterMotor.set(speed);
    }
    @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
