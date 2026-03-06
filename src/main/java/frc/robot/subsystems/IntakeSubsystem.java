package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OtherMotors;

/** Placeholder intake subsystem kept during revert; no hardware owned here. */
public class IntakeSubsystem extends SubsystemBase {
  public IntakeSubsystem() {}
   public static SparkMax IntakeMotor = new SparkMax(OtherMotors.IntakeMotorId, MotorType.kBrushless);

   public void setIntakeSpeed(double speed){
    IntakeMotor.set(speed);
  }

   @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
