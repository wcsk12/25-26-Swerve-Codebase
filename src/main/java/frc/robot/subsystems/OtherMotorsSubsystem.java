// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OtherMotors;

public class OtherMotorsSubsystem extends SubsystemBase {

   //Create a new OtherMotorsSubsystem
    public OtherMotorsSubsystem() {}
  
    public static SparkMax IntakeMotor = new SparkMax(OtherMotors.IntakeMotorId, MotorType.kBrushless);
    public static SparkMax ShooterMotor = new SparkMax(OtherMotors.ShooterMotorId, MotorType.kBrushless);
    public static SparkMax ReleaseMotor = new SparkMax(OtherMotors.ShooterMotorReleaseId, MotorType.kBrushless);
  //Intake Speed
  public void setIntakeSpeed(double speed){
    IntakeMotor.set(speed);
  }
  //Shooter Speed -Not Yet Set Up.
  public void setShooterSpeed(double speed){
    ShooterMotor.set(speed);
  }
   public void setReleaseSpeed(double speed){
    ReleaseMotor.set(speed);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
