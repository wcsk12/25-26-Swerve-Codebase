// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OtherMotors;

public class OtherMotorsSubsystem extends SubsystemBase {

  /** Creates a new ExampleSubsystem. */
  
    public static SparkMax sparkMaxMotor = new SparkMax(OtherMotors.IntakeMotorId, MotorType.kBrushless);
  //Intake Speed
  public static void setIntakeSpeed(double speed){
    sparkMaxMotor.set(speed);
  }
  //Shooter Speed -Not Yet Set Up.
  public static void setShooterSpeed(double speed){
    sparkMaxMotor.set(speed);
  }
  //Create a new OtherMotorsSubsystem
  public OtherMotorsSubsystem() {}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
