// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OtherMotors;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {}
  public static SparkMax ClimberMotor = new SparkMax(OtherMotors.ClimberMotorID, MotorType.kBrushless);

  public void SetClimberSpeed(double speed){
    ClimberMotor.set(speed);
    }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
