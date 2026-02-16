// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class MiscSubsystem extends SubsystemBase {
  private final SparkMax intakeMotor;
  private final SparkMax shooterMotor;
  private final SparkMax releaseMotor;

  /** Creates a new ExampleSubsystem. */
  public MiscSubsystem() {
    intakeMotor = new SparkMax(DriveConstants.intakeId, MotorType.kBrushless);
    shooterMotor = new SparkMax(DriveConstants.shooterId, MotorType.kBrushless);
    releaseMotor = new SparkMax(DriveConstants.releaseId, MotorType.kBrushless);
  }

  public void setIntakeSpeed(double speed){
    intakeMotor.set(speed);
  }

  public void setShooterSpeed(double speed){
    shooterMotor.set(speed);
  }

  public void setReleaseSpeed(double speed){
    releaseMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
