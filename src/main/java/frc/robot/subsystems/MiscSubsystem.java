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
  private final SparkMax shooterMotor1;
  private final SparkMax shooterMotor2;
  private final SparkMax launcherMotor;
  private final SparkMax indexerMotor;

  /** Creates a new ExampleSubsystem. */
  public MiscSubsystem() {
    intakeMotor = new SparkMax(DriveConstants.intakeId, MotorType.kBrushless);
    shooterMotor1 = new SparkMax(DriveConstants.shooter1Id, MotorType.kBrushless);
    shooterMotor2 = new SparkMax(DriveConstants.shooter2Id, MotorType.kBrushless);
    launcherMotor = new SparkMax(DriveConstants.launcherId, MotorType.kBrushless);
    indexerMotor = new SparkMax(DriveConstants.indexerId, MotorType.kBrushless);
  }

  public void setIntakeSpeed(double speed){
    intakeMotor.set(speed);
  }

  public void setShooterSpeed(double speed){
    shooterMotor1.set(speed);
    shooterMotor2.set(-speed);
  }

  public void setLauncherSpeed(double speed){
    // Invert launcher direction here so callers can pass a positive logical speed.
    launcherMotor.set(-speed);
  }

  public void setIndexerSpeed(double speed){
    indexerMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
