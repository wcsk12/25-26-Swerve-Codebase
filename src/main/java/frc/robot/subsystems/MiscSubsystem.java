// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Configs;

public class MiscSubsystem extends SubsystemBase {
  private final SparkMax intakeMotor;
  private final SparkMax shooterMotor1;
  private final SparkMax shooterMotor2;

  /** Creates a new ExampleSubsystem. */
  public MiscSubsystem() {
  intakeMotor = new SparkMax(DriveConstants.intakeId, MotorType.kBrushless);
  shooterMotor1 = new SparkMax(DriveConstants.shooter1Id, MotorType.kBrushless);
  shooterMotor2 = new SparkMax(DriveConstants.shooter2Id, MotorType.kBrushless);

  // Apply conservative current limits to reduce brownout risk during matches.
  intakeMotor.configure(Configs.MAXSwerveModule.generalConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
    com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);
  shooterMotor1.configure(Configs.MAXSwerveModule.shooterConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
    com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);
  shooterMotor2.configure(Configs.MAXSwerveModule.shooterConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
    com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);
  }

  public void setIntakeSpeed(double speed){
    intakeMotor.set(speed);
  }

  public void setShooterSpeed(double speed){
    shooterMotor1.set(-speed);
    shooterMotor2.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
