// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.DriveConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax intakeMotor;
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeMotor = new SparkMax(DriveConstants.intakeId, MotorType.kBrushless);

    // Apply conservative current limits to reduce brownout risk during matches.
    intakeMotor.configure(Configs.MAXSwerveModule.intakeConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
      com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);
  }

  public void setIntakeSpeed(double speed){
    intakeMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
