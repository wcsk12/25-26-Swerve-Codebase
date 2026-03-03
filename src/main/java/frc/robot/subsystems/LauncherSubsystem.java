// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class LauncherSubsystem extends SubsystemBase {
  private final SparkMax launcherMotor;
  private final SparkMax indexerMotor;
  /** Creates a new LauncherSubsystem. */
  public LauncherSubsystem() {
    launcherMotor = new SparkMax(DriveConstants.launcherId, MotorType.kBrushless);
    indexerMotor = new SparkMax(DriveConstants.indexerId, MotorType.kBrushless);
  }

  public void setLauncherSpeed(double speed){
    // Invert launcher direction here so callers can pass a positive logical speed.
    launcherMotor.set(-speed);
    indexerMotor.set(-speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
