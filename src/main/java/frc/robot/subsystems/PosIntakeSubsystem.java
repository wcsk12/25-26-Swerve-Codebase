// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import com.revrobotics.AbsoluteEncoder;

public class PosIntakeSubsystem extends SubsystemBase {
  private SparkMax posIntakeMotor;
  private AbsoluteEncoder posIntakeEncoder;

  /** Creates a new PosIntakeSubsystem. */
  public PosIntakeSubsystem() {
    posIntakeMotor = new SparkMax(DriveConstants.posIntakeId, MotorType.kBrushless);
    posIntakeEncoder = posIntakeMotor.getAbsoluteEncoder(); 
  }

  public void setPosIntakeSpeed(double speed) {
    posIntakeMotor.set(speed); //negative goes up, positive goes down
  }

  public double getPosIntakePosition() {
    return posIntakeEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
