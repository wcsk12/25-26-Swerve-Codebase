// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Configs;
import com.revrobotics.AbsoluteEncoder;

public class PosIntakeSubsystem extends SubsystemBase {
  private SparkMax posIntakeMotor;
  private AbsoluteEncoder posIntakeEncoder;

  /** Creates a new PosIntakeSubsystem. */
  public PosIntakeSubsystem() {
    posIntakeMotor = new SparkMax(DriveConstants.posIntakeId, MotorType.kBrushless);
    posIntakeEncoder = posIntakeMotor.getAbsoluteEncoder(); 

    // Apply conservative current limit preset for the positioner
    posIntakeMotor.configure(Configs.MAXSwerveModule.posIntakeConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
        com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);
  }

  public void setPosIntakeSpeed(double speed) {
    posIntakeMotor.set(speed); //negative goes up, positive goes down
  }

  public double getPosIntakePosition() {
    double pos = posIntakeEncoder.getPosition();
    // Defensive: if the encoder is not returning a finite value, return NaN so callers
    // can differentiate a bad reading from a legitimate zero/wrap value.
    if (!Double.isFinite(pos)) {
      return Double.NaN;
    }
    return pos;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
