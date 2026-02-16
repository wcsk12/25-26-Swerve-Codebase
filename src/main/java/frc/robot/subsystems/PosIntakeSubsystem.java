// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class PosIntakeSubsystem extends SubsystemBase {
  private SparkMax posIntakeMotor;
  private RelativeEncoder posIntakeRelative;
  private SparkClosedLoopController m_posIntakePID;
  private SparkMaxConfig posIntakeMotorConfig;

  public enum IntakePositions{
    zero(0),
    low(8);

    private final double value;

    IntakePositions(double value){
      this.value = value;
    }

    public double getValue(){
      return value;
    }
  }

  /** Creates a new PosIntakeSubsystem. */
  public PosIntakeSubsystem() {
    posIntakeMotor = new SparkMax(DriveConstants.posIntakeId, MotorType.kBrushless);
    posIntakeRelative = posIntakeMotor.getEncoder();
    m_posIntakePID = posIntakeMotor.getClosedLoopController();
    posIntakeMotorConfig = new SparkMaxConfig();

    // Change P(Elevator Speed) till it doesn't skip
            // p = Speed 
            // i = Correction
            // d = Dampening
    posIntakeMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.05)
        .i(0)
        .d(0)
        .outputRange(-1, 1);
    posIntakeMotorConfig
        .inverted(false)
        .smartCurrentLimit(30)
        .idleMode(IdleMode.kBrake);
    posIntakeMotor.configure(posIntakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    posIntakeRelative.setPosition(0);
  }

  public void setPosIntakeSpeed(double speed){
    posIntakeMotor.set(speed);
  }

  public void stopPosIntake(){
    posIntakeMotor.stopMotor();
  }

  public void setPosition(IntakePositions position){
            m_posIntakePID.setSetpoint(position.getValue(), ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Encoder Value", posIntakeRelative.getPosition());
  }
}
