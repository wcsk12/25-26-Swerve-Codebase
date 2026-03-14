// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// --------------------- Spark Imports --------------------- \\
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OtherMotors;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  public static SparkMax ClimberMotor = new SparkMax(OtherMotors.ClimberMotorID, MotorType.kBrushless);
  private RelativeEncoder climberRelative;
  private SparkClosedLoopController m_ClimberPID;
  private SparkMaxConfig climberMotorConfig;

  public enum ClimberPositions{
      zero(0), //Starting position.
      high(8); //Value will almost absolutely need changed.

      private final double value;

      ClimberPositions(double value){
        this.value = value;
      }

      public double getValue(){
        return value;
      }
    }

  public ClimberSubsystem() {
      climberRelative = ClimberMotor.getEncoder();
      m_ClimberPID = ClimberMotor.getClosedLoopController();
      climberMotorConfig = new SparkMaxConfig();

      climberMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .p(0.05)
          .i(0)
          .d(0)
          .outputRange(-1, 1);
      climberMotorConfig
          .inverted(false)
          .smartCurrentLimit(30)
          .idleMode(IdleMode.kBrake);
      ClimberMotor.configure(climberMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); //Look up replacement

      climberRelative.setPosition(0);
    }

    public void SetClimberSpeed(double speed) { // This is used to set motor speed.
      ClimberMotor.set(speed);
    }

    public void stopClimber(double speed) {
      ClimberMotor.stopMotor();
    }

    public void setPosition(ClimberPositions position){
      m_ClimberPID.setSetpoint(position.getValue(), ControlType.kPosition);
    }

    @Override
    public void periodic(){
      SmartDashboard.putNumber("Climber Encoder Value", climberRelative.getPosition());
    }
  }
