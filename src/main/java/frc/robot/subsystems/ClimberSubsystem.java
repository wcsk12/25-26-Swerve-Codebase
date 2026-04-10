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

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OtherMotors;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  public static SparkMax ClimberMotor = new SparkMax(OtherMotors.ClimberMotorID, MotorType.kBrushless);
  private RelativeEncoder climberRelative;
  private SparkClosedLoopController m_ClimberPID;
  private SparkMaxConfig climberMotorConfig;
  private final Servo climberServo = new Servo(0); // Assuming the servo is connected to PWM port 0

    public void SetClimberSpeed(double speed) { // This is used to set motor speed.
      ClimberMotor.set(speed);
    }

    public void SetServoPosition(double position) { // This is used to set the servo position.
      climberServo.set(position);
    }


    public void stopClimber(double speed) {
      ClimberMotor.stopMotor();
    }

    @Override
    public void periodic(){

    }
  }
