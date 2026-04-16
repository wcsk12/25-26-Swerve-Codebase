// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Configs;

public class ShooterSubsystem extends SubsystemBase {
  private final SparkMax shooterMotor1;
  private final SparkMax shooterMotor2;
  private final RelativeEncoder shooterRPMEncoder;

  // Software PID constants
  private static final double kNEOFreeSpeedRPM = 5676.0;
  private static final double kP = 0.0003;

  // Current closed-loop target (0 = off)
  private double m_targetRPM = 0;

  public ShooterSubsystem() {
    shooterMotor1 = new SparkMax(DriveConstants.shooter1Id, MotorType.kBrushless);
    shooterMotor2 = new SparkMax(DriveConstants.shooter2Id, MotorType.kBrushless);
    shooterRPMEncoder = shooterMotor2.getEncoder();

    // Apply current limit (50A) and brake mode to both motors
    shooterMotor2.configure(Configs.MAXSwerveModule.shooterConfig,
        com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Motor1 follows motor2 (inverted — motors face opposite directions)
    SparkMaxConfig followerConfig = new SparkMaxConfig();
    followerConfig.apply(Configs.MAXSwerveModule.shooterConfig);
    followerConfig.follow(shooterMotor2, true);
    shooterMotor1.configure(followerConfig,
        com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    shooterRPMEncoder.setPosition(0);
  }

  // ---- Software closed-loop RPM control ----

  /**
   * Set a closed-loop RPM target. Pass 0 to stop.
   * Works for all shooter speeds (soft=2800, mid=3100, hard=3800).
   */
  public void setClosedLoopTargetRPM(double rpm) {
    m_targetRPM = rpm;
    if (rpm <= 0) {
      shooterMotor2.set(0);
    }
  }

  /** Return current measured shooter RPM from encoder. */
  public double getMeasuredRPM() {
    return shooterRPMEncoder.getVelocity();
  }

  public double getShooterRPM() {
    return shooterRPMEncoder.getVelocity();
  }

  public double getShooterPosition() {
    return shooterRPMEncoder.getPosition();
  }

  // ---- Open-loop legacy methods ----

  public void setShooterSpeed(double speed) {
    m_targetRPM = 0; // disable software PID when using open-loop
    shooterMotor1.set(-speed);
    shooterMotor2.set(speed);
  }

  public void stopShooterSpeed() {
    m_targetRPM = 0;
    shooterMotor1.stopMotor();
    shooterMotor2.stopMotor();
  }

  @Override
  public void periodic() {
    double measuredRPM = shooterRPMEncoder.getVelocity();

    // Telemetry
    SmartDashboard.putNumber("Shooter Velocity Value", measuredRPM);
    SmartDashboard.putNumber("Shooter Target RPM", m_targetRPM);
    SmartDashboard.putNumber("Shooter Motor2 Output", shooterMotor2.getAppliedOutput());
    SmartDashboard.putNumber("Shooter Motor1 Output", shooterMotor1.getAppliedOutput());
    SmartDashboard.putNumber("Shooter Motor2 Current", shooterMotor2.getOutputCurrent());
    SmartDashboard.putNumber("Shooter Motor1 Current", shooterMotor1.getOutputCurrent());

    // Software FF+P closed-loop control
    if (m_targetRPM > 0) {
      double ff = m_targetRPM / kNEOFreeSpeedRPM;
      double error = m_targetRPM - Math.abs(measuredRPM);
      double output = ff + (kP * error);
      output = Math.max(0.0, Math.min(1.0, output));
      shooterMotor2.set(output);
      SmartDashboard.putNumber("Shooter SW Output", output);
    }
  }
}
