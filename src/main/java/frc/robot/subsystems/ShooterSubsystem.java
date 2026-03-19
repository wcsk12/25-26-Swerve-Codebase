// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Configs;

public class ShooterSubsystem extends SubsystemBase {
  private final SparkMax shooterMotor1;
  private final SparkMax shooterMotor2;
  private final RelativeEncoder shooterRPMEncoder;
  private SparkClosedLoopController m_ShooterPID;

  private final double kP;  
  private final double kI; 
  private final double kD; 
  private final double kMinOutput;
  private final double kMaxOutput;

  SparkMaxConfig config = new SparkMaxConfig();

  public enum ShooterSetSpeed {
            ZeroSpeed(0),
            SlowSpeed(3000),
            FastSpeed(256);
    
            private final double value;
            
    
            ShooterSetSpeed(double value){
                this.value = value;
            }

    
            public double getValue() {
                return value;
            }
        }

  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {
    shooterMotor1 = new SparkMax(DriveConstants.shooter1Id, MotorType.kBrushless);
    shooterMotor2 = new SparkMax(DriveConstants.shooter2Id, MotorType.kBrushless);
    shooterRPMEncoder = shooterMotor2.getEncoder(); //shooterMotor1's encoder appears to be broken?

    m_ShooterPID = shooterMotor2.getClosedLoopController();

    // Apply conservative current limits to reduce brownout risk during matches.
    shooterMotor1.configure(Configs.MAXSwerveModule.shooterConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
      com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);
    shooterMotor2.configure(Configs.MAXSwerveModule.shooterConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
      com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);
    // PID Config
    this.kP = 0.10; // Proportianal \\ "Speed"
    this.kI = 0.00; // Integral DON'T Change \\\
    this.kD = 0.00; // Differential \\ "Dampening"
    this.kMinOutput = 0.00;
    this.kMaxOutput = 1.00;
    // Configure closed-loop values for SparkMax (if used)
    config.closedLoop
        .p(kP)
        .i(kI)
        .d(kD)
        .outputRange(kMinOutput, kMaxOutput);

        shooterMotor1.configure(config,
      com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterMotor2.configure(config,
      com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        shooterRPMEncoder.setPosition(0);
  }

  public void setShooterSpeed(double speed){
    shooterMotor1.set(-speed);
    shooterMotor2.set(speed);
  }

  public void stopShooterSpeed(){
    shooterMotor1.stopMotor();
    shooterMotor2.stopMotor();
  }

  public double getShooterPosition() {
    return shooterRPMEncoder.getPosition();
  }

  public double getShooterRPM() {
    return shooterRPMEncoder.getVelocity();
  }

  public void setSpeed(ShooterSetSpeed position){
            m_ShooterPID.setReference(position.getValue(), ControlType.kVelocity);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Velocity Value", shooterRPMEncoder.getVelocity());
  }
}
