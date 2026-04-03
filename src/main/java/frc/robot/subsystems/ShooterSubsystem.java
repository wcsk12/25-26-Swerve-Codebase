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
            TrenchSpeed(3030),
            FarSpeed(3450);
    
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
    // Configure shooterMotor1 to follow shooterMotor2 (inverted) so closed-loop control on motor2
    // drives the full shooter pair consistently. We keep motor2 as the master with the encoder.
    SparkMaxConfig followerConfig = new SparkMaxConfig();
    followerConfig.apply(Configs.MAXSwerveModule.shooterConfig);
    followerConfig.follow(shooterMotor2, true);
    // PID Config - set conservative defaults near the first P candidate used by the L3 tuner
    // The tuner computes ff = openLoopOutput / avgRPM and startP = ff * 3.0. The first
    // candidate tested is startP * 0.25. Using the expected soft target RPM and the
    // tuner's open-loop output (0.5) yields approximate defaults below.
    this.kP = 0.000134; // conservative default P (first candidate approx)
    this.kI = 0.00; // Integral DON'T Change \
    this.kD = 0.00; // Differential \ "Dampening"
    this.kMinOutput = 0.00;
    this.kMaxOutput = 1.00;
    // Set velocity FF approximate default derived from open-loop expectation:
    // ff ~= openLoopOutput / expectedRPM -> 0.5 / 2800 ~= 0.00017857
    config.closedLoop
        .p(kP)
        .i(kI)
        .d(kD)
        .velocityFF(0.00017857)
        .outputRange(kMinOutput, kMaxOutput);

        shooterMotor2.configure(config,
      com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterMotor1.configure(followerConfig,
      com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        shooterRPMEncoder.setPosition(0);
    // On startup, attempt to load persisted gains from the roboRIO filesystem
    try {
      java.io.File f = new java.io.File("/home/lvuser/shooter_gains.properties");
      if (f.exists()) {
        java.util.Properties p = new java.util.Properties();
        try (java.io.FileInputStream fis = new java.io.FileInputStream(f)) {
          p.load(fis);
        }
        double pGain = Double.parseDouble(p.getProperty("p", "0"));
        double iGain = Double.parseDouble(p.getProperty("i", "0"));
        double dGain = Double.parseDouble(p.getProperty("d", "0"));
        double ff = Double.parseDouble(p.getProperty("ff", "0"));
        System.out.println(String.format("[ShooterSubsystem] Loaded persisted gains p=%.8f i=%.8f d=%.8f ff=%.8f", pGain, iGain, dGain, ff));
        applyClosedLoopGains(pGain, iGain, dGain, ff);
      }
    } catch (Exception e) {
      System.out.println("[ShooterSubsystem] Failed to load persisted gains: " + e);
    }
  }

  /**
   * Apply closed-loop gains at runtime. This updates the SparkMax config and writes
   * it to both motors so the closed-loop controller uses the new gains.
   */
  public void applyClosedLoopGains(double p, double i, double d, double velocityFF) {
    config.closedLoop
        .p(p)
        .i(i)
        .d(d)
        .velocityFF(velocityFF)
        .outputRange(kMinOutput, kMaxOutput);

    shooterMotor2.configure(config,
      com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
    SparkMaxConfig followerConfig = new SparkMaxConfig();
    followerConfig.apply(Configs.MAXSwerveModule.shooterConfig);
    followerConfig.follow(shooterMotor2, true);
    shooterMotor1.configure(followerConfig,
      com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
  }

  /** Set a closed-loop RPM target using the SparkMax closed-loop controller. */
  public void setClosedLoopTargetRPM(double rpm) {
    if (m_ShooterPID != null) {
      m_ShooterPID.setReference(rpm, com.revrobotics.spark.SparkBase.ControlType.kVelocity);
    }
  }

  /** Convenience: return current measured shooter RPM from encoder. */
  public double getMeasuredRPM() {
    return shooterRPMEncoder.getVelocity();
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
