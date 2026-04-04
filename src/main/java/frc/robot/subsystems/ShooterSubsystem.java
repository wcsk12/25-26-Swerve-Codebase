package frc.robot.subsystems;

import java.util.ArrayList;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OtherMotors;
import frc.robot.Configs;
import frc.robot.Robot;

/**
 * Placeholder ShooterSubsystem left in the tree for compatibility during the revert.
 * This class intentionally does not own hardware. The project has been reverted to use
 * OtherMotorsSubsystem for motor ownership; this file is a harmless placeholder so
 * leftover references (if any) won't break the build.
 */
public class ShooterSubsystem extends SubsystemBase {
  private final RelativeEncoder ShooterRPMEncoder;
  private SparkClosedLoopController m_ShooterPID;

  private final double kP;
  private final double kI;
  private final double kD;
  private final double kMinOutput;
  private final double kMaxOutput;
  SparkMaxConfig config = new SparkMaxConfig();

  public enum SetShooterSpeed {
    ZeroSpeed, SlowSpeed, FarSpeed; //Modes - Really probably don't even need to exists (with the exception of ZERO and one other speed)
  }
    private volatile double slowRPM = 3420.0; // Default value, can be updated at runtime
    private volatile double farRPM = 3900.0; // Default value, can be updated at runtime

    //private final double value;

    //SetShooterSpeed(double value) {
    //  this.value = value;
    //}

    
    // public double getValue() {
    //   return value;
    // }

    public void setSlowRPM(double rpm) {
    slowRPM = rpm;
    //SmartDashboard.putNumber("Shooter/slowRPM", slowRPM);
    }
    public void setFarRPM(double rpm) {
    farRPM = rpm;
    //SmartDashboard.putNumber("Shooter/farRPM", farRPM);
    }
  public ShooterSubsystem() {
    ShooterRPMEncoder = ShooterMotor.getEncoder();
    m_ShooterPID = ShooterMotor.getClosedLoopController();

    ShooterMotor.configure(Configs.MAXSwerveModule.shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    this.kP = 0.000134;
    this.kI = 0.00;
    this.kD = 0.00;
    this.kMinOutput = 0.00;
    this.kMaxOutput = 1.00;

    config.closedLoop
      .p(kP)
      .i(kI)
      .d(kD)
      .velocityFF(0.00017857)
      .outputRange(kMinOutput, kMaxOutput);

      ShooterMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters
      );
      ShooterRPMEncoder.setPosition(0);
      
  }

  public void setClosedLoopTargetRPM(double rpm) {
    if (m_ShooterPID != null) {
      m_ShooterPID.setReference(rpm, com.revrobotics.spark.SparkBase.ControlType.kVelocity);
    }
  }

  public double getMeasuredRPM() {
    return ShooterRPMEncoder.getVelocity();
  }


    public static SparkMax ShooterMotor = new SparkMax(OtherMotors.ShooterMotorId, MotorType.kBrushless);

    public void setShooterSpeed(double speed){
        speed = DriveConstants.ShooterMotorSpeed;
    ShooterMotor.set(speed);
    }
    public void stopShooter() {
      ShooterMotor.stopMotor();
    }

    public double getShooterRPM() {
      return ShooterRPMEncoder.getVelocity();
    }

    public void setSpeed(SetShooterSpeed mode) {
      double target;
      switch (mode) {
        case SlowSpeed:
          target = slowRPM;
          break;
        case FarSpeed:
          target = farRPM;
          break;
        case ZeroSpeed:
        default:
          target = 0.0; //Zero Speed
          break;
      }
      m_ShooterPID.setReference(target, ControlType.kVelocity);
    }

    @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Velocity value: ", ShooterRPMEncoder.getVelocity());
  }
}
