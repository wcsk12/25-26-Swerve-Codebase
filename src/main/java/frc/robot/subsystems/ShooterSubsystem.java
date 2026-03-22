package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OtherMotors;
import frc.robot.Robot;

/**
 * Placeholder ShooterSubsystem left in the tree for compatibility during the revert.
 * This class intentionally does not own hardware. The project has been reverted to use
 * OtherMotorsSubsystem for motor ownership; this file is a harmless placeholder so
 * leftover references (if any) won't break the build.
 */
public class ShooterSubsystem extends SubsystemBase {
  public ShooterSubsystem() {}

    public static SparkMax ShooterMotor = new SparkMax(OtherMotors.ShooterMotorId, MotorType.kBrushless);

    public void setShooterSpeed(double speed){
    //   if (Robot.limelight_id() == 10 || Robot.limelight_id() == 26){
    //     double ty = Robot.limelight_range_proportional();
    //     if (0.95 > Math.abs((ty+20.5)/41) && ty != 0)  {
    //     speed = (ty*10);
    //     System.out.println("Works");
    //     System.out.println("Ty : " + ty);
    //   }
    //   else {
    speed = DriveConstants.ShooterMotorSpeed;
      //}
    //}
    ShooterMotor.set(speed);
    }
    @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
