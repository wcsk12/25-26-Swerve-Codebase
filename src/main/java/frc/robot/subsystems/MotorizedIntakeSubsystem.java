// package frc.robot.subsystems;

// // ============================== Spark Related Imports ============================== \\
// import com.revrobotics.spark.SparkLowLevel.MotorType;

// import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// import com.revrobotics.PersistMode;
// import com.revrobotics.ResetMode;
// import com.revrobotics.RelativeEncoder;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.OtherMotors;
// import frc.robot.Constants.DriveConstants;

// //import com.revrobotics.AbsoluteEncoder;
// //import com.revrobotics.RelativeEncoder;

//     public class MotorizedIntakeSubsystem extends SubsystemBase {
//   // initialize Motor
//     public SparkMax IntakeMotor = new SparkMax(OtherMotors.IntakeMotorId, MotorType.kBrushless);
//     private RelativeEncoder intakeRelative;
//     private SparkClosedLoopController m_IntakePID;
//     private SparkMaxConfig intakeMotorConfig;

//     public enum IntakePositions{
//       zero(0),
//       down(8);

//       private final double value;

//       IntakePositions(double value){
//         this.value = value;
//       }

//       public double getValue(){
//         return value;
//       }
//     }
      
//     public MotorizedIntakeSubsystem() {
//       intakeRelative = IntakeMotor.getEncoder();
//       m_IntakePID = IntakeMotor.getClosedLoopController();
//       intakeMotorConfig = new SparkMaxConfig();

//       //This section of code my cause future problems!!!
//       intakeMotorConfig.closedLoop
//           .p(0.05)
//           .i(0)
//           .d(0)
//           .outputRange(-1, 1);
//       intakeMotorConfig
//           .inverted(false)
//           .smartCurrentLimit(30)
//           .idleMode(IdleMode.kBrake);
//       IntakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

//       intakeRelative.setPosition(0);
//     }

//     public void SetIntakeSpeed(double speed) { // This is used to set motor speed.
//       IntakeMotor.set(speed);
//     }

//     public void stopIntake(double speed) {
//       IntakeMotor.stopMotor();
//     }

//     public void setPosition(IntakePositions position){
//       m_IntakePID.setReference(position.getValue(), ControlType.kPosition);
//     }

//     @Override
//     public void periodic(){
//       SmartDashboard.putNumber("Intake Encoder Value", intakeRelative.getPosition());
//     }
// }

