package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
//import frc.robot.subsystems.Motor3_SUB_HAO;

/**
 * Command that updates the LED pattern based on a motor's power.
 * Runs continuously (isFinished() == false) and should be used as a default
 * command for the LED subsystem so it updates automatically.
 */
public class LED_command extends Command { 
    private final LEDSubsystem ledSubsystem;
    private final Motor3_SUB_HAO motorSubsystem;
    //Lower threshold so small alignment outputs are dectected. We'll also 
    //log the observed power for diagnostics.
    private final double threshold= 0.02;// Threshold to determine if the motor is running

    public LED_command(LEDSubsystem ledSubsystem, Motor3_SUB_HAO motorSubsystem) {
        this.ledSubsystem = ledSubsystem;
        this.motorSubsystem = motorSubsystem;
        addRequirements(ledSubsystem); // Declare subsystem dependencies
    }
    
    @Override
    public void execute() {
        double power = Math.abs(motorSubsystem.getMotorPower()); // read from the motor subsystem
        // Diagnostic log to observe motor output during Limelight alignment
        System.out.println("[LED_command] motor power=" + power);
        if (power > threshold) {
            // Motor is running, set green
            ledSubsystem.setPattern(0.71); // green
        } else {
            // Motor stopped, set red
            ledSubsystem.setPattern(0.59); // red
        }

     }
    
     @Override
    public void end(boolean interrupted) {
          // Optionally set a safe/default color when the command ends
        // ledSubsystem.setBlue();
    }
     @Override
    public boolean isFinished() {
        return false; // run until interrupted
    }
}