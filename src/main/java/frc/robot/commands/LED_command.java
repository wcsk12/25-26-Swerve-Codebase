package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Command that updates the LED pattern based on a motor's power.
 * Runs continuously (isFinished() == false) and should be used as a default
 * command for the LED subsystem so it updates automatically.
 */
public class LED_command extends Command {
    private final LEDSubsystem ledSubsystem;
    // Lower threshold so small alignment outputs are detected. We'll also
    // log the observed power for diagnostics.
    private final double threshold = 2.0; // Threshold to determine if the motor is running
    private final double trenchThreshold = 2.4;
    private final double farThreshold = 3;
    private final double offset = .1;

    public LED_command(LEDSubsystem ledSubsystem) {
        this.ledSubsystem = ledSubsystem;
        // The LED subsystem is required because this command updates it.
        addRequirements(ledSubsystem);
    }

    @Override
    public void execute() {
        double power = Math.abs(Robot.limelight_distance_proportional()); // read from the motor subsystem
        // Diagnostic log to observe motor output during Limelight alignment
        System.out.println("[LED_command] motor power=" + power);
        if (power < threshold - offset){
            // Motor is running, set blue (It is too close)
            ledSubsystem.setPattern(-0.15); // fading blue
        } else if (power > threshold - offset && power < threshold + offset){
            // Motor stopped, set green
            ledSubsystem.setPattern(0.71); // green
        } else if (power > trenchThreshold - offset && power < trenchThreshold + offset) {
            ledSubsystem.setPattern(0.91); // purple
        } else if (power > farThreshold - offset && power < farThreshold + offset) {
            ledSubsystem.setPattern(0); // Light Blue
        } else {
            // Motor is running, set red (It is too far)
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