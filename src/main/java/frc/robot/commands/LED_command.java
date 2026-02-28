package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

/**
 * Minimal LED default command — keeps LEDs in the default blue pattern while
 * scheduled. This avoids referencing external subsystems and provides a
 * safe default behavior until a more advanced LED command is implemented.
 */
public class LED_command extends Command {
    private final LEDSubsystem ledSubsystem;

    public LED_command(LEDSubsystem ledSubsystem) {
        this.ledSubsystem = ledSubsystem;
        addRequirements(ledSubsystem);
    }

    @Override
    public void initialize() {}

        @Override
        public void execute() {
            // Change LED color based on AutoAlign status reported to SmartDashboard
            String status = edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getString("AutoAlign/status", "no_results");
            switch (status) {
                case "ended":
                    // AutoAlign completed successfully -> aligned
                    ledSubsystem.setGreen();
                    break;
                case "running":
                    // Currently aligning
                    ledSubsystem.setRed();
                    break;
                default:
                    // No results or other states -> default blue
                    ledSubsystem.setBlue();
                    break;
            }
        }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
