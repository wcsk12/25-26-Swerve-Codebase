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
    //
    
}
