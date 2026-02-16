package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  
    //Init PWM for Blinkin LED
    private final PWM Blinkinled;

    //Init Colors for Blinken via voltage 
    private final double red= 0.59;
    private final double green = 0.71;
    private final double blue = -0.15;

    public LEDSubsystem(int BlinkinID) {
        Blinkinled = new PWM(BlinkinID); //PWM port for Blinkin LED
        setPattern(blue); //Set default color to blue
    }

     @Override
    public void periodic() {
        // No periodic work required for LEDs by default
    }

    public void setPattern(double pattern){
        Blinkinled.setSpeed(pattern); //Set the Blinkin LED pattern based on voltage
    }

    /*Create methods for each color option */
    public void setBlue(){
        setPattern(blue);
    }
    
    public void setRed(){
        setPattern(red);
    }

    public void setGreen(){
        setPattern(green);
    }
 
}
