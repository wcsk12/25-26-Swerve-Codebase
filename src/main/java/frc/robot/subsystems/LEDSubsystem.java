package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

    // Init PWM
    private final PWM Blinkinled;

    // Init Colors for Blinkin via Voltage
    private final double red = 0.59;
    private final double green = 0.71;
    private final double blue = -0.15;

  public LEDSubsystem(int BlinkinID) {
    // Initialize the driver with the name from your config
    Blinkinled = new PWM(BlinkinID);
    setPattern(blue);
    }

    @Override
    public void periodic() {
        // No periodic work required for LEDs by default
    }

    public void setPattern(double pattern) {
        Blinkinled.setSpeed(pattern);
    }
    /* Create methods for each color option */
    public void setBlue(){
        Blinkinled.setSpeed(blue);
    }

    public void setRed(){
        Blinkinled.setSpeed(red);
    }

    public void setGreen(){
        Blinkinled.setSpeed(green);
    }


//   public void updateLEDs(boolean buttonA, boolean buttonB) {
//         if (buttonA) {
//             blinkinLedDriver.setPattern(GREEN);
//         } else if (buttonB) {
//             blinkinLedDriver.setPattern(RED);
//         } else {
//             blinkinLedDriver.setPattern(BLUE);
//         }
//     }
}