package frc.robot.generated;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class T16000MController extends Joystick {
    public T16000MController(int port) {
        super(port);
    }
    
    // Example method for left X-axis; adjust the axis number as needed
    public double getLeftX() {
        return getRawAxis(0);
    }
    
    // Example method for left Y-axis; adjust the axis number as needed
    public double getLeftY() {
        return getRawAxis(1);
    }
    
    // Add more methods for other axes/buttons if needed
    public JoystickButton T() {
        // Map to the correct button number for your controller
        return new JoystickButton(this, 1);
    }
    public JoystickButton D() {
        // Map to the correct button number for your controller
        return new JoystickButton(this, 2);
    }
    public JoystickButton L() {
        // Map to the correct button number for your controller
        return new JoystickButton(this, 3);
    }
    public JoystickButton R() {
        // Map to the correct button number for your controller
        return new JoystickButton(this, 4);
    }
}