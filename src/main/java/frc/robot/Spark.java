/*package frc.robot;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.Joystick;

public class Spark {
    private CANSparkMax sparkMax;
    private PS5Controller Controller;
    private int deviceID; // SparkMax device ID
    private int joystickPort; // Joystick port
    private double speedWhenPressed = 1; // Speed to set when the button is pressed

    // Constructor
    public Spark(int deviceID, int joystickPort) {
        this.deviceID = deviceID;
        this.joystickPort = joystickPort;
        this.sparkMax = new CANSparkMax(this.deviceID, CANSparkLowLevel.MotorType.kBrushless); // Initialize SparkMax
        this.Controller = new PS5Controller(this.joystickPort); // Initialize joystick
    }

    // Method to control the SparkMax with a joystick button
    public void controlWithButton(int buttonNumber) {
        boolean buttonPressed = this.Controller.getR2Button();
        if (buttonPressed) {
            this.sparkMax.set(speedWhenPressed); // Set motor speed when button is pressed
        } else {
            this.sparkMax.set(0); // Stop motor when button is not pressed
        }
    }

    public void controlWithJoystick() {
        double speed = this.Controller.getRightY(); // Get Y-axis value
        this.sparkMax.set(speed); // Set motor speed
    }

    public void set() {
        this.sparkMax.set(1); // Set motor speed
    }

    public void controlWithPOVDifferentDirections() {
    int povAngle = this.Controller.getPOV(); // Get the POV angle
    
    if (povAngle == 0) {
        this.sparkMax.set(speedWhenPressed); // Set motor speed forward when POV is 90 degrees
    } else if (povAngle == 180) {
        this.sparkMax.set(-speedWhenPressed); // Set motor speed in reverse when POV is 180 degrees
    } else {
        this.sparkMax.set(0); // Stop motor when the POV is not at 90 or 180 degrees
    }
} 
  }
*/