package frc.robot;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Joystick;

public class Spark {
    private CANSparkMax sparkMax;
    private Joystick joystick;
    private int deviceID; // SparkMax device ID
    private int joystickPort; // Joystick port
    private double speedWhenPressed = 0.5; // Speed to set when the button is pressed

    // Constructor
    public Spark(int deviceID, int joystickPort) {
        this.deviceID = deviceID;
        this.joystickPort = joystickPort;
        this.sparkMax = new CANSparkMax(this.deviceID, CANSparkLowLevel.MotorType.kBrushless); // Initialize SparkMax
        this.joystick = new Joystick(this.joystickPort); // Initialize joystick
    }

    // Method to control the SparkMax with a joystick button
    public void controlWithButton(int buttonNumber) {
        boolean buttonPressed = this.joystick.getRawButton(buttonNumber);
        if (buttonPressed) {
            this.sparkMax.set(speedWhenPressed); // Set motor speed when button is pressed
        } else {
            this.sparkMax.set(0); // Stop motor when button is not pressed
        }
    }

    public void controlWithJoystick() {
        double speed = this.joystick.getY(); // Get Y-axis value
        this.sparkMax.set(speed); // Set motor speed
    }
}