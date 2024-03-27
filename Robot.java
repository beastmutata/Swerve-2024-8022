// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  //Defines Controller
  public PS5Controller controller = new PS5Controller(0);


  static final CANSparkMax SRXFlyWheel1 = new CANSparkMax(11, CANSparkLowLevel.MotorType.kBrushed);
  static final WPI_VictorSPX sparkFlyWheel2 = new WPI_VictorSPX(9);
  static final MotorControllerGroup FlywheelGroup = new MotorControllerGroup(SRXFlyWheel1, sparkFlyWheel2);
  //Defines all Spark motor controllers  

  static final CANSparkMax sparkIntake = new CANSparkMax(10, CANSparkLowLevel.MotorType.kBrushed);
  static final CANSparkMax sparkArm = new CANSparkMax(12, CANSparkLowLevel.MotorType.kBrushed);
  static final CANSparkMax sparkClimberLeft = new CANSparkMax(13, CANSparkLowLevel.MotorType.kBrushed);
  static final CANSparkMax sparkClimberRight = new CANSparkMax(14, CANSparkLowLevel.MotorType.kBrushed);
  static final MotorControllerGroup ClimberGroup = new MotorControllerGroup(sparkClimberLeft, sparkClimberRight);

  //defines limelight 
  private Limelight limelight = new Limelight(); 


  //defines all stuff for swerve 
  public Swerve drive_swerve = new Swerve(new int[] {1, 2, 0}, new int[] {3, 4, 1}, new int[] {5, 6, 2}, new int[] {7, 8, 3});
  static final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  public double drive_velocity = 1.0, turn_velocity = 1.00;
  public double joystick_threshold = 0.05, twist_threshold = 0.05;

  GenericEntry encoderEntry = Shuffleboard.getTab("idk").add("Reset Encoders", false).getEntry();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    //Sets up SparkMaxs



    Shuffleboard.getTab("te").add("gyro", gyro);
    gyro.calibrate();
  }

  @Override
  public void robotPeriodic() {


    if (encoderEntry.getBoolean(false)) {
      drive_swerve.reset();
      drive_swerve.update();
      SmartDashboard.putBoolean("test",true);
    }
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  //private static double old_heading = 0;

  @Override
  public void teleopPeriodic() {
    





    double[] angles = {45, -45, -45, 45}; //{45, -45, -45, 45};
    double[] speeds = {0, 0, 0, 0};

    double translate_x = Math.abs(controller.getLeftX()) > joystick_threshold ? controller.getLeftX() : 0;
    double translate_y = Math.abs(controller.getLeftY()) > joystick_threshold ? -controller.getLeftY() : 0;
    double rotation = (Math.abs(controller.getRightX()) > twist_threshold ? controller.getRightX() : 0) * -turn_velocity;
    // double right_y = controller.getRightY();

    double actual_heading = gyro.getAngle();
    

    // do math for the headings
    double drive_heading = (Math.atan2(translate_x, translate_y) * 180 / Math.PI);
    double drive_magnitude = Math.sqrt(Math.pow(translate_x, 2) + Math.pow(translate_y, 2)) * drive_velocity / Math.sqrt(2);
    double translation_heading = drive_heading - actual_heading; // field relative
    // for robot relative just set actual_heading to 0 (i.e. gyro always reads as 0)
    if (drive_magnitude >= 0.008) {
        angles = new double[] {
        translation_heading,
        translation_heading,
        translation_heading,
        translation_heading
      };
    }
    // else if (rotation == 0) angles = new double[] {
    //   old_heading,
    //   old_heading,
    //   old_heading,
    //   old_heading
    // };
    // old_heading = translation_heading;

    speeds = new double[] {
      drive_magnitude,
      drive_magnitude,
      drive_magnitude,
      drive_magnitude
    };

    double tk1 = 0.295, tk2 = 0.42, tk3 = 0.35;

    // modify the speeds based on the turning value
    if (drive_magnitude != 0) for (int i = 0; i != speeds.length; i++) {
      if (i <= 1) speeds[i] +=
        Math.cos(
          -translation_heading * Math.PI / 180 + (0.25 + .5 * i) * Math.PI) *
          (Math.pow(drive_magnitude, 0.2)) * rotation * tk1;
      else speeds[i] +=
        Math.sin(
          translation_heading * Math.PI / 180 + (-0.25 + .5 * i) * Math.PI) *
          (Math.pow(drive_magnitude, 0.2)) * rotation * tk1;
    } else {
      for (int i = 0; i != speeds.length; i++) {
        if (i % 2 == 0) speeds[i] = rotation * tk2;
        else speeds[i] = -rotation * tk2;
      }
    }

    // modify the angles based on the turning value
    if (rotation != 0 && drive_magnitude != 0) for (int i = 0; i != angles.length; i++) {
      // if (i <= 1) angles[i] +=
      //   Math.asin(Math.cos(
      //     translation_heading * Math.PI / 360 + (-0.375 + .25 * i) * Math.PI) *
      //     (1 - Math.pow(drive_magnitude, 0.4))) * 360 / Math.PI * rotation / Math.abs(rotation);
      // else angles[i] +=
      //   Math.asin(Math.sin(
      //     translation_heading * Math.PI / 360 + (0.625 + .25 * i) * Math.PI) *
      //     (1 - Math.pow(drive_magnitude, 0.4))) * 360 / Math.PI * rotation / Math.abs(rotation);
      if (i <= 1) angles[i] +=
        Math.asin(Math.cos(
          -translation_heading * Math.PI / 180 + (-0.25 + .5 * i) * Math.PI) *
          (1 - Math.pow(drive_magnitude, 0.4))) * 180 / Math.PI * rotation / Math.abs(rotation) * tk3;
      else angles[i] +=
        Math.asin(Math.sin(
          translation_heading * Math.PI / 180 + (0.25 + .5 * i) * Math.PI) *
          (1 - Math.pow(drive_magnitude, 0.4))) * 180 / Math.PI * rotation / Math.abs(rotation) * tk3;
    } else {
      for (double angle : angles) {
        if (angle == 0 || angle == 1) angle = 45;
        else angle = -45;
      }
    }
    
    drive_swerve.setHeadings(angles);
    drive_swerve.calculateAlignment();
    drive_swerve.setSpeeds(speeds);


    if (controller.getRawButton(1)) {
      gyro.calibrate();
    }


     if (controller.getCrossButton()) {
      FlywheelGroup.set(1);
    } else if (controller.getCircleButton()) {
      FlywheelGroup.set(0);
    }

    if (controller.getRawButton(11)) {
      ClimberGroup.set(1);
    } else if (controller.getRawButton(12)) {
      ClimberGroup.set(-1);
    }
    else {
      ClimberGroup.set(0);
    }


    if (controller.getR1Button()) {
      sparkIntake.set(-.50);
    }
    else {
     sparkIntake.set(0);
    }




     if (controller.getL2Button()) {
      sparkArm.set(1);
    } else if (controller.getL1Button()) {
      sparkArm.set(-1);
    }
    else {
      sparkArm.set(0);
    }


  }

  

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
