package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import com.ctre.phoenix6.hardware.*;
//import com.ctre.phoenix6.hardware.TalonFX;

public class Swerve {
	public Module front_left, front_right, rear_left, rear_right;

	private PIDController drivePID = new PIDController(0, 0, 0, 0.02);
	private static final double drive_precision = 0;

	public Swerve(int[] front_left_ids, int[] front_right_ids, int[] rear_left_ids, int[] rear_right_ids) {

		SmartDashboard.putData("DrivePID", drivePID);

		// Create modules
		front_left = new Module(front_left_ids[0], front_left_ids[1], front_left_ids[2]);
		front_right = new Module(front_right_ids[0], front_right_ids[1], front_right_ids[2]);
		rear_left = new Module(rear_left_ids[0], rear_left_ids[1], rear_left_ids[2]);
		rear_right = new Module(rear_right_ids[0], rear_right_ids[1], rear_right_ids[2]);

		// PID configurations
		drivePID.setTolerance(drive_precision);
	}

	public void reset() {
		front_left.reset();
		front_right.reset();
		rear_left.reset();
		rear_right.reset();
	}

	public void update() {
		front_left.update();
		front_right.update();
		rear_left.update();
		rear_right.update();
	}

	public void setHeadings(double[] headings) {
		front_left.setHeading(headings[0]);
		front_right.setHeading(headings[1]);
		rear_left.setHeading(headings[2]);
		rear_right.setHeading(headings[3]);
	}

	public void setSpeeds(double[] speeds) {
		front_left.spin(speeds[0]);
		front_right.spin(speeds[1]);
		rear_left.spin(speeds[2]);
		rear_right.spin(speeds[3]);
	}

	public void calculateAlignment() {
		front_left.calculateAlignment();
		front_right.calculateAlignment();
		rear_left.calculateAlignment();
		rear_right.calculateAlignment();
	}

	private static int id = 0;

	public class Module {
		public TalonFX alignment_motor, drive_motor;
		public DutyCycleEncoder rotation_encoder;
		private double idealHeading;
		// public PIDController alignmentPID;

		int meid;

		public Module(int alignment_id, int drive_id, int encoder_channel) {

			// id setting
			this.alignment_motor = new TalonFX(alignment_id);
			this.drive_motor = new TalonFX(drive_id);
			this.rotation_encoder = new DutyCycleEncoder(new DigitalInput(encoder_channel));

			// encoder config
			this.rotation_encoder.setDistancePerRotation(-360);
			this.rotation_encoder.setPositionOffset(((double) this.alignment_motor.configGetCustomParam(1)) / 1000);

			// set brake
			this.alignment_motor.setNeutralMode(NeutralMode.Brake);
			this.drive_motor.setNeutralMode(NeutralMode.Coast);
			// set reversed
			this.drive_motor.setInverted(false);
			this.alignment_motor.setInverted(true);

			// pid
			// this.alignmentPID = new PIDController(alignment_kp, alignment_ki, alignment_kd, 0.02);
			// this.alignmentPID.setTolerance(alignment_precision);

			// SmartDashboard.putData("AlignPID" + id, alignmentPID);
			// SmartDashboard.putData("Encoder" + id, rotation_encoder);
			Shuffleboard.getTab("te").add("Encoder" + id, rotation_encoder);
		
			meid = id;
			id++;
		}

		public void update() {
			// SmartDashboard.putNumber("encoderDistance" + meid, rotation_encoder.getDistance());
		}

		//private static final double alignment_kp = 1.214, alignment_ki = 0.0095, alignment_kd = 0.768;
		private static final double alignment_kp = 1.214, alignment_ki = 0.000095, alignment_kd = .768;
		private double pi = 0, pd = 0;

		/* 
		 * Calculates and sets velocity to the alignment motor
		 */
		 
		public void calculateAlignment() {
			if (this.idealHeading == 404) {
				alignment_motor.neutralOutput();
			} else {
				double offset = this.idealHeading % 404 - this.heading();
				if (offset > 180) offset -= 360;
				else if (offset < -180) offset += 360;
				SmartDashboard.putNumber("testing offset" + this.meid, offset);
				double error = offset / 180;
				this.pi += error;
				if (error == 0 || Math.abs(error) >= 30 || error * this.pd < 0) this.pi = 0;
				double speed = error * alignment_kp + this.pi * alignment_ki + (error - this.pd) * alignment_kd;
				this.pd = error;
				if (meid == 0) SmartDashboard.putNumber("speed test", speed);
				alignment_motor.set(TalonFXControlMode.PercentOutput, speed);
			}
		}
        
		/* 
		private double alignment_tolerance = 15.0;
		public void calculateAlignment() {
			// Neutral output if heading is uninitialized
			if (this.idealHeading == 404) {
				alignment_motor.neutralOutput();
				return;
			}
			
			double currentHeading = this.heading();
			double desiredHeading = this.idealHeading % 360; // Ensure desired heading is within 0-359 range
			
			double offset = desiredHeading - currentHeading;
			
			// Adjust for the shortest rotational direction
			if (offset > 180) offset -= 360;
			else if (offset < -180) offset += 360;
			
			// Apply tolerance check
			if (Math.abs(offset) <= alignment_tolerance) {
				alignment_motor.neutralOutput(); // Within tolerance, stop adjusting
				return;
			}
			
			// Calculate PID control manually or through a PIDController instance
			double error = offset / 180; // Normalize error for calculation
			this.pi += error * alignment_ki;
			double pdComponent = (error - this.pd) * alignment_kd;
			this.pd = error;
			double speed = error * alignment_kp + this.pi + pdComponent;
			
			// Apply the calculated speed to the alignment motor
			alignment_motor.set(TalonFXControlMode.PercentOutput, speed);
			
			// Debugging output
			SmartDashboard.putNumber("Module " + meid + " Offset", offset);
			SmartDashboard.putNumber("Module " + meid + " Speed", speed);
		}
		*/
		
		/**
		 * Sets the angle to which the module will attempt to align
		 * 
		 */
		public Module setHeading(double angle) {
			// angle *= 180 / Math.PI;
			this.idealHeading = angle;
			SmartDashboard.putNumber("testing heading" + this.meid, this.idealHeading % 360);
			return this;
		}

		// public Module spinFor(double rotations, double speed) {
		// 	return this;
		// }

		public void reset() {
			this.rotation_encoder.reset();
			this.alignment_motor.configSetCustomParam((int) (this.rotation_encoder.getPositionOffset() * 1000), 1);

		}

		public Module spin(double speed) {
			this.drive_motor.set(ControlMode.PercentOutput, speed);
			return this;
		}

		// public double alignment() { return this.rotation_encoder.getDistance(); }

		public double heading() {
			// double angle = this.rotation_encoder.getDistance() % 360;
			return this.rotation_encoder.getDistance() % 360;
		}
	}
}