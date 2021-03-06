package org.usfirst.frc3534.RobotBasic;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	public static WPI_TalonSRX frontRightMotor;
	public static WPI_TalonSRX backRightMotor;
	public static WPI_TalonSRX frontLeftMotor;
	public static WPI_TalonSRX backLeftMotor;

	public static WPI_TalonSRX shooter;

	public static WPI_TalonSRX pidArm;

	public static SpeedControllerGroup rightSideMotors;
	public static SpeedControllerGroup leftSideMotors;

	public static AHRS navx;

	public static Solenoid cylinder;

	//public static DigitalInput pneumaticSensor;
	//COME BACK TO THIS

	public static final double wheelBase_width = 23.375;
	public static final double robotMaxVeloctiy = 175; // inches per second
	public static final double minMoveVelocity = .275; 

	// Wheel Encoder Calculations
	public static final int countsPerRevEncoders = 1440; // 1440 if plugged into talon. 360 if directly into the roborio; just go with, it its weird
	public static final double wheelDiameter = 6.00; // measured in inches
	public static final double inchesPerCountMultiplier = wheelDiameter * Math.PI / countsPerRevEncoders;

	public static void init() {

		frontRightMotor = new WPI_TalonSRX(5);
		frontRightMotor.set(ControlMode.PercentOutput, 0);
		frontRightMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		//frontRightMotor.config_kF(1, .07652 * 10);
		//frontRightMotor.config_kP(1, .0625);
		frontRightMotor.setNeutralMode(NeutralMode.Brake);

		backRightMotor = new WPI_TalonSRX(6);
		backRightMotor.set(ControlMode.PercentOutput, 0);
		//backRightMotor.follow(frontRightMotor);
		//backRightMotor.config_kF(1, .07652 * 10);
		//backRightMotor.config_kP(1, .0625);
		backRightMotor.setNeutralMode(NeutralMode.Brake);

		frontLeftMotor = new WPI_TalonSRX(1);
		frontLeftMotor.set(ControlMode.PercentOutput, 0);
		frontLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		//frontLeftMotor.config_kF(1, .07652 * 10);
		//frontLeftMotor.config_kP(1, .0625);
		frontLeftMotor.setNeutralMode(NeutralMode.Brake);

		backLeftMotor = new WPI_TalonSRX(2);
		backLeftMotor.set(ControlMode.PercentOutput, 0);
		//backLeftMotor.follow(frontLeftMotor);
		//backLeftMotor.config_kF(1, .07652 * 10);
		//backLeftMotor.config_kP(1, .0625);
		backLeftMotor.setNeutralMode(NeutralMode.Brake);

		rightSideMotors = new SpeedControllerGroup(frontRightMotor, backRightMotor);
		leftSideMotors = new SpeedControllerGroup(frontLeftMotor, backLeftMotor);
		
		//pidArm.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);

		navx = new AHRS(SerialPort.Port.kMXP);

		//pneumaticSensor = new DigitalInput(0);
		
		

		
	}
}